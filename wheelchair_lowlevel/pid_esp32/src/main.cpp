#include <Arduino.h>
#include "WheelchairController.h"
#include "EncoderReader.h"
#include "CmdVelReceiver.h"
#include "PidController.h"
#include "um7_parser.h"
#include <math.h>
#include "LookupTable.h"
#include "SensorPublisher.h"

// =========================
// GPIO PINS
// =========================

// Wheelchair I2C (MCP4728)
const int SDA_PIN = 6;
const int SCL_PIN = 7;

// Encoders
const int LEFT_ENCODER_A  = 35;
const int LEFT_ENCODER_B  = 36;
const int RIGHT_ENCODER_A = 47;
const int RIGHT_ENCODER_B = 48;

// Encoder Sign
const int LEFT_SIGN  = 1;
const int RIGHT_SIGN = -1;

// =========================
// Encoder & wheelchair params
// =========================
const float LEFT_CPR         = 715.0f;
const float RIGHT_CPR        = 1200.0f;
const float WHEEL_RADIUS     = 0.171f;
const float WHEEL_SEPARATION = 0.575f;

// =========================
// Voltage settings
// =========================
const float Y_NEUTRAL = 2.689f;
const float X_NEUTRAL = 2.750f;

const float Y_MIN_V = 1.0f;
const float Y_MAX_V = 4.2f;
const float X_MIN_V = 1.0f;
const float X_MAX_V = 4.2f;

// =========================
// Timing
// =========================
const unsigned long CMD_TIMEOUT_MS      = 1000;
const unsigned long ENCODER_INTERVAL_MS = 20;   // 50 Hz
const unsigned long LOG_INTERVAL_MS     = 100;  // 10 Hz
const unsigned long IMU_TIMEOUT_MS      = 200;

// =========================
// Linear Mapping Settings
// =========================
const float MAX_LINEAR_CMD  = 1.0f;   // m/s
const float MAX_ANGULAR_CMD = 1.0f;   // rad/s

const float Y_SPAN_V = 0.90f;
const float X_SPAN_V = 0.70f;

const float Y_DEADBAND_V = 0.05f;
const float X_DEADBAND_V = 0.05f;

// =========================
// X trim settings
// =========================
const float X_BASE_TRIM        = 0.002f;
const float X_TRIM_START_SPEED = 0.20f;   // m/s
const float X_TRIM_GAIN        = 0.0215f;
const float X_TRIM_MAX         = 0.0086f;

// =========================
// PID Settings
// =========================
const float PIDY_KP = 0.02f;
const float PIDY_KI = 0.001f;
const float PIDY_KD = 0.0f;

// Turn-rate PID
const float PIDW_KP = 0.038f;
const float PIDW_KI = 0.001f;
const float PIDW_KD = 0.0f;

// Heading-hold PID
const float PIDH_KP = 1.20f;
const float PIDH_KI = 0.0f;
const float PIDH_KD = 0.0f;

// Output correction limits (V)
const float Y_PID_MIN = -0.20f;
const float Y_PID_MAX =  0.20f;

const float X_PID_MIN = -0.18f;
const float X_PID_MAX =  0.18f;

// Heading-hold outer loop output limits (rad/s)
const float HEADING_W_REF_MIN = -0.20f;
const float HEADING_W_REF_MAX =  0.20f;

// Enable thresholds
const float SPEED_ENABLE_THRESHOLD = 0.02f;
const float TURN_ENABLE_THRESHOLD  = 0.02f;

// Debug flag
const bool ENABLE_DEBUG_PUBLISH = false;

// =========================
// Objects
// =========================
CmdVelReceiver cmdVelReceiver;
SensorPublisher sensorPublisher;

EncoderReader encoderReader(
    LEFT_ENCODER_A, LEFT_ENCODER_B,
    RIGHT_ENCODER_A, RIGHT_ENCODER_B,
    LEFT_SIGN, RIGHT_SIGN,
    LEFT_CPR, RIGHT_CPR,
    WHEEL_RADIUS, WHEEL_SEPARATION
);

WheelchairController wheelchair(
    SDA_PIN, SCL_PIN,
    MCP4728_CHANNEL_A, MCP4728_CHANNEL_B,
    Y_NEUTRAL, X_NEUTRAL,
    Y_MIN_V, Y_MAX_V,
    X_MIN_V, X_MAX_V
);

PidController pidY;
PidController pidW;
PidController pidHeading;

// =========================
// Interrupts
// =========================
void IRAM_ATTR leftISR() {
    encoderReader.handleLeftISR();
}

void IRAM_ATTR rightISR() {
    encoderReader.handleRightISR();
}

// =========================
// Encoder snapshot state
// =========================
EncoderSnapshot prev_snap;
EncoderSnapshot curr_snap;
bool has_prev_snap = false;

// =========================
// Timing state
// =========================
unsigned long last_log_time = 0;
unsigned long last_imu_time = 0;

// =========================
// Serial input line buffer
// =========================
String serial_line = "";

// =========================
// IMU state
// =========================
ImuSample imu_sample;
bool imu_ok = false;

// =========================
// Heading hold state
// =========================
float current_yaw = 0.0f;
float heading_target = 0.0f;
float heading_hold_w_ref = 0.0f;
bool heading_initialized = false;
bool heading_mode_active = false;

// =========================
// Reference states
// =========================
float v_ref = 0.0f;
float w_ref = 0.0f;

// =========================
// Measured states
// =========================
float v_meas = 0.0f;
float w_meas_encoder = 0.0f;
float w_meas_pid = 0.0f;
float imu_wz_ctrl = 0.0f;

// =========================
// Final output voltages
// =========================
float y_ff = Y_NEUTRAL;
float x_ff = X_NEUTRAL;

float y_corr = 0.0f;
float x_corr = 0.0f;

float y_cmd = Y_NEUTRAL;
float x_cmd = X_NEUTRAL;

void resetControllers() {
    pidY.reset();
    pidW.reset();
    pidHeading.reset();
    y_corr = 0.0f;
    x_corr = 0.0f;
    heading_hold_w_ref = 0.0f;
}

float clampf_local(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

float wrapAngle(float a) {
    while (a > PI)  a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

float quaternionToYaw(const ImuSample& s) {
    const float siny_cosp = 2.0f * (s.qw * s.qz + s.qx * s.qy);
    const float cosy_cosp = 1.0f - 2.0f * (s.qy * s.qy + s.qz * s.qz);
    return atan2f(siny_cosp, cosy_cosp);
}

float computeXSpeedTrim(float v_cmd) {
    float trim = 0.0f;

    if (v_cmd > X_TRIM_START_SPEED) {
        trim = X_TRIM_GAIN * (v_cmd - X_TRIM_START_SPEED);
    } else if (v_cmd < -X_TRIM_START_SPEED) {
        trim = X_TRIM_GAIN * (v_cmd + X_TRIM_START_SPEED);
    }

    return clampf_local(trim, -X_TRIM_MAX, X_TRIM_MAX);
}

float computeXTotalTrim(float v_cmd, float w_cmd) {
    const bool straight_motion = 
        (fabsf(v_cmd) > SPEED_ENABLE_THRESHOLD) &&
        (fabsf(w_cmd) <= TURN_ENABLE_THRESHOLD);

        if(!straight_motion)    return 0.0f;
        float trim = 0.0f;

        if(v_cmd > 0.0f)    trim += X_BASE_TRIM;
        else if(v_cmd < 0.0f)   trim -= X_BASE_TRIM;

        trim += computeXSpeedTrim(v_cmd);
        return clampf_local(trim, -X_TRIM_MAX, X_TRIM_MAX);
}

void handleSerialInput() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r') continue;

        if (c == '\n') {
            if (serial_line.length() > 0) {
                if (serial_line.startsWith("<")) {
                    cmdVelReceiver.processLine(serial_line);
                }
            }
            serial_line = "";
        } else {
            serial_line += c;
            if (serial_line.length() > 120) {
                serial_line = "";
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    cmdVelReceiver.begin();
    sensorPublisher.begin();
    encoderReader.begin();
    um7_begin();

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightISR, CHANGE);

    if (!wheelchair.begin()) {
        Serial.println("MCP4728 not found");
        while (1) delay(10);
    }

    wheelchair.setCommands(MAX_LINEAR_CMD, MAX_ANGULAR_CMD);
    wheelchair.setVoltageSpans(Y_SPAN_V, X_SPAN_V);
    wheelchair.setDeadbands(Y_DEADBAND_V, X_DEADBAND_V);
    wheelchair.setNeutral();

    pidY.begin(PIDY_KP, PIDY_KI, PIDY_KD, Y_PID_MIN, Y_PID_MAX);
    pidW.begin(PIDW_KP, PIDW_KI, PIDW_KD, X_PID_MIN, X_PID_MAX);
    pidHeading.begin(PIDH_KP, PIDH_KI, PIDH_KD, HEADING_W_REF_MIN, HEADING_W_REF_MAX);

    last_log_time = millis();
    last_imu_time = millis();

    Serial.println("SENSORS_READY");
}

// ===================== MAIN LOOP =====================

void loop() {

    // 1. Read incoming cmd_vel packets from serial
    handleSerialInput();

    // 2. Safety timeout
    if (millis() - cmdVelReceiver.getLastCmdMs() > CMD_TIMEOUT_MS) {
        cmdVelReceiver.setZero();
        resetControllers();
        heading_initialized = false;
        heading_mode_active = false;
    }

    // 3. Read references
    v_ref = cmdVelReceiver.getVRef();

    // ROS2: +angular.z = left turn
    // Wheelchair X voltage: + = right turn
    w_ref = -cmdVelReceiver.getWRef();

    // 4. Update IMU
    um7_update();
    if (um7_get_sample(imu_sample)) {
        imu_ok = imu_sample.valid;
        if (imu_ok) {
            imu_wz_ctrl = -imu_sample.wz;
            current_yaw = -quaternionToYaw(imu_sample);
            last_imu_time = millis();
        }
    }

    if (millis() - last_imu_time > IMU_TIMEOUT_MS) {
        imu_ok = false;
    }

    // 5. Pure neutral when command is effectively zero
    if (fabsf(v_ref) <= SPEED_ENABLE_THRESHOLD &&
        fabsf(w_ref) <= TURN_ENABLE_THRESHOLD) {
        resetControllers();
        heading_initialized = false;
        heading_mode_active = false;
        wheelchair.setNeutral();

        if (millis() - last_log_time > LOG_INTERVAL_MS) {
            last_log_time = millis();

            sensorPublisher.publishEncoder(
                encoderReader.getLeftCount(),
                encoderReader.getRightCount(),
                millis()
            );

            if (imu_ok) {
                sensorPublisher.publishImu(imu_sample);
            }
        }
        return;
    }

    // 6. Motion mode selection
    const bool straight_mode =
        (fabsf(v_ref) > SPEED_ENABLE_THRESHOLD) &&
        (fabsf(w_ref) <= TURN_ENABLE_THRESHOLD) &&
        imu_ok;

    if (straight_mode) {
        if (!heading_mode_active) {
            pidW.reset();
            pidHeading.reset();
            heading_initialized = false;
            heading_mode_active = true;
        }

        if (!heading_initialized) {
            heading_target = current_yaw;
            heading_initialized = true;
        }
    } else {
        if (heading_mode_active) {
            pidW.reset();
            pidHeading.reset();
            heading_mode_active = false;
        }
        heading_initialized = false;
        heading_hold_w_ref = 0.0f;
    }

    // 7. Update encoder-based velocity and run control
    if (encoderReader.readSnapshot(curr_snap, ENCODER_INTERVAL_MS)) {
        if (has_prev_snap) {
            encoderReader.updateVelocitiesFromSnapshot(prev_snap, curr_snap);

            float dt = (curr_snap.time_ms - prev_snap.time_ms) / 1000.0f;
            v_meas = encoderReader.getVBody();
            w_meas_encoder = encoderReader.getWBody();

            if (imu_ok)
                w_meas_pid = imu_wz_ctrl;
            else
                w_meas_pid = -w_meas_encoder;

            // Feedforward from linear mapping
            wheelchair.commandToVoltage(v_ref, w_ref, y_ff, x_ff);

            // Add trim only during motion
            x_ff += computeXTotalTrim(v_ref, w_ref);

            // Y-axis PID
            if (fabsf(v_ref) > SPEED_ENABLE_THRESHOLD)
                y_corr = pidY.update(v_ref, v_meas, dt);
            else {
                y_corr = 0.0f;
                pidY.reset();
            }

            // X-axis control
            if (straight_mode) {
                const float heading_error = wrapAngle(heading_target - current_yaw);
                heading_hold_w_ref = pidHeading.update(0.0f, -heading_error, dt);
                x_corr = pidW.update(heading_hold_w_ref, w_meas_pid, dt);
            } else if (fabsf(v_ref) > SPEED_ENABLE_THRESHOLD ||
                       fabsf(w_ref) > TURN_ENABLE_THRESHOLD) {
                heading_hold_w_ref = 0.0f;
                x_corr = pidW.update(w_ref, w_meas_pid, dt);
            } else {
                heading_hold_w_ref = 0.0f;
                x_corr = 0.0f;
                pidW.reset();
                pidHeading.reset();
            }

            // Final command
            y_cmd = y_ff + y_corr;
            x_cmd = x_ff + x_corr;

            wheelchair.writeXYVoltages(y_cmd, x_cmd);
        }
        else {
            wheelchair.commandToVoltage(v_ref, w_ref, y_cmd, x_cmd);
            x_cmd += computeXTotalTrim(v_ref, w_ref);
            wheelchair.writeXYVoltages(y_cmd, x_cmd);
        }

        prev_snap = curr_snap;
        has_prev_snap = true;
    }

    // 8. Publish encoder and IMU data
    if (millis() - last_log_time > LOG_INTERVAL_MS) {
        last_log_time = millis();

        sensorPublisher.publishEncoder(
            encoderReader.getLeftCount(),
            encoderReader.getRightCount(),
            millis()
        );

        if (imu_ok) {
            sensorPublisher.publishImu(imu_sample);
        }
    }
}