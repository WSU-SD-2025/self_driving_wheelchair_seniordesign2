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
const int RIGHT_SIGN = 1;

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
// =========================Took
const float MAX_LINEAR_CMD = 1.0f;   // m/s
const float MAX_ANGULAR_CMD = 1.0f;  // rad/s

const float Y_SPAN_V = 0.90f;
const float X_SPAN_V = 0.70f;

const float Y_DEADBAND_V = 0.10f;
const float X_DEADBAND_V = 0.10f;


// =========================
// PID Settings
// =========================
const float PIDY_KP = 0.01f;
const float PIDY_KI = 0.08f;
const float PIDY_KD = 0.0f;

const float PIDW_KP = 0.025f;
const float PIDW_KI = 0.005f;
const float PIDW_KD = 0.0f;

// Output correction limits (V)
const float Y_PID_MIN = -0.20f;
const float Y_PID_MAX = 0.20f;

const float X_PID_MIN = -0.18f;
const float X_PID_MAX = 0.18f;

// Enable thresholds
const float SPEED_ENABLE_THRESHOLD = 0.02f;
const float TURN_ENABLE_THRESHOLD = 0.02f;


// =========================
// Sign settings
// =========================
const float IMU_WZ_SIGN  = -1.0f;
//Debug flag
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


// =========================
// Final output voltages
// =========================
float y_ff = Y_NEUTRAL;
float x_ff = X_NEUTRAL;

float y_corr = 0.0f;
float x_corr = 0.0f;

float y_cmd = Y_NEUTRAL;
float x_cmd = X_NEUTRAL;

void resetControllers(){
    pidY.reset();
    pidW.reset();
    y_corr = 0.0f;
    x_corr = 0.0f;
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

    // Initialize DAC
    if (!wheelchair.begin()) {
        Serial.println("MCP4728 not found");
        while (1) delay(10);
    }

    // Configure linear mapping parameters
    wheelchair.setCommands(MAX_LINEAR_CMD, MAX_ANGULAR_CMD);
    wheelchair.setVoltageSpans(Y_SPAN_V, X_SPAN_V);
    wheelchair.setDeadbands(Y_DEADBAND_V, X_DEADBAND_V);
    // Set neutral output at startup
    wheelchair.setNeutral();

    pidY.begin(PIDY_KP, PIDY_KI, PIDY_KD, Y_PID_MIN, Y_PID_MAX);
    pidW.begin(PIDW_KP, PIDW_KI, PIDW_KD, X_PID_MIN, X_PID_MAX);

    last_log_time = millis();
    last_imu_time = millis();

    Serial.println("SENSORS_READY");
}

// ===================== MAIN LOOP =====================

void loop() {

    // 1. Read incoming cmd_vel packets from serial
    handleSerialInput();

    // 2. Safety timeout: stop if no command received
    if (millis() - cmdVelReceiver.getLastCmdMs() > CMD_TIMEOUT_MS) {
        cmdVelReceiver.setZero();
        resetControllers();
    }

    // 3. Read command references
    v_ref = cmdVelReceiver.getVRef();

    // IMPORTANT:
    // ROS2: +angular.z = left turn (CCW)
    // Wheelchair X voltage: + = right turn
    // → invert sign here to match behavior
    w_ref = -cmdVelReceiver.getWRef();

    // 4. Update IMU (for publishing only)
    um7_update();
    if (um7_get_sample(imu_sample)) {
        imu_ok = imu_sample.valid;
        if (imu_ok) {
            // Apply sign correction to match ROS2 convention
            imu_sample.wz = IMU_WZ_SIGN * imu_sample.wz;
            last_imu_time = millis();
        }
    }

    // IMU timeout handling
    if (millis() - last_imu_time > IMU_TIMEOUT_MS) {
        imu_ok = false;
    }

    // 5. Update encoder-based velocity and run control
    if (encoderReader.readSnapshot(curr_snap, ENCODER_INTERVAL_MS)) {
        if (has_prev_snap) {
            encoderReader.updateVelocitiesFromSnapshot(prev_snap, curr_snap);

            float dt = (curr_snap.time_ms - prev_snap.time_ms) / 1000.0f;
            v_meas = encoderReader.getVBody();
            w_meas_encoder = encoderReader.getWBody();
            w_meas_pid = -w_meas_encoder;

            // Feedforward frrom linear mapping
            wheelchair.commandToVoltage(v_ref, w_ref, y_ff, x_ff);

            // Y-axis PID (linear velocity)
            if(fabsf(v_ref) > SPEED_ENABLE_THRESHOLD)
                y_corr = pidY.update(v_ref, v_meas, dt);
            else{
                y_corr = 0.0f;
                pidY.reset();
            }
            // X-axis PID (angular velocity)
            if(fabsf(w_ref) > TURN_ENABLE_THRESHOLD)
                x_corr = pidW.update(w_ref, w_meas_pid, dt);
            else{
                x_corr = 0.0f;
                pidW.reset();
            }

            //Final command = feedforward + correction
            y_cmd = y_ff + y_corr;
            x_cmd = x_ff + x_corr;

            wheelchair.writeXYVoltages(y_cmd, x_cmd);
        }
        else{
            //First snapshot: just apply feedforward only
            wheelchair.commandToVoltage(v_ref, w_ref, y_cmd, x_cmd);
            wheelchair.writeXYVoltages(y_cmd, x_cmd);
        }

        prev_snap = curr_snap;
        has_prev_snap = true;
    }
    

    // 6. Publish encoder and IMU data
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