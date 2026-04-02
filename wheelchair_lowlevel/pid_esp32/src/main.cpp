#include <Arduino.h>
#include "WheelchairController.h"
#include "EncoderReader.h"
#include "CmdVelReceiver.h"
#include "PidController.h"
#include "um7_parser.h"
#include <math.h>
#include "LookupTable.h"
#include "DebugConsole.h"

// GPIO PINS

// Wheelchair I2C (MCP4728)
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// Encoders
const int LEFT_ENCODER_A = 1;
const int LEFT_ENCODER_B = 2;
const int RIGHT_ENCODER_A = 47;
const int RIGHT_ENCODER_B = 48;

// Encoder Sign
const int LEFT_SIGN = 1;
const int RIGHT_SIGN = 1;

// Encoder & Wheelchair parameters
const float LEFT_CPR = 715.0f;
const float RIGHT_CPR = 1200.0f;
const float WHEEL_RADIUS = 0.171f;
const float WHEEL_SEPARATION = 0.575f;

// Voltage settings
const float Y_NEUTRAL = 2.689f;
const float X_NEUTRAL = 2.6951f;

const float Y_MIN_V = 1.0f;
const float Y_MAX_V = 4.2f;
const float X_MIN_V = 1.0f;
const float X_MAX_V = 4.2f;

// Timing
const unsigned long CMD_TIMEOUT_MS = 1000;
const unsigned long ENCODER_INTERVAL_MS = 20; //50Hz
const unsigned long CONTROL_INTERVAL_MS = 20; //50Hz
const unsigned long LOG_INTERVAL_MS = 100;
const unsigned long IMU_TIMEOUT_MS = 200;


// Control Settings

// Lookup Table Gain
const float Y_FF_GAIN = 1.0f;
const float X_FF_GAIN = 0.50f;

// If IMU sign is opposite, change one of these to -1.0f
// If the wheelchair's rotation is reverse, change the sign
const float IMU_WZ_SIGN = 1.0f;
// If the heading hold is changing in reverse way with yaw, change the sign
const float IMU_YAW_SIGN = 1.0f;

const float TURN_ENABLE_THRESHOLD = 0.03f;
const float SPEED_ENABLE_THRESHOLD = 0.02f;

// Heading hold settings
const float HEADING_HOLD_ENABLE_W = 0.02f;
const float HEADING_HOLD_ENABLED_SPEED = 0.05f;
const float HEADING_HOLD_MAX_W = 0.20f;

// Safety / sanity
const float IMU_WZ_VALID_LIMIT = 3.5f;
const float HEADING_ASSIST_BLEND = 0.5f;
const float X_ASSIST_LIMIT = 0.10f;


// Debug only mode enum
enum ControlMode: uint8_t {
  MODE_STOP = 0,
  MODE_TRACKING_ENCODER_W = 1,
  MODE_TRACKING_IMU_W = 2,
  MODE_TRACKING_IMU_W_WITH_HEADING_HOLD = 3
};



//
// Objects
//
CmdVelReceiver cmdVelReceiver;
DebugConsole debugConsole;
DebugFlags debugFlags;
DebugTuning debugTuning;

EncoderReader encoderReader(
    LEFT_ENCODER_A, LEFT_ENCODER_B, 
    RIGHT_ENCODER_A, RIGHT_ENCODER_B, 
    LEFT_SIGN, RIGHT_SIGN,
    LEFT_CPR, RIGHT_CPR, 
    WHEEL_RADIUS, WHEEL_SEPARATION);

WheelchairController wheelchair(
    SDA_PIN, SCL_PIN, 
    MCP4728_CHANNEL_A, MCP4728_CHANNEL_B, 
    Y_NEUTRAL, X_NEUTRAL, 
    Y_MIN_V, Y_MAX_V, 
    X_MIN_V, X_MAX_V);

// PID Controllers
PidController pidY;   // Linear velocity correction
PidController pidW;   // Angular velocity correction
PidController pidHeading; // Heading hold outer loop
PidController pidWAssist; // Heading hold inner assist loop

//ISR wrappers
void IRAM_ATTR leftISR(){
  encoderReader.handleLeftISR();
}

void IRAM_ATTR rightISR(){
  encoderReader.handleRightISR();
}

// Encoder snapshots
EncoderSnapshot prev_snap;
EncoderSnapshot curr_snap;
bool has_prev_snap = false;

unsigned long last_log_time = 0;
unsigned long last_control_time = 0;
unsigned long last_imu_time = 0;

// Serial input line buffer
String serial_line = "";

// IMU state
ImuSample imu_sample;
bool imu_ok = false;
float yaw_meas = 0.0f; 

// Heading hold state
float heading_ref = 0.0f;
bool heading_hold_active = false;
float heading_error = 0.0f;
float heading_hold_w_ref = 0.0f;

// Reference states
float v_ref = 0.0f;
float w_ref = 0.0f;

// Measured states
float v_meas = 0.0f;
float w_meas_encoder = 0.0f;
float imu_wz_for_control = 0.0f;
float w_meas_for_control = 0.0f;
bool using_imu_for_w = false;

// Feedforward voltages from LUT
float y_ff = Y_NEUTRAL;
float x_ff = X_NEUTRAL;

// PID correction voltages
float y_corr_v = 0.0f;
float x_corr_v = 0.0f;
float x_assist_v = 0.0f;

// Final output voltages
float y_cmd = Y_NEUTRAL;
float x_cmd = X_NEUTRAL;

// Debug-only
ControlMode control_mode = MODE_STOP;


// Quaternion to Yaw
float quaternionToYaw(const ImuSample& imu){
    float siny_cosp = 2.0f * (imu.qw * imu.qz + imu.qx * imu.qy);
    float cosy_cosp = 1.0f - 2.0f * (imu.qy * imu.qy + imu.qz * imu.qz);
    return atan2f(siny_cosp, cosy_cosp);
}

float wrapAngle(float a){
  while(a > PI) a-= 2.0f * PI;
  while(a < -PI) a+= 2.0f * PI;
  return a;
}

float clampf_local(float v, float lo, float hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

void resetControllerAndHold(){
  pidY.reset();
  pidW.reset();
  pidHeading.reset();
  pidWAssist.reset();

  heading_hold_active = false;
  heading_error = 0.0f;
  heading_hold_w_ref = 0.0f;
  x_assist_v = 0.0f;
}


void handleSerialInput(){
  while(Serial.available() > 0){
    char c = (char)Serial.read();

    if(c == '\r')   continue;

    if(c == '\n'){
      if(serial_line.length() > 0){
        if(serial_line.startsWith("<")){
          cmdVelReceiver.processLine(serial_line);
        }
        else if(serial_line.startsWith("!")){
          debugConsole.processLine(
            serial_line,
            debugFlags,
            debugTuning,
            pidY, pidW, pidHeading, pidWAssist,
            cmdVelReceiver,
            wheelchair,
            heading_hold_active,
            HEADING_HOLD_MAX_W
          );
        }
      }
      serial_line = "";
    }
    else{
      serial_line += c;
      if(serial_line.length() > 120){
        serial_line = "";
      }
    }
  }
}



void setup(){
    Serial.begin(115200);
    delay(500);

    cmdVelReceiver.begin();
    debugConsole.begin();
    encoderReader.begin();
    um7_begin();

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightISR, CHANGE);

    if(!wheelchair.begin()){
      Serial.println("MCP4728 not found");
      while(1){
        delay(10);
      }
    }

    wheelchair.setNeutral();

    // PID outputs are voltage corrections, not normalized values
    // Conservative starting gains

    // Begin(Kp, Ki, Kd, out_min, out_max)
    // PidY: Linear velocity correction, max correction of +/- 0.25V
    // PidW: Angular velocity correction, max correction of +/- 0.20V
    pidY.begin(0.10f, 0.01f, 0.0f, -0.20f, 0.20f);
    pidW.begin(0.12f, 0.01f, 0.0f, -0.18f, 0.18f);
    pidHeading.begin(1.20f, 0.00f, 0.00f, -HEADING_HOLD_MAX_W, HEADING_HOLD_MAX_W);
    pidWAssist.begin(0.10f, 0.00f, 0.00f, -X_ASSIST_LIMIT, X_ASSIST_LIMIT);

    last_log_time = millis();
    last_control_time = millis();
    last_imu_time = millis();

    Serial.println("DEBUG,time_ms,mode,ref_v,ref_w,"
                    "v_meas,w_meas_enc,w_ctrl,using_imu,"
                    "imu_ok,imu_wz,yaw,"
                    "heading_ref,heading_error,heading_hold_w_ref,"
                    "y_ff,x_ff,y_corr_v,x_corr_v,x_assist_v,"
                    "trimy,trimx,y_cmd,x_cmd,vL,vR");
    Serial.println("SENSORS_READY");
}


void loop(){
    // 1. Read Serial input line by line
    handleSerialInput();

    // 2. timeout safety
    if(millis() - cmdVelReceiver.getLastCmdMs() > CMD_TIMEOUT_MS){
      cmdVelReceiver.setZero();
      resetControllerAndHold();
    }

    v_ref = cmdVelReceiver.getVRef();
    w_ref = cmdVelReceiver.getWRef();

    // 3. Update IMU continuously
    um7_update();
    if(um7_get_sample(imu_sample)){
      imu_ok = imu_sample.valid;
      if(imu_ok){
        yaw_meas = IMU_YAW_SIGN * quaternionToYaw(imu_sample);
        last_imu_time = millis();
      }
    }

    // IMU timeout
    if(millis() - last_imu_time > IMU_TIMEOUT_MS){
      imu_ok = false;
    } 

    // 4. Encoder update
    if(encoderReader.readSnapshot(curr_snap, ENCODER_INTERVAL_MS)){
      if(has_prev_snap){
        encoderReader.updateVelocitiesFromSnapshot(prev_snap, curr_snap);
      }
      prev_snap = curr_snap;
      has_prev_snap = true;
    }

    // 5. Measured states
    v_meas = encoderReader.getVBody();
    w_meas_encoder = encoderReader.getWBody();

    imu_wz_for_control = IMU_WZ_SIGN * imu_sample.wz;
    bool imu_wz_reasonable = fabsf(imu_wz_for_control) < IMU_WZ_VALID_LIMIT;
    using_imu_for_w = (imu_ok && imu_wz_reasonable);

    if(using_imu_for_w){
        w_meas_for_control = imu_wz_for_control;
    }
    else{
        w_meas_for_control = w_meas_encoder;
    }
    

    // 6. Control loop: LUT + PID correction
    unsigned long now = millis();
    if(now - last_control_time >= CONTROL_INTERVAL_MS){
        float dt = (now - last_control_time) / 1000.0f;
        last_control_time = now;

        bool stop_mode = 
            (fabsf(v_ref) <= SPEED_ENABLE_THRESHOLD) && 
            (fabsf(w_ref) < TURN_ENABLE_THRESHOLD);

        bool nearly_straight = 
            (fabsf(w_ref) < HEADING_HOLD_ENABLE_W) && 
            (fabsf(v_ref) >= HEADING_HOLD_ENABLED_SPEED);

        bool heading_hold_allowed = using_imu_for_w && nearly_straight;

        // Feedforward from LookUp Table
        float y_ff_raw = lookupYVoltage(v_ref);
        float x_ff_raw = lookupXVoltage(w_ref);

        y_ff = Y_NEUTRAL + Y_FF_GAIN * (y_ff_raw - Y_NEUTRAL);
        x_ff = X_NEUTRAL + X_FF_GAIN * (x_ff_raw - X_NEUTRAL);

        if(stop_mode){
            control_mode = MODE_STOP;
            y_corr_v = 0.0f;
            x_corr_v = 0.0f;
            x_assist_v = 0.0f;
            resetControllerAndHold();

            y_cmd = Y_NEUTRAL + debugTuning.y_neutral_trim;
            x_cmd = X_NEUTRAL + debugTuning.x_neutral_trim;
        }

        else{
          // Always do linear-speed tracking
          if(fabsf(v_ref) > SPEED_ENABLE_THRESHOLD)
              y_corr_v = pidY.update(v_ref, v_meas, dt);
          else{
              y_corr_v = 0.0f;
              pidY.reset();
          }

          // Always do angular-rate tracking
          x_corr_v = -pidW.update(w_ref, w_meas_for_control, dt);
          x_assist_v = 0.0f;

          if(heading_hold_allowed){
              if(!heading_hold_active){
                  heading_ref = yaw_meas;
                  heading_hold_active = true;
                  pidHeading.reset();
                  pidWAssist.reset();
              }


              heading_error = wrapAngle(heading_ref - yaw_meas);

              // Outer loop: heading error -> desired yaw rate assist
              heading_hold_w_ref = pidHeading.update(0.0f, -heading_error, dt);

              // Inner loop assist: desired yaw rate -> x voltage assist
              float assist_cmd = heading_hold_w_ref * HEADING_ASSIST_BLEND; 
              x_assist_v = -pidWAssist.update(assist_cmd, w_meas_for_control, dt);
              x_assist_v = clampf_local(x_assist_v, -X_ASSIST_LIMIT, X_ASSIST_LIMIT);
              
              control_mode = MODE_TRACKING_IMU_W_WITH_HEADING_HOLD;
          }

          else{
              heading_hold_active = false;
              heading_error = 0.0f;
              heading_hold_w_ref = 0.0f;
              x_assist_v = 0.0f;
              pidHeading.reset();
              pidWAssist.reset();

              if(using_imu_for_w)   control_mode = MODE_TRACKING_IMU_W;
              else                  control_mode = MODE_TRACKING_ENCODER_W;
          }

          // Final command = feedforward + PID + heading assist + trim
          y_cmd = y_ff + y_corr_v + debugTuning.y_neutral_trim;
          x_cmd = x_ff + x_corr_v + x_assist_v + debugTuning.x_neutral_trim;
        }
        // Output
        wheelchair.writeXYVoltages(y_cmd, x_cmd);
    }


    // 7. logging / Bridge output
    if(millis() - last_log_time > LOG_INTERVAL_MS){
        last_log_time = millis();

        if(debugFlags.print_encoder){
            Serial.print("ENCODER,");
            Serial.print(encoderReader.getLeftCount());
            Serial.print(",");
            Serial.print(encoderReader.getRightCount());
            Serial.print(",");
            Serial.println(millis());
        }

        // ROS2 IMU Bridge
        if(debugFlags.print_imu && imu_ok){
            Serial.print("IMU,");
            Serial.print(imu_sample.qx, 6); Serial.print(",");
            Serial.print(imu_sample.qy, 6); Serial.print(",");
            Serial.print(imu_sample.qz, 6); Serial.print(",");
            Serial.print(imu_sample.qw, 6); Serial.print(",");
            Serial.print(imu_sample.wx, 6); Serial.print(",");
            Serial.print(imu_sample.wy, 6); Serial.print(",");
            Serial.print(imu_sample.wz, 6); Serial.print(",");
            Serial.print(imu_sample.ax, 6); Serial.print(",");
            Serial.print(imu_sample.ay, 6); Serial.print(",");
            Serial.println(imu_sample.az, 6);
        }

        // Debug packet
        if(debugFlags.print_debug){
            Serial.printf("DEBUG,"); 
            Serial.print(millis()); Serial.print(",");
            Serial.print((int)control_mode); Serial.print(",");
            Serial.print(v_ref, 4); Serial.print(",");
            Serial.print(w_ref, 4); Serial.print(",");
            Serial.print(v_meas, 4); Serial.print(",");
            Serial.print(w_meas_encoder, 4); Serial.print(",");
            Serial.print(w_meas_for_control, 4); Serial.print(",");
            Serial.print(using_imu_for_w ? 1 : 0); Serial.print(",");
            Serial.print(imu_ok ? 1 : 0); Serial.print(",");
            Serial.print(imu_wz_for_control, 4); Serial.print(",");
            Serial.print(yaw_meas, 4); Serial.print(",");
            Serial.print(heading_ref, 4); Serial.print(",");
            Serial.print(heading_error, 4); Serial.print(",");
            Serial.print(heading_hold_w_ref, 4); Serial.print(",");
            Serial.print(y_ff, 4); Serial.print(",");
            Serial.print(x_ff, 4); Serial.print(",");
            Serial.print(y_corr_v, 4); Serial.print(",");
            Serial.print(x_corr_v, 4); Serial.print(",");
            Serial.print(x_assist_v, 4); Serial.print(",");
            Serial.print(debugTuning.y_neutral_trim, 4); Serial.print(",");
            Serial.print(debugTuning.x_neutral_trim, 4); Serial.print(",");
            Serial.print(y_cmd, 4); Serial.print(",");
            Serial.print(x_cmd, 4); Serial.print(",");
            Serial.print(encoderReader.getVL(), 4); Serial.print(",");
            Serial.println(encoderReader.getVR(), 4);
        }
    }
}