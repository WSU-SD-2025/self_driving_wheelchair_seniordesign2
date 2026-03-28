#include <Arduino.h>
#include "WheelchairController.h"
#include "EncoderReader.h"
#include "CmdVelReceiver.h"
#include "PidController.h"
#include "um7_parser.h"
#include <math.h>
#include "LookupTable.h"

// GPIO PINS

// Wheelchair I2C (MCP4728)
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// Encoders
const int LEFT_ENCODER_A = 4;
const int LEFT_ENCODER_B = 5;
const int RIGHT_ENCODER_A = 16;
const int RIGHT_ENCODER_B = 17;

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

// X Neutral: Should be 2.6951 later
const float X_NEUTRAL = 2.690f;

const float Y_MIN_V = 1.0f;
const float Y_MAX_V = 4.2f;
const float X_MIN_V = 1.0f;
const float X_MAX_V = 4.2f;

// Timing
const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long ENCODER_INTERVAL_MS = 20; //50Hz
const unsigned long CONTROL_INTERVAL_MS = 20; //50Hz
const unsigned long LOG_INTERVAL_MS = 100;
const unsigned long IMU_TIMEOUT_MS = 200;



//
// Objects
//
CmdVelReceiver cmdVelReceiver;

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

// IMU state
ImuSample imu_sample;
bool imu_ok = false;
float yaw_meas = 0.0f; 

// Reference states
float v_ref = 0.0f;
float w_ref = 0.0f;

// Measured states
float v_meas = 0.0f;
float w_meas = 0.0f;

// Output voltages
float y_cmd = Y_NEUTRAL;
float x_cmd = X_NEUTRAL;

// Quaternion to Yaw
float quaternionToYaw(const ImuSample& imu){
    float siny_cosp = 2.0f * (imu.qw * imu.qz + imu.qx * imu.qy);
    float cosy_cosp = 1.0f - 2.0f * (imu.qy * imu.qy + imu.qz * imu.qz);
    return atan2f(siny_cosp, cosy_cosp);
}




void setup(){
  Serial.begin(115200);
  delay(500);

  cmdVelReceiver.begin();
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

  wheelchair.setCommands(1.5f, 1.5f);
  wheelchair.setVoltageSpans(0.80f, 0.80f);
  wheelchair.setDeadbands(0.12f, 0.12f);

  wheelchair.setNeutral();

  last_log_time = millis();
  last_control_time = millis();
  last_imu_time = millis();

  Serial.println("STATE,time_ms,ref_v,ref_w,v_meas,w_meas,yaw,y_voltage,x_voltage,vL,vR,qx,qy,qz,qw,wx,wy,wz,ax,ay,az");
}


void loop(){
    // 1. Receive lastest cmd_vel
    cmdVelReceiver.update();

    // 2. timeout safety
    if(millis() - cmdVelReceiver.getLastCmdMs() > CMD_TIMEOUT_MS){
      cmdVelReceiver.setZero();
    }

    v_ref = cmdVelReceiver.getVRef();
    w_ref = cmdVelReceiver.getWRef();

    // 3. Update IMU continuously
    um7_update();
    if(um7_get_sample(imu_sample)){
      imu_ok = imu_sample.valid;
      if(imu_ok){
        yaw_meas = quaternionToYaw(imu_sample);
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

    // 5. Latest measured states
    v_meas = encoderReader.getVBody();
    w_meas = encoderReader.getWBody();

    // 6. Control loop
    unsigned long now = millis();
    if(now - last_control_time >= CONTROL_INTERVAL_MS){
      last_control_time = now;

      // Direct lookup from references
      y_cmd = lookupYVoltage(v_ref);
      x_cmd = lookupXVoltage(w_ref);
      wheelchair.writeXYVoltages(y_cmd, x_cmd);
    }


    // 7. logging / Bridge output
    if(millis() - last_log_time > LOG_INTERVAL_MS){
      last_log_time = millis();
    
      // ODOM packet
      Serial.print("ODOM,");
      Serial.print(millis());
      Serial.print(",");
      Serial.print(encoderReader.getVL());
      Serial.print(",");
      Serial.print(encoderReader.getVR());
      Serial.print(",");
      Serial.print(encoderReader.getVBody());
      Serial.print(",");
      Serial.println(encoderReader.getWBody());
    
      // IMU packet
      if(imu_ok){
        Serial.print("IMU,");
        Serial.print(millis());
        Serial.print(",");
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
      Serial.print("DEBUG,");
      Serial.print(millis());
      Serial.print(",");
      Serial.print(v_ref); Serial.print(",");
      Serial.print(w_ref); Serial.print(",");
      Serial.print(v_meas); Serial.print(",");
      Serial.print(w_meas); Serial.print(",");
      Serial.print(yaw_meas); Serial.print(",");
      Serial.print(y_cmd, 4); Serial.print(",");
      Serial.println(x_cmd, 4);

  }
}