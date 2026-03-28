#include <Arduino.h>
#include "WheelchairController.h"
#include "EncoderReader.h"
#include "CmdVelReceiver.h"

// GPIO PINS
const int SDA_PIN = 8;
const int SCL_PIN = 9;

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

// X Neutral: 2.69 -> 2.70 (Trying to find the actual neutral)
const float X_NEUTRAL = 2.6951f;

const float Y_MIN_V = 1.0f;
const float Y_MAX_V = 4.2f;
const float X_MIN_V = 1.0f;
const float X_MAX_V = 4.2f;

// Timing
const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long ENCODER_INTERVAL_MS = 50; //20Hz
const unsigned long LOG_INTERVAL_MS = 100;

// Objects
CmdVelReceiver cmdVelReceiver;

EncoderReader encoderReader(
    LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B, LEFT_SIGN, RIGHT_SIGN,
    LEFT_CPR, RIGHT_CPR, WHEEL_RADIUS, WHEEL_SEPARATION);

WheelchairController wheelchair(SDA_PIN, SCL_PIN, MCP4728_CHANNEL_A, MCP4728_CHANNEL_B, Y_NEUTRAL, X_NEUTRAL, Y_MIN_V, Y_MAX_V, X_MIN_V, X_MAX_V);


//ISR wrappers
void IRAM_ATTR leftISR(){
  encoderReader.handleLeftISR();
}

void IRAM_ATTR rightISR(){
  encoderReader.handleRightISR();
}

EncoderSnapshot prev_snap;
EncoderSnapshot curr_snap;
bool has_prev_snap = false;
unsigned long last_log_time = 0;

void setup(){
  Serial.begin(115200);
  delay(500);

  cmdVelReceiver.begin();
  encoderReader.begin();

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

  last_log_time = millis();

  Serial.println("time_ms,ref_v,ref_w,y_voltage,x_voltage,vL,vR,v,w");
}


void loop(){
  // 1. cmd_vel parse
  cmdVelReceiver.update();

  // 2. timeout safety
  if(millis() - cmdVelReceiver.getLastCmdMs() > CMD_TIMEOUT_MS){
    cmdVelReceiver.setZero();
  }

  // 3. cmd_vel to voltage
  float v_ref = cmdVelReceiver.getVRef();
  float w_ref = cmdVelReceiver.getWRef();

  float y_cmd = Y_NEUTRAL;
  float x_cmd = X_NEUTRAL;
  wheelchair.commandToVoltage(v_ref, w_ref, y_cmd, x_cmd);
  wheelchair.writeXYVoltages(y_cmd, x_cmd);

  // 4. encoder snapshot update
  if(encoderReader.readSnapshot(curr_snap, ENCODER_INTERVAL_MS)){
    if(has_prev_snap){
      encoderReader.updateVelocitiesFromSnapshot(prev_snap, curr_snap);
    }
    prev_snap = curr_snap;
    has_prev_snap = true;
  }

  // 5. logging
  if(millis() - last_log_time > LOG_INTERVAL_MS){
    last_log_time = millis();
    
    Serial.print(millis());
    Serial.print(",");

    Serial.print(v_ref, 3);
    Serial.print(",");
    Serial.print(w_ref, 3);
    Serial.print(",");

    Serial.print(wheelchair.getYVoltage(), 3);
    Serial.print(",");
    Serial.print(wheelchair.getXVoltage(), 3);
    Serial.print(",");

    Serial.print(encoderReader.getVL(), 4);
    Serial.print(",");
    Serial.print(encoderReader.getVR(), 4);
    Serial.print(",");

    Serial.print(encoderReader.getVBody(), 3);
    Serial.print(",");
    Serial.println(encoderReader.getWBody(), 3);

  }
}