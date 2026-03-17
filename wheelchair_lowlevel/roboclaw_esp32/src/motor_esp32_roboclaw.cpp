#include<Arduino.h>
#include <math.h>
#include "Basicmicro.h"

// Serial Settings
#define PC_BAUD 115200
#define ROBOCLAW_BAUD 38400

// Roboclaw UART pins
#define ROBOCLAW_RX_PIN 16      // RoboClaw S2 -> ESP32 RX
#define ROBOCLAW_TX_PIN 17      // RoboClaw S1 -> ESP32 TX

const uint8_t ROBOCLAW_ADDRESS = 0x80;
Basicmicro roboclaw(&Serial2, 10000)  ; // 10ms timeout

// Parameters
const float MAX_WHEEL_LINEAR_SPEED = 1.5f; // m/s
const float MAX_ALLOWED_WHEEL_SPEED = 1.5f;

const int MOTOR_CMD_DEADBAND = 3;

// Ramping
const float RAMP_STEP = 4.0f;

// Timeout
const unsigned long CMD_TIMEOUT_MS = 250; // 250ms

// Main loop
const unsigned long LOOP_PERIOD_MS = 10;

// Direction correction
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = 1;

// State
float target_left_speed = 0.0f;
float target_right_speed = 0.0f;

float current_left_cmd = 0.0f;
float current_right_cmd = 0.0f;

unsigned long last_cmd_time = 0;
unsigned long last_loop_time = 0;




// clamp
float clampFloat(float x, float min_val, float max_val){
    if(x < min_val) return min_val;
    if(x > max_val) return max_val;
    return x;
}


float ramp(float current_value, float target_value, float step){
    if(target_value > current_value + step) return current_value + step;
    if(target_value < current_value - step) return current_value - step;
    return target_value;
}


int wheelSpeedToCommand(float wheel_speed_mps, int motor_sign){
    wheel_speed_mps = clampFloat(wheel_speed_mps, -MAX_ALLOWED_WHEEL_SPEED, MAX_ALLOWED_WHEEL_SPEED);

    float normalized = wheel_speed_mps / MAX_WHEEL_LINEAR_SPEED;
    normalized = clampFloat(normalized, -1.0f, 1.0f);

    int cmd = (int)(normalized * 127.0f);
    cmd *= motor_sign;

    if(abs(cmd) < MOTOR_CMD_DEADBAND)   cmd = 0;

    cmd = constrain(cmd, -127, 127);
    return cmd;
}


void sendM1(int cmd){
    cmd = constrain(cmd, -127, 127);
    if(cmd >= 0)    roboclaw.ForwardM1(ROBOCLAW_ADDRESS, cmd);
    else            roboclaw.BackwardM1(ROBOCLAW_ADDRESS, -cmd);
}


void sendM2(int cmd){
    cmd = constrain(cmd, -127, 127);
    if(cmd >= 0)    roboclaw.ForwardM2(ROBOCLAW_ADDRESS, cmd);
    else            roboclaw.BackwardM2(ROBOCLAW_ADDRESS, -cmd);
}


void stopMotors(){
    sendM1(0);
    sendM2(0);
    current_left_cmd = 0.0f;
    current_right_cmd = 0.0f;
}


// Parser
bool parseWheelCommand(const String& line, float& left_speed, float& right_speed){
    if(!line.startsWith("WHEEL,")) return false;

    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 + 1);

    if(comma1 < 0 || comma2 < 0) return false;

    String left_str = line.substring(comma1 + 1, comma2);
    String right_str = line.substring(comma2+1);

    left_speed = left_str.toFloat();
    right_speed = right_str.toFloat();
    return true;
}


void readCommandFromPC(){
    if(!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if(line.length() == 0) return;

    float new_left, new_right;
    if(parseWheelCommand(line, new_left, new_right)){
        target_left_speed = new_left;
        target_right_speed = new_right;
        last_cmd_time = millis();

        Serial.print("ACK,");
        Serial.print(target_left_speed, 4);
        Serial.print(",");
        Serial.println(target_right_speed, 4);
    }
}


// Drive
void driveFromWheelTargets(){
    int left_target_cmd = wheelSpeedToCommand(target_left_speed, LEFT_MOTOR_SIGN);
    int right_target_cmd = wheelSpeedToCommand(target_right_speed, RIGHT_MOTOR_SIGN);

    current_left_cmd = ramp(current_left_cmd, (float)left_target_cmd, RAMP_STEP);
    current_right_cmd = ramp(current_right_cmd, (float)right_target_cmd, RAMP_STEP);

    sendM1((int)current_left_cmd);
    sendM2((int)current_right_cmd);

    Serial.print("DBG,");
    Serial.print(target_left_speed, 4);
    Serial.print(",");
    Serial.print(target_right_speed, 4);
    Serial.print(",");
    Serial.print(current_left_cmd, 4);
    Serial.print(",");
    Serial.println(current_right_cmd, 4);
}


void setup(){
    Serial.begin(PC_BAUD);
    Serial.setTimeout(5);
    Serial2.begin(ROBOCLAW_BAUD, SERIAL_8N1, ROBOCLAW_RX_PIN, ROBOCLAW_TX_PIN);
    roboclaw.begin(ROBOCLAW_BAUD);
    delay(1000);
    stopMotors();

    last_cmd_time = millis();
    last_loop_time = millis();

    Serial.println("RoboClaw bridge Started");
}

void loop(){
    readCommandFromPC();

    unsigned long now = millis();
    if(now - last_loop_time < LOOP_PERIOD_MS) return;
    last_loop_time = now;

    if(now - last_cmd_time > CMD_TIMEOUT_MS){
        stopMotors();
        return;
    }

    driveFromWheelTargets();
}
