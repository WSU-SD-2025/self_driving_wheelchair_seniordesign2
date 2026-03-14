#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// I2C pins
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// DAC Channel settings
const MCP4728_channel_t X_CH = MCP4728_CHANNEL_B;
const MCP4728_channel_t Y_CH = MCP4728_CHANNEL_A;

// Voltage settings
// 12-bit DAC: 0 ~ 4095
// DAC VDD = 5.0V
const float DAC_VREF = 5.0f;


// Measured Wheelchair Voltages
const float X_NEUTRAL_V = 2.50f;
const float Y_NEUTRAL_V = 2.50f;

const float X_LEFT_MAX_V = 0.81f;
const float X_RIGHT_MAX_V = 4.18f;

const float Y_FORWARD_MAX_V = 4.23f;
const float Y_BACKWARD_MAX_V = 1.17f;

const float X_STRAIGHT_LOCK_V = 0.0f;
const float Y_TURN_LOCK_V = 0.0f;


// ROS2 Max Speed
const float MAX_LINEAR_CMD = 1.5f;
const float MAX_ANGULAR_CMD = 3.0f;


// Safety variables
const float DEADZONE_LINEAR = 0.01f;
const float DEADZONE_ANGULAR = 0.01f;

const unsigned long CMD_TIMEOUT_MS = 500; // 500 ms timeout for commands
const unsigned long LOOP_DT_MS = 20; // 20 ms loop time (50 Hz)

const float MAX_VOLT_STEP = 0.03f; // Max voltage change per loop to prevent sudden jumps


// State variables
float latest_linear_cmd = 0.0f;
float latest_angular_cmd = 0.0f;

float target_x_v = X_NEUTRAL_V;
float target_y_v = Y_NEUTRAL_V;

float current_x_v = X_NEUTRAL_V;
float current_y_v = Y_NEUTRAL_V;

unsigned long last_packet_time = 0;

String rx_buffer = ""; // Buffer for incoming serial data



// Helpers
float clampf(float x, float lo, float hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}

float applyDeadzone(float x, float deadzone){
  return (fabs(x) < deadzone) ? 0.0f : x;
}


uint16_t voltageToBit(float voltage){
  // Convert voltage to DAC code (0-4095)
  voltage = clampf(voltage, 0.0f, DAC_VREF);
  return (uint16_t)((voltage / DAC_VREF) * 4095.0f);
}

float rampV(float current, float target, float max_step){
  float diff = target - current;
  if(diff > max_step) return current + max_step;
  if(diff < -max_step) return current - max_step;
  return target;
}

void writeVoltages(float x_v, float y_v){
  uint16_t x_code = voltageToBit(x_v);
  uint16_t y_code = voltageToBit(y_v);

  mcp.setChannelValue(X_CH, x_code);
  mcp.setChannelValue(Y_CH, y_code);
}


float mapForwardBackward(float v_norm){
  // Map normalized linear command (-1 to 1) to voltage
  v_norm = clampf(v_norm, -1.0f, 1.0f);

  if(v_norm >= 0.0f)
    return Y_NEUTRAL_V + v_norm * (Y_FORWARD_MAX_V - Y_NEUTRAL_V);
  else
    return Y_NEUTRAL_V + v_norm * (Y_NEUTRAL_V - Y_BACKWARD_MAX_V);
}


float mapTurn(float w_norm){
  // Map normalized angular command (-1 to 1) to voltage
  w_norm = clampf(w_norm, -1.0f, 1.0f);

  if(w_norm >= 0.0f)
    return X_NEUTRAL_V + w_norm * (X_RIGHT_MAX_V - X_NEUTRAL_V);
  else
    return X_NEUTRAL_V + w_norm * (X_NEUTRAL_V - X_LEFT_MAX_V);
}


void computeTargetVoltages(float linear_cmd, float angular_cmd, float& x_v, float& y_v){
  float v = linear_cmd / MAX_LINEAR_CMD; // Normalize linear command
  float w = angular_cmd / MAX_ANGULAR_CMD; // Normalize angular command

  v = clampf(v, -1.0f, 1.0f);
  w = clampf(w, -1.0f, 1.0f);

  v = applyDeadzone(v, DEADZONE_LINEAR);
  w = applyDeadzone(w, DEADZONE_ANGULAR);

  // neutral
  if(v == 0.0f && w == 0.0f){
    x_v = X_NEUTRAL_V;
    y_v = Y_NEUTRAL_V;
    return;
  }

  // pure straight
  if(fabs(v) > 0.0f && fabs(w) == 0.0f){
    x_v = X_STRAIGHT_LOCK_V;
    y_v = mapForwardBackward(v);
    return;
  }

  // pure turn
  if(fabs(w) > 0.0f && fabs(v) == 0.0f){
    x_v = mapTurn(w);
    y_v = Y_TURN_LOCK_V;
    return;
  }

  // mixed motion
  if(fabs(v) >= fabs(w)){
    x_v = X_STRAIGHT_LOCK_V;
    y_v = mapForwardBackward(v);
  }
  else{
    x_v = mapTurn(w);
    y_v = Y_TURN_LOCK_V;
  }
}


// Packet Parser: <v,w>
bool packetParser(const String& packet, float& linear_out, float& angular_out){
  int start = packet.indexOf('<');
  int comma = packet.indexOf(',');
  int end = packet.indexOf('>');

  if(start == -1 || comma == -1 || end == -1) return false;
  if(!(start < comma && comma < end)) return false;

  String vs = packet.substring(start + 1, comma);
  String ws = packet.substring(comma + 1, end);

  linear_out = vs.toFloat();
  angular_out = ws.toFloat();
  return true;
}

void handleSerialInput(){
  while(Serial.available() > 0){
    char c = (char)Serial.read();

    if(c == '\n'){
      float linear_cmd = 0.0f;
      float angular_cmd = 0.0f;

      if(packetParser(rx_buffer, linear_cmd, angular_cmd)){
        latest_linear_cmd = linear_cmd;
        latest_angular_cmd = angular_cmd;

        computeTargetVoltages(latest_linear_cmd, latest_angular_cmd, target_x_v, target_y_v);
        last_packet_time = millis();

        Serial.print("Received v= ");
        Serial.print(latest_linear_cmd, 3);
        Serial.print(" Received w= ");
        Serial.println(latest_angular_cmd, 3);
        Serial.print(" -> Target X Voltage= ");
        Serial.print(target_x_v, 3);
        Serial.print(" Target Y Voltage= ");
        Serial.println(target_y_v, 3);
      }
      else{
        Serial.print("Parse failed");
        Serial.println(rx_buffer);
      }

      rx_buffer = "";
    }
    else{
      rx_buffer += c;

      if(rx_buffer.length() > 64)
        rx_buffer = "";
    }
  }
}


void setup(){
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if(!mcp.begin()){
    Serial.println("MCP4728 not found");
    while(1){
      delay(1000);
    }
  }

  writeVoltages(X_NEUTRAL_V, Y_NEUTRAL_V);
  last_packet_time = millis();

  Serial.println("ESPP32 wheelchair bridge ready");
}


void loop(){
  handleSerialInput();

  // Packet timeout -> Neutral
  if(millis() - last_packet_time > CMD_TIMEOUT_MS){
    target_x_v = X_NEUTRAL_V;
    target_y_v = Y_NEUTRAL_V;
  }

  current_x_v = rampV(current_x_v, target_x_v, MAX_VOLT_STEP);
  current_y_v = rampV(current_y_v, target_y_v, MAX_VOLT_STEP);

  writeVoltages(current_x_v, current_y_v);

  delay(LOOP_DT_MS);
}