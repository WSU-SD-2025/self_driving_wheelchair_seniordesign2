#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// I2C pins
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// Voltage settings
// 12-bit DAC: 0 ~ 4095
// DAC VDD = 5.0V
const float DAC_VREF = 5.0f;

// CH A -> Yellow wire (y)
// CH B -> Blue wire (x)
const uint8_t YELLOW_CH = MCP4728_CHANNEL_A;
const uint8_t BLUE_CH = MCP4728_CHANNEL_B;



// Neutral voltage
const float X_CENTER_V = 2.50f;
const float Y_CENTER_V = 2.50f;

//Forward
const float Y_FORWARD_V = 4.00f;
const float X_FORWARD_V = 0.00f;

// Backward
const float Y_BACKWARD_V = 1.50f;
const float X_BACKWARD_V = 0.00f;

// Right
const float Y_RIGHT_V = 0.00f;
const float X_RIGHT_V = 4.00f;

// Left
const float Y_LEFT_V = 0.00f;
const float X_LEFT_V = 0.90f;



// Convert voltage to 12-bit DAC code
uint16_t voltageToCode(float voltage) {

  //= Clamp voltage to valid range
  if (voltage < 0.0f) voltage = 0.0f;
  if (voltage > DAC_VREF) voltage = DAC_VREF;

  return (uint16_t)((voltage / DAC_VREF) * 4095.0f);
}


// Write voltage to one channel
void writeChannelVoltage(uint8_t channel, float voltage) {
  uint16_t code = voltageToCode(voltage);

  switch (channel) {
    case 0:
      mcp.setChannelValue(MCP4728_CHANNEL_A, code);
      break;
    case 1:
      mcp.setChannelValue(MCP4728_CHANNEL_B, code);
      break;
    case 2:
      mcp.setChannelValue(MCP4728_CHANNEL_C, code);
      break;
    case 3:
      mcp.setChannelValue(MCP4728_CHANNEL_D, code);
      break;
  }

  Serial.print("CH");
  Serial.print(channel);
  Serial.print(" voltage = ");
  Serial.print(voltage, 3);
  Serial.print(" V, code = ");
  Serial.println(code);
}

// Write both X and Y together
void writeXYVoltages(float y_voltage, float x_voltage){
    writeChannelVoltage(YELLOW_CH, y_voltage);
    writeChannelVoltage(BLUE_CH, x_voltage);
}

// Stop wheelchair = center on both axes
void setNeutral() {
    Serial.println("Neutral");
    writeXYVoltages(Y_CENTER_V, X_CENTER_V);
}

// Forward test
void setForwardTest() {
    Serial.println("Forward test");
    writeXYVoltages(Y_FORWARD_V, X_FORWARD_V);
}

// Backward test
void setBackwardTest() {
    Serial.println("Backward test");
    writeXYVoltages(Y_BACKWARD_V, X_BACKWARD_V);
}

// Right test
void setRightTest() {
    Serial.println("Right test");
    writeXYVoltages(Y_RIGHT_V, X_RIGHT_V);
}

// Left test
void setLeftTest() {
    Serial.println("Left test");
    writeXYVoltages(Y_LEFT_V, X_LEFT_V);
}






void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("MCP4728 connected.");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mcp.begin()) {
    Serial.println("MCP4728 not found. Check wiring.");
    while (1) delay(10);
  }

  // Start with neutral for safety
  setNeutral();
  Serial.println("Set neutral.");
  delay(3000);

  // Short forward pulse
  setForwardTest();
  Serial.println("Applying forward command");
  delay(5000);

  // Return to Neutral
  setNeutral();
  Serial.println("Returned to neutral");
  delay(3000);
}

void loop() {
  
}