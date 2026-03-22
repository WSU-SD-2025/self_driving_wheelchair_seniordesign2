#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// I2C pins
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// DAC reference voltage
const float DAC_VREF = 5.0f;

// CH A -> Yellow wire (Y)
// CH B -> Blue wire (X)
const uint8_t YELLOW_CH = MCP4728_CHANNEL_A;
const uint8_t BLUE_CH   = MCP4728_CHANNEL_B;

// Initial neutral guess
float x_voltage = 2.68f;
float y_voltage = 2.69f;

// Step size for manual tuning
float step_size = 0.10f;

// Keep neutral output during wheelchair boot
const int BOOT_NEUTRAL_HOLD_MS = 10000;

// Clamp helper
float clampVoltage(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > DAC_VREF) return DAC_VREF;
  return v;
}

// Convert voltage to 12-bit DAC code
uint16_t voltageToCode(float voltage) {
  voltage = clampVoltage(voltage);
  return (uint16_t)((voltage / DAC_VREF) * 4095.0f);
}

// Write one channel
void writeChannelVoltage(uint8_t channel, float voltage) {
  uint16_t code = voltageToCode(voltage);

  switch (channel) {
    case MCP4728_CHANNEL_A:
      mcp.setChannelValue(MCP4728_CHANNEL_A, code);
      break;
    case MCP4728_CHANNEL_B:
      mcp.setChannelValue(MCP4728_CHANNEL_B, code);
      break;
    case MCP4728_CHANNEL_C:
      mcp.setChannelValue(MCP4728_CHANNEL_C, code);
      break;
    case MCP4728_CHANNEL_D:
      mcp.setChannelValue(MCP4728_CHANNEL_D, code);
      break;
  }
}

// Write both Y and X
void writeXYVoltages(float y, float x) {
  y_voltage = clampVoltage(y);
  x_voltage = clampVoltage(x);

  writeChannelVoltage(YELLOW_CH, y_voltage);
  writeChannelVoltage(BLUE_CH, x_voltage);

  Serial.print("Y = ");
  Serial.print(y_voltage, 3);
  Serial.print(" V  |  X = ");
  Serial.print(x_voltage, 3);
  Serial.print(" V  |  step = ");
  Serial.println(step_size, 2);
}

// Set current voltages as output again
void applyCurrentVoltages() {
  writeXYVoltages(y_voltage, x_voltage);
}

// Force known neutral guess
void setNeutral() {
  y_voltage = 2.49f;
  x_voltage = 2.47f;
  applyCurrentVoltages();
}

// Hold neutral during wheelchair startup
void holdNeutralForBoot(int hold_ms) {
  Serial.println("Holding neutral for boot...");
  unsigned long start = millis();

  while (millis() - start < (unsigned long)hold_ms) {
    applyCurrentVoltages();
    delay(100);
  }

  Serial.println("Boot neutral hold complete.");
}

// Print current status
void printStatus() {
  Serial.println();
  Serial.println("=== Current Output Status ===");
  Serial.print("Y voltage : ");
  Serial.print(y_voltage, 4);
  Serial.println(" V");

  Serial.print("X voltage : ");
  Serial.print(x_voltage, 4);
  Serial.println(" V");

  Serial.print("Step size : ");
  Serial.print(step_size, 4);
  Serial.println(" V");
  Serial.println("=============================");
  Serial.println();
}

// Print command menu
void printMenu() {
  Serial.println();
  Serial.println("=== Manual Voltage Tuning ===");
  Serial.println("w : Y + step");
  Serial.println("s : Y - step");
  Serial.println("a : X - step");
  Serial.println("d : X + step");
  Serial.println("z : step + 0.01");
  Serial.println("c : step - 0.01");
  Serial.println("n : reset to initial neutral guess");
  Serial.println("p : print current status");
  Serial.println("=============================");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing ESP32 + MCP4728 DAC ...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mcp.begin()) {
    Serial.println("MCP4728 not found.");
    while (1) {
      delay(10);
    }
  }

  // Start from current guessed neutral
  applyCurrentVoltages();
  Serial.println("Initial neutral output started.");

  // Important: wheelchair should see neutral during boot
  holdNeutralForBoot(BOOT_NEUTRAL_HOLD_MS);

  printStatus();
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'w' || cmd == 'W') {
      y_voltage += step_size;
      applyCurrentVoltages();
    }
    else if (cmd == 's' || cmd == 'S') {
      y_voltage -= step_size;
      applyCurrentVoltages();
    }
    else if (cmd == 'a' || cmd == 'A') {
      x_voltage -= step_size;
      applyCurrentVoltages();
    }
    else if (cmd == 'd' || cmd == 'D') {
      x_voltage += step_size;
      applyCurrentVoltages();
    }
    else if (cmd == 'z' || cmd == 'Z') {
      step_size += 0.01f;
      if (step_size > 1.00f) step_size = 1.00f;
      Serial.print("Step increased to: ");
      Serial.println(step_size, 4);
    }
    else if (cmd == 'c' || cmd == 'C') {
      step_size -= 0.01f;
      if (step_size < 0.001f) step_size = 0.001f;
      Serial.print("Step decreased to: ");
      Serial.println(step_size, 4);
    }
    else if (cmd == 'n' || cmd == 'N') {
      setNeutral();
      Serial.println("Reset to initial neutral guess.");
    }
    else if (cmd == 'p' || cmd == 'P') {
      printStatus();
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}