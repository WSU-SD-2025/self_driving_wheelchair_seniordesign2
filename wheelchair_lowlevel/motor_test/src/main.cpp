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
const float X_CENTER_V = 2.47f;
const float Y_CENTER_V = 2.49f;

//Forward (max speed)
const float Y_FORWARD_V = 4.23f;
const float X_FORWARD_V = 0.00f;

// Backward
const float Y_BACKWARD_V = 1.17f;
const float X_BACKWARD_V = 0.00f;

// Right
const float Y_RIGHT_V = 0.00f;
const float X_RIGHT_V = 4.18f;

// Left
const float Y_LEFT_V = 0.00f;
const float X_LEFT_V = 0.81f;


// Timing / Ramp Settings
const int BOOT_NEUTRAL_HOLD_MS = 10000;
const int HOLD_PEAK_MS = 2000;
const int RAMP_STEPS = 80;
const int RAMP_DELAY_MS = 25;


// Current output
float current_y = Y_CENTER_V;
float current_x = X_CENTER_V;



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

// Write both X and Y together
void writeXYVoltages(float y_voltage, float x_voltage){
  writeChannelVoltage(YELLOW_CH, y_voltage);
  writeChannelVoltage(BLUE_CH, x_voltage);

  current_y = y_voltage;
  current_x = x_voltage;

  Serial.print("Y = ");
  Serial.print(y_voltage, 3);
  Serial.print(" V, X = ");
  Serial.print(x_voltage, 3);
  Serial.println(" V");
}

// Stop wheelchair = center on both axes
void setNeutral() {
  writeXYVoltages(Y_CENTER_V, X_CENTER_V);
}


// Ramp from current position to target position
void rampTo(float target_y, float target_x, int steps, int delay_ms){
  float start_y = current_y;
  float start_x = current_x;

  for(int i = 1; i<=steps; i++){
    float t = (float)i / (float) steps;
    float y = start_y + (target_y - start_y) * t;
    float x = start_x + (target_x - start_x) * t;
    writeXYVoltages(y, x);
    delay(delay_ms);
  }
}


void moveOnce(float target_y, float target_x, const char* label){
  Serial.println();
  Serial.print("Command: ");
  Serial.println(label);

  Serial.println("Ramp up...");
  rampTo(target_y, target_x, RAMP_STEPS, RAMP_DELAY_MS);

  Serial.println("Hold peak...");
  delay(HOLD_PEAK_MS);

  Serial.println("Ramp down to neutral...");
  rampTo(Y_CENTER_V, X_CENTER_V, RAMP_STEPS, RAMP_DELAY_MS);

  Serial.println("Back to neutral");
  setNeutral();
}


void printMenu(){
  Serial.println();
  Serial.println("=== Commands ===");
  Serial.println("w : forward once");
  Serial.println("s : backward once");
  Serial.println("a : left once");
  Serial.println("d : right once");
  Serial.println("n : force neutral");
  Serial.println("================");
  Serial.println();
}


void holdNeutralForBoot(int hold_ms){
  Serial.println("Holding neutral for boot...");
  unsigned long start = millis();

  while(millis() - start < (unsigned long)hold_ms){
    setNeutral();
    delay(100);
  }

  Serial.println("Boot neutral hold complete.");
}





void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Initializing ESP32 + MCP4728 DAC ...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if(!mcp.begin()){
    Serial.println("MCP4728 not found");
    while(1){
      delay(10);
    }
  }

  setNeutral();
  Serial.println("Neutral output started.");
  
  holdNeutralForBoot(BOOT_NEUTRAL_HOLD_MS);

  printMenu();
}

void loop() {
  if(Serial.available() > 0){
    char cmd = Serial.read();

    if(cmd == 'w' || cmd == 'W')
      moveOnce(Y_FORWARD_V, X_FORWARD_V, "FORWARD");

    else if(cmd == 's' || cmd == 'S')
      moveOnce(Y_BACKWARD_V, X_BACKWARD_V, "BACKWARD");

    else if(cmd == 'a' || cmd == 'A')
      moveOnce(Y_LEFT_V, X_LEFT_V, "LEFT");

    else if(cmd == 'd' || cmd == 'D')
      moveOnce(Y_RIGHT_V, X_RIGHT_V, "RIGHT");

    else if(cmd == 'n' || cmd == 'N')
      setNeutral();

    while(Serial.available() > 0)
      Serial.read();

    printMenu();
  }
}