#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 dac;

// GPIO for DAC
const int DAC_SDA = 8;
const int DAC_SCL = 9;  

// DAC Voltage
const float DAC_VOLTAGE = 5.0f;


// Voltage for Wheelchair
const float X_CENTER = 2.50f;
const float X_MIN = 0.81f;
const float X_MAX = 4.18f;

const float Y_CENTER = 2.50f;
const float Y_MIN = 1.17f;
const float Y_MAX = 4.23f;


// Configuration
// Ramping
const float STEP_V = 0.01f;     // ramping step (Voltage per update)
// Deadzone
float DEADZONE = 0.05f;
//update period
const int LOOP_MS = 20;


// Current Output Voltage
float X_OUT = X_CENTER;
float Y_OUT = Y_CENTER;


// Command (-1 to 1)
float X_CMD = 0.0f;
float Y_CMD = 0.0f;
// Keyboard step
float cmd_step = 0.05f;
// Timeout for command
uint32_t cmd_timeout_ms = 1000;
uint32_t last_cmd_time = 0;




// Mapping to drive motor(-1 to 1   ->   Voltage) 
float axisToVoltage(float input, float centerV, float minV, float maxV, float deadzone){
    // deadzone
    if(fabsf(input) < deadzone) input = 0.0f;

    // clamp
    if(input > 1.0f) input = 1.0f;
    if(input < -1.0f) input = -1.0f;

    if(input >= 0.0f){
        return centerV + input * (maxV - centerV);      // Forward & Right
    } else {
        return centerV + input * (centerV - minV);      // Backward & Left
    }
}


float ramping(float current, float target, float step){
    float diff = target - current;

    if(fabsf(diff) <= step) return target;
    else return current + (diff > 0 ? step : -step);
}


// Voltage to Analog    (0.0V to 5.0V -> 0 to 4095)
uint16_t voltageToAnalog(float voltage){
    if(voltage < 0.0f)  voltage = 0.0f;
    if(voltage > DAC_VOLTAGE)   voltage = DAC_VOLTAGE;

    uint16_t output = (uint16_t)lroundf(voltage / DAC_VOLTAGE * 4095.0f);
    return output;
}


// Write X/Y to DAC
void writeXY_DAC(float x_voltage, float y_voltage){
    uint16_t x_analog = voltageToAnalog(x_voltage);
    uint16_t y_analog = voltageToAnalog(y_voltage);

    dac.setChannelValue(MCP4728_CHANNEL_A, x_analog);
    dac.setChannelValue(MCP4728_CHANNEL_B, y_analog);
}


void printStatus(){
    Serial.printf("cmd_step=%.3f | X_CMD=%.3f Y_CMD=%.3f | X_OUT=%.3f Y_OUT=%.3f\n", cmd_step, X_CMD, Y_CMD, X_OUT, Y_OUT);
}


void setup(){
    Serial.begin(115200);
    delay(300);

    Wire.begin(DAC_SDA, DAC_SCL);

    if(!dac.begin())
    {
        Serial.println("MCP4728 not found");
        while(1)    delay(100);
    }

    // Start (Set to center)
    X_OUT = X_CENTER;
    Y_OUT = Y_CENTER;
    writeXY_DAC(X_OUT, Y_OUT);

    // Initialize timeout timer
    last_cmd_time = millis();

    Serial.println("Setup Complete");
    Serial.println("Controls:");
    Serial.println("\tw: Forward\ts: Backward");
    Serial.println("\ta: Left\t\td: Right");
    Serial.println("\tspace: Stop");
    Serial.println("\tq: decrease cmd step\te: increase cmd step");
    printStatus();
}

void loop(){
    static uint32_t lastUpdatedTime = 0;
    static bool timed_out = false;

    // Keyboard Control
    while (Serial.available() > 0){
        char c = (char)Serial.read();

        if(c == '\n' || c == '\r') continue; // ignore newline

        last_cmd_time = millis(); // update last command time
        timed_out = false; // reset timeout flag

        if(c == 'w') Y_CMD += cmd_step; // Forward
        else if(c == 's') Y_CMD -= cmd_step; // Backward
        else if(c == 'a') X_CMD -= cmd_step; // Left
        else if(c == 'd') X_CMD += cmd_step; // Right
        else if(c == ' ') { X_CMD = 0.0f; Y_CMD = 0.0f; } // Stop
        else if(c == 'q') cmd_step = fmaxf(0.02f, cmd_step - 0.01f); // decrease step
        else if(c == 'e') cmd_step = fminf(0.50f, cmd_step + 0.01f); // increase step

        X_CMD = fminf(1.0f, fmaxf(-1.0f, X_CMD));
        Y_CMD = fminf(1.0f, fmaxf(-1.0f, Y_CMD));
        printStatus();
    }

    // Update Period
    if(millis() - lastUpdatedTime < LOOP_MS) return;
    lastUpdatedTime = millis();

    // Timeout (No input -> Stop)
    if(millis() - last_cmd_time > cmd_timeout_ms){
        X_CMD = 0.0f;
        Y_CMD = 0.0f;
        
        if(!timed_out){
            Serial.println("Timeout: STOP");
            timed_out = true;
        }
    }

    // CMD to voltage
    float x_target = axisToVoltage(X_CMD, X_CENTER, X_MIN, X_MAX, DEADZONE);
    float y_target = axisToVoltage(Y_CMD, Y_CENTER, Y_MIN, Y_MAX, DEADZONE);

    // Ramping
    X_OUT = ramping(X_OUT, x_target, STEP_V);
    Y_OUT = ramping(Y_OUT, y_target, STEP_V);

    // Write to DAC
    writeXY_DAC(X_OUT, Y_OUT);

}