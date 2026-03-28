#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// DAC GPIO pins
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// DAC reference voltage
const float DAC_VREF = 5.0f;

// CH A -> Y, CH B -> X
const uint8_t YELLOW_CH = MCP4728_CHANNEL_A;
const uint8_t BLUE_CH = MCP4728_CHANNEL_B;



// Encoder GPIO pins
const int LEFT_ENCODER_A = 4;
const int LEFT_ENCODER_B = 5;
const int RIGHT_ENCODER_A = 16;
const int RIGHT_ENCODER_B = 17;

// Encoder Sign
const int LEFT_SIGN = 1;
const int RIGHT_SIGN = 1;

volatile long left_count = 0;
volatile long right_count = 0;

struct EncoderSnapshot {
    long left_current;
    long right_current;
    unsigned long time_ms;
};



// Neutral values
const float Y_NEUTRAL = 2.689f;
const float X_NEUTRAL = 2.691f;

// Current command voltages
float y_voltage = Y_NEUTRAL;
float x_voltage = X_NEUTRAL;

// Keep neutral during wheelchair boot
const int BOOT_NEUTRAL_HOLD_MS = 2000;



// Communication / timing
const unsigned long CMD_TIMEOUT_MS = 500;   // if Simulink stops, go neutral
const unsigned long TX_PERIOD_MS = 20;     // Send encoder counts at 50Hz

unsigned long last_cmd_ms = 0;
unsigned long last_tx_ms = 0;



float clamp(float v){
    if(v < 0.0f) return 0.0f;
    if(v > DAC_VREF) return DAC_VREF;
    return v;
}

uint16_t voltageToCode(float v){
    v = clamp(v);
    return (uint16_t)((v / DAC_VREF) * 4095.0f);
}

void writeChannelVoltage(uint8_t channel, float v){
    uint16_t code = voltageToCode(v);

    switch(channel){
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

void writeXYVoltages(float y, float x){
    y_voltage = clamp(y);
    x_voltage = clamp(x);

    writeChannelVoltage(YELLOW_CH, y_voltage);
    writeChannelVoltage(BLUE_CH, x_voltage);
}

void applyCurrentVoltages(){
    writeXYVoltages(y_voltage, x_voltage);
}

void setNeutral(){
    y_voltage = Y_NEUTRAL;
    x_voltage = X_NEUTRAL;

    applyCurrentVoltages();
}

void holdNeutralForBoot(int hold_ms){
    unsigned long start = millis();
    while(millis() - start < (unsigned long)hold_ms){
        applyCurrentVoltages();
        delay(50);
    }
}


// Encoder ISR
void IRAM_ATTR updateLeftEncoder(){
    if(digitalRead(LEFT_ENCODER_B) != digitalRead(LEFT_ENCODER_A))
        left_count += LEFT_SIGN;
    else
        left_count -= LEFT_SIGN;
}

void IRAM_ATTR updateRightEncoder(){
    if(digitalRead(RIGHT_ENCODER_B) != digitalRead(RIGHT_ENCODER_A))
        right_count += RIGHT_SIGN;
    else
        right_count -= RIGHT_SIGN;
}

void initEncoders(){
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);
}

void resetEncoders(){
    noInterrupts();
    left_count = 0;
    right_count = 0;
    interrupts();
}

// Simulink I/O
// Simulink sends: [y, x] as float32 + float32 (8 bytes)

// ESP32 sends back: [time_ms, y_voltage, x_voltage, left_count, right_count] as float32[5] (20 bytes)

void handleIncomingCommand(){
    if(Serial.available() >= (int)(sizeof(float) * 2)){
        float cmd[2];
        size_t n = Serial.readBytes((char*)cmd, sizeof(cmd));

        if(n == sizeof(cmd)){
            float y_cmd = cmd[0];
            float x_cmd = cmd[1];

            writeXYVoltages(y_cmd, x_cmd);
            last_cmd_ms = millis();
        }
    }
}

void sendTelemetry(){
    unsigned long now = millis();
    if(now - last_tx_ms < TX_PERIOD_MS) return;

    long left_snapshot, right_snapshot;
    noInterrupts();
    left_snapshot = left_count;
    right_snapshot = right_count;
    interrupts();

    float out[5];
    out[0] = (float)now;
    out[1] = y_voltage;
    out[2] = x_voltage;
    out[3] = (float)left_snapshot;
    out[4] = (float)right_snapshot;
    Serial.write((uint8_t*)out, sizeof(out));
    last_tx_ms = now;
}



void setup(){
    Serial.begin(115200);
    delay(300);
    Serial.flush();
    Wire.begin(SDA_PIN, SCL_PIN);

    if(!mcp.begin()){
        while(1)
            delay(10);
    }

    initEncoders();
    resetEncoders();

    // Start Neutral
    setNeutral();

    // Important for wheelchair boot
    holdNeutralForBoot(BOOT_NEUTRAL_HOLD_MS);

    last_cmd_ms = millis();
    last_tx_ms = millis();
}

void loop(){
    // Receive Simulink command [y, x]
    handleIncomingCommand();
    
    // Timeout Safety
    if(millis() - last_cmd_ms > CMD_TIMEOUT_MS){
        setNeutral();
        last_cmd_ms = millis();
    }

    // Send encoder counts back to Simulink
    sendTelemetry();
}


