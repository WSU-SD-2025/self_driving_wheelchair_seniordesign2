#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <math.h>

Adafruit_MCP4728 mcp;

// GPIO Pins for Wheelchair
const int SDA_PIN = 8;
const int SCL_PIN = 9;

const float DAC_VREF = 5.0f;

const uint8_t YELLOW_CH = MCP4728_CHANNEL_A;
const uint8_t BLUE_CH = MCP4728_CHANNEL_B;

// GPIO pins for Encoders
const int LEFT_ENCODER_A = 4;
const int LEFT_ENCODER_B = 5;
const int RIGHT_ENCODER_A = 16;
const int RIGHT_ENCODER_B = 17;

// Encoder Direction Sign
const int LEFT_SIGN = 1;
const int RIGHT_SIGN = 1;

// Encoder counters
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// Voltage Limits
const float MIN_VOLTAGE = 1.0f;
const float MAX_VOLTAGE = 4.2f;

// Neutral Center Voltage
const float X_CENTER = 2.6915f;
const float Y_CENTER = 2.689f;

// Experiment timing
const unsigned long BOOT_NEUTRAL_HOLD_MS = 10000; // Hold neutral for 10 seconds on boot
const unsigned long LOG_PERIOD_MS = 20;           // Log every 20ms (50Hz)
const unsigned long VOLTAGE_UPDATE_MS = 100;      // Update voltage every 100ms
const unsigned long HOLD_TIME_MS = 5000;          // Hold each position for 5 seconds

// Experiment stepping
const float RAMP_STEP = 0.01f;
const float TARGET_STEP = 0.25f;

// Runtime Voltage
float x_voltage = X_CENTER;
float y_voltage = Y_CENTER;

// Phase enum
enum PhaseType {
    PHASE_BOOT_NEUTRAL = 0,
    PHASE_X_ONLY = 1,
    PHASE_Y_ONLY = 2,
    PHASE_BOTH = 3,
    PHASE_OPPOSITE = 4,
    PHASE_DONE = 99
};

// Step definition
struct TestStep {
    int phase;
    int step_idx;
    float target_x;
    float target_y;
};

const int MAX_STEPS = 400;
TestStep steps[MAX_STEPS];
int total_steps = 0;
int current_step_index = 0;

// State variables
bool experiment_started = false;
bool target_reached = false;

float target_x = X_CENTER;
float target_y = Y_CENTER;

unsigned long boot_start_time = 0;
unsigned long last_log_time = 0;
unsigned long last_voltage_update_time = 0;
unsigned long hold_start_time = 0;

// ================================= ENCODER ==================================
// Encoder ISR
void IRAM_ATTR updateLeftEncoder() {
    if (digitalRead(LEFT_ENCODER_B) != digitalRead(LEFT_ENCODER_A))
        left_encoder_count += LEFT_SIGN; // Forward
    else
        left_encoder_count -= LEFT_SIGN; // Backward
}

void IRAM_ATTR updateRightEncoder() {
    if (digitalRead(RIGHT_ENCODER_B) != digitalRead(RIGHT_ENCODER_A))
        right_encoder_count += RIGHT_SIGN;
    else
        right_encoder_count -= RIGHT_SIGN;
}

void initEncoders() {
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);
}

void resetEncoders() {
    noInterrupts();
    left_encoder_count = 0;
    right_encoder_count = 0;
    interrupts();
}

long readLeftEncoderCount() {
    noInterrupts();
    long count = left_encoder_count;
    interrupts();
    return count;
}

long readRightEncoderCount() {
    noInterrupts();
    long count = right_encoder_count;
    interrupts();
    return count;
}
// ============================================================================




// ================================ Wheelchair =================================
// Clamp
float clamp(float v) {
    if (v < MIN_VOLTAGE) return MIN_VOLTAGE;
    if (v > MAX_VOLTAGE) return MAX_VOLTAGE;
    return v;
}

// Convert voltage to 12-bit DAC code
uint16_t voltageToBit(float voltage) {
    voltage = clamp(voltage);
    return (uint16_t)((voltage / DAC_VREF) * 4095.0f);
}

// Write DAC channel
void writeDAC(uint8_t channel, float voltage) {
    uint16_t dac_value = voltageToBit(voltage);

    switch (channel) {
        case MCP4728_CHANNEL_A:
            mcp.setChannelValue(MCP4728_CHANNEL_A, dac_value);
            break;
        case MCP4728_CHANNEL_B:
            mcp.setChannelValue(MCP4728_CHANNEL_B, dac_value);
            break;
        case MCP4728_CHANNEL_C:
            mcp.setChannelValue(MCP4728_CHANNEL_C, dac_value);
            break;
        case MCP4728_CHANNEL_D:
            mcp.setChannelValue(MCP4728_CHANNEL_D, dac_value);
            break;
    }
}

// Write Y and X
void writeXYVoltages(float y, float x) {
    y_voltage = clamp(y);
    x_voltage = clamp(x);

    writeDAC(YELLOW_CH, y_voltage);
    writeDAC(BLUE_CH, x_voltage);
}

// Apply current output voltage
void applyCurrentVoltages() {
    writeXYVoltages(y_voltage, x_voltage);
}

// Set center neutral
void setNeutral() {
    y_voltage = Y_CENTER;
    x_voltage = X_CENTER;
    applyCurrentVoltages();
}

void printCSVHeader() {
    Serial.println("time_ms,phase,step_idx,x_v,y_v,left_count,right_count");
}

void logCSVRow(int phase, int step_idx) {
    unsigned long t = millis();
    long lc = readLeftEncoderCount();
    long rc = readRightEncoderCount();

    Serial.print(t); Serial.print(",");
    Serial.print(phase); Serial.print(",");
    Serial.print(step_idx); Serial.print(",");
    Serial.print(x_voltage, 4); Serial.print(",");
    Serial.print(y_voltage, 4); Serial.print(",");
    Serial.print(lc); Serial.print(",");
    Serial.println(rc);
}

// Math helpers
bool nearlyEqual(float a, float b, float eps = 1e-5f) {
    return fabs(a - b) < eps;
}

float rampToward(float current, float target, float step) {
    if (fabs(target - current) <= step) return target;
    if (target > current) return current + step;
    return current - step;
}

// Step plan
void addStep(int phase, int step, float tx, float ty) {
    if (total_steps >= MAX_STEPS) return;

    steps[total_steps].phase = phase;
    steps[total_steps].step_idx = step;
    steps[total_steps].target_x = clamp(tx);
    steps[total_steps].target_y = clamp(ty);
    total_steps++;
}

void addCenterReturn(int phase, int step) {
    addStep(phase, step, X_CENTER, Y_CENTER);
}

void buildSingleAxisSweepX() {
    int step = 0;

    for (float tx = X_CENTER + TARGET_STEP; tx <= MAX_VOLTAGE + 1e-6f; tx += TARGET_STEP) {
        addStep(PHASE_X_ONLY, step++, tx, Y_CENTER);
        addCenterReturn(PHASE_X_ONLY, step++);
    }

    for (float tx = X_CENTER - TARGET_STEP; tx >= MIN_VOLTAGE - 1e-6f; tx -= TARGET_STEP) {
        addStep(PHASE_X_ONLY, step++, tx, Y_CENTER);
        addCenterReturn(PHASE_X_ONLY, step++);
    }
}

void buildSingleAxisSweepY() {
    int step = 0;

    for (float ty = Y_CENTER + TARGET_STEP; ty <= MAX_VOLTAGE + 1e-6f; ty += TARGET_STEP) {
        addStep(PHASE_Y_ONLY, step++, X_CENTER, ty);
        addCenterReturn(PHASE_Y_ONLY, step++);
    }

    for (float ty = Y_CENTER - TARGET_STEP; ty >= MIN_VOLTAGE - 1e-6f; ty -= TARGET_STEP) {
        addStep(PHASE_Y_ONLY, step++, X_CENTER, ty);
        addCenterReturn(PHASE_Y_ONLY, step++);
    }
}

void buildBothSweep() {
    int step = 0;

    for (float d = TARGET_STEP; ; d += TARGET_STEP) {
        float xp = X_CENTER + d;
        float yp = Y_CENTER + d;
        if (xp > MAX_VOLTAGE || yp > MAX_VOLTAGE) break;
        addStep(PHASE_BOTH, step++, xp, yp);
        addCenterReturn(PHASE_BOTH, step++);
    }

    for (float d = TARGET_STEP; ; d += TARGET_STEP) {
        float xp = X_CENTER - d;
        float yp = Y_CENTER - d;
        if (xp < MIN_VOLTAGE || yp < MIN_VOLTAGE) break;
        addStep(PHASE_BOTH, step++, xp, yp);
        addCenterReturn(PHASE_BOTH, step++);
    }
}

void buildBothOppositeSweep() {
    int step = 0;

    for (float d = TARGET_STEP; ; d += TARGET_STEP) {
        float xp = X_CENTER + d;
        float yp = Y_CENTER - d;
        if (xp > MAX_VOLTAGE || yp < MIN_VOLTAGE) break;
        addStep(PHASE_OPPOSITE, step++, xp, yp);
        addCenterReturn(PHASE_OPPOSITE, step++);
    }

    for (float d = TARGET_STEP; ; d += TARGET_STEP) {
        float xp = X_CENTER - d;
        float yp = Y_CENTER + d;
        if (xp < MIN_VOLTAGE || yp > MAX_VOLTAGE) break;
        addStep(PHASE_OPPOSITE, step++, xp, yp);
        addCenterReturn(PHASE_OPPOSITE, step++);
    }
}

void buildExperimentPlan() {
    total_steps = 0;
    buildSingleAxisSweepX();
    buildSingleAxisSweepY();
    buildBothSweep();
    buildBothOppositeSweep();
}

// Step progression
void startCurrentStep() {
    if (current_step_index >= total_steps) return;

    target_x = steps[current_step_index].target_x;
    target_y = steps[current_step_index].target_y;
    target_reached = false;
    hold_start_time = 0;

    Serial.print("# START_STEP,phase=");
    Serial.print(steps[current_step_index].phase);
    Serial.print(",step_idx=");
    Serial.print(steps[current_step_index].step_idx);
    Serial.print(",target_x=");
    Serial.print(target_x, 4);
    Serial.print(",target_y=");
    Serial.println(target_y, 4);
}

void advanceToNextStep() {
    current_step_index++;
    if (current_step_index < total_steps) {
        startCurrentStep();
    } else {
        Serial.println("# EXPERIMENT_DONE");
    }
}

void updateExperiment() {
    if (current_step_index >= total_steps) return;

    unsigned long now = millis();

    if (now - last_voltage_update_time >= VOLTAGE_UPDATE_MS) {
        last_voltage_update_time = now;

        if (!target_reached) {
            x_voltage = rampToward(x_voltage, target_x, RAMP_STEP);
            y_voltage = rampToward(y_voltage, target_y, RAMP_STEP);
            applyCurrentVoltages();

            if (nearlyEqual(x_voltage, target_x) && nearlyEqual(y_voltage, target_y)) {
                target_reached = true;
                hold_start_time = now;

                Serial.print("# TARGET_REACHED,phase=");
                Serial.print(steps[current_step_index].phase);
                Serial.print(",step_idx=");
                Serial.println(steps[current_step_index].step_idx);
            }
        } else {
            if (now - hold_start_time >= HOLD_TIME_MS) {
                advanceToNextStep();
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("# EXPERIMENT_START");

    Wire.begin(SDA_PIN, SCL_PIN);

    if (!mcp.begin()) {
        Serial.println("MCP4728 not found.");
        while (1) {
            delay(10);
        }
    }

    setNeutral();

    initEncoders();
    resetEncoders();

    printCSVHeader();

    buildExperimentPlan();

    boot_start_time = millis();
    last_log_time = millis();
    last_voltage_update_time = millis();

    Serial.println("# BOOT_NEUTRAL_START");
}

void loop() {
    unsigned long now = millis();

    if (now - last_log_time >= LOG_PERIOD_MS) {
        last_log_time = now;

        if (!experiment_started) {
            logCSVRow(PHASE_BOOT_NEUTRAL, -1);
        }
        else if (current_step_index < total_steps) {
            logCSVRow(steps[current_step_index].phase, steps[current_step_index].step_idx);
        }
        else {
            logCSVRow(PHASE_DONE, -1);
        }
    }

    if (!experiment_started) {
        setNeutral();

        if (now - boot_start_time >= BOOT_NEUTRAL_HOLD_MS) {
            experiment_started = true;
            current_step_index = 0;
            startCurrentStep();
            Serial.println("# BOOT_NEUTRAL_END");
        }
        return;
    }

    if (current_step_index < total_steps) {
        updateExperiment();
    }
    else {
        setNeutral();
    }
}