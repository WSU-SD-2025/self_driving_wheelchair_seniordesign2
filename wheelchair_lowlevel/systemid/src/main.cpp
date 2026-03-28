#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <math.h>

Adafruit_MCP4728 mcp;

// ============================= GPIO / Hardware =============================
const int SDA_PIN = 8;
const int SCL_PIN = 9;

const float DAC_VREF = 5.0f;

const uint8_t YELLOW_CH = MCP4728_CHANNEL_A;  // Y
const uint8_t BLUE_CH   = MCP4728_CHANNEL_B;  // X

// Encoder pins
const int LEFT_ENCODER_A  = 4;
const int LEFT_ENCODER_B  = 5;
const int RIGHT_ENCODER_A = 16;
const int RIGHT_ENCODER_B = 17;

// Encoder sign (keep as current)
const int LEFT_SIGN  = 1;
const int RIGHT_SIGN = 1;

// Encoder counters
volatile long left_encoder_count  = 0;
volatile long right_encoder_count = 0;

// ============================= Voltage Settings ============================
const float MIN_VOLTAGE = 1.0f;
const float MAX_VOLTAGE = 4.2f;

const float X_CENTER = 2.6915f;
const float Y_CENTER = 2.6890f;

// Step test voltage levels
const float STEP_CENTER_X = 2.5f;
const float STEP_CENTER_Y = 2.5f;
const float STEP_HIGH     = 3.5f;
const float STEP_LOW      = 1.5f;

// ============================= Timing Settings =============================
const unsigned long BOOT_NEUTRAL_HOLD_MS = 5000;

const unsigned long LOG_PERIOD_MS     = 20;   // 50 Hz
const unsigned long CONTROL_PERIOD_MS = 20;   // 50 Hz

const unsigned long TARGET_HOLD_MS      = 3000;  // ramp target hold
const unsigned long NEUTRAL_HOLD_MS     = 2000;  // between ramp motions
const unsigned long STEP_HOLD_MS        = 4000;  // hold after instant step
const unsigned long STEP_NEUTRAL_HOLD_MS= 2000;  // hold at 2.5 after step return

const float RAMP_RATE_V_PER_S = 0.20f;

// ============================= Runtime Voltage =============================
float x_voltage = X_CENTER;
float y_voltage = Y_CENTER;

// ============================= Phase Enum ==================================
enum PhaseType {
    PHASE_BOOT_NEUTRAL   = 0,
    PHASE_X_ONLY         = 1,
    PHASE_Y_ONLY         = 2,
    PHASE_BOTH_SAME      = 3,
    PHASE_BOTH_OPPOSITE  = 4,
    PHASE_DONE           = 99
};

// ============================= Segment Type ================================
enum SegmentMode {
    SEG_RAMP = 0,
    SEG_STEP = 1
};

// ============================= Segment Plan ================================
struct Segment {
    int phase;
    int step_idx;
    int mode;                // SEG_RAMP or SEG_STEP
    float target_x;
    float target_y;
    unsigned long hold_ms;
};

const int MAX_SEGMENTS = 80;
Segment segments[MAX_SEGMENTS];
int total_segments = 0;
int current_segment_index = 0;

// Segment runtime state
bool experiment_started = false;
bool segment_target_reached = false;
bool step_applied = false;
unsigned long segment_hold_start_ms = 0;

// Timers
unsigned long boot_start_time = 0;
unsigned long last_log_time = 0;
unsigned long last_control_time = 0;

// ================================ ENCODER ==================================
void IRAM_ATTR updateLeftEncoder() {
    if (digitalRead(LEFT_ENCODER_B) != digitalRead(LEFT_ENCODER_A))
        left_encoder_count += LEFT_SIGN;
    else
        left_encoder_count -= LEFT_SIGN;
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

// ================================ DAC / Output =============================
float clampVoltage(float v) {
    if (v < MIN_VOLTAGE) return MIN_VOLTAGE;
    if (v > MAX_VOLTAGE) return MAX_VOLTAGE;
    return v;
}

uint16_t voltageToBit(float voltage) {
    voltage = clampVoltage(voltage);
    return (uint16_t)((voltage / DAC_VREF) * 4095.0f);
}

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

void writeXYVoltages(float y, float x) {
    y_voltage = clampVoltage(y);
    x_voltage = clampVoltage(x);

    writeDAC(YELLOW_CH, y_voltage); // Y
    writeDAC(BLUE_CH,   x_voltage); // X
}

void applyCurrentVoltages() {
    writeXYVoltages(y_voltage, x_voltage);
}

void setNeutral() {
    x_voltage = X_CENTER;
    y_voltage = Y_CENTER;
    applyCurrentVoltages();
}

void setStepNeutral() {
    x_voltage = STEP_CENTER_X;
    y_voltage = STEP_CENTER_Y;
    applyCurrentVoltages();
}

// ================================ Logging ==================================
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

// ================================ Helpers ==================================
bool nearlyEqual(float a, float b, float eps = 0.002f) {
    return fabs(a - b) < eps;
}

float rampToward(float current, float target, float step) {
    if (fabs(target - current) <= step) return target;
    if (target > current) return current + step;
    return current - step;
}

// ================================ Plan Builder =============================
void addSegment(int phase, int step_idx, int mode, float tx, float ty, unsigned long hold_ms) {
    if (total_segments >= MAX_SEGMENTS) return;

    segments[total_segments].phase    = phase;
    segments[total_segments].step_idx = step_idx;
    segments[total_segments].mode     = mode;
    segments[total_segments].target_x = clampVoltage(tx);
    segments[total_segments].target_y = clampVoltage(ty);
    segments[total_segments].hold_ms  = hold_ms;
    total_segments++;
}

// X only step test around 2.5
void addXOnlyStepTest(int phase, int &s) {
    addSegment(phase, s++, SEG_STEP, STEP_HIGH, STEP_CENTER_Y, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_LOW, STEP_CENTER_Y, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
}

// Y only step test around 2.5
void addYOnlyStepTest(int phase, int &s) {
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_HIGH, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_LOW, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
}

// Both same direction step test
void addBothSameStepTest(int phase, int &s) {
    addSegment(phase, s++, SEG_STEP, STEP_HIGH, STEP_HIGH, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_LOW, STEP_LOW, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
}

// Opposite direction step test
void addBothOppositeStepTest(int phase, int &s) {
    addSegment(phase, s++, SEG_STEP, STEP_LOW,  STEP_HIGH, STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_HIGH, STEP_LOW,  STEP_HOLD_MS);
    addSegment(phase, s++, SEG_STEP, STEP_CENTER_X, STEP_CENTER_Y, STEP_NEUTRAL_HOLD_MS);
}

void buildExperimentPlan() {
    total_segments = 0;
    int s = 0;

    // ---------------------------------------------------------------------
    // 1) X only ramp: Y fixed at center
    //    X -> 4.2 -> neutral -> 1.0 -> neutral
    // ---------------------------------------------------------------------
    addSegment(PHASE_X_ONLY, s++, SEG_RAMP, MAX_VOLTAGE, Y_CENTER, TARGET_HOLD_MS);
    addSegment(PHASE_X_ONLY, s++, SEG_RAMP, X_CENTER,    Y_CENTER, NEUTRAL_HOLD_MS);
    addSegment(PHASE_X_ONLY, s++, SEG_RAMP, MIN_VOLTAGE, Y_CENTER, TARGET_HOLD_MS);
    addSegment(PHASE_X_ONLY, s++, SEG_RAMP, X_CENTER,    Y_CENTER, NEUTRAL_HOLD_MS);

    // X only step improvement
    addXOnlyStepTest(PHASE_X_ONLY, s);

    // ---------------------------------------------------------------------
    // 2) Y only ramp: X fixed at center
    //    Y -> 4.2 -> neutral -> 1.0 -> neutral
    // ---------------------------------------------------------------------
    addSegment(PHASE_Y_ONLY, s++, SEG_RAMP, X_CENTER, MAX_VOLTAGE, TARGET_HOLD_MS);
    addSegment(PHASE_Y_ONLY, s++, SEG_RAMP, X_CENTER, Y_CENTER,    NEUTRAL_HOLD_MS);
    addSegment(PHASE_Y_ONLY, s++, SEG_RAMP, X_CENTER, MIN_VOLTAGE, TARGET_HOLD_MS);
    addSegment(PHASE_Y_ONLY, s++, SEG_RAMP, X_CENTER, Y_CENTER,    NEUTRAL_HOLD_MS);

    // Y only step improvement
    addYOnlyStepTest(PHASE_Y_ONLY, s);

    // ---------------------------------------------------------------------
    // 3) Both same direction
    //    (X,Y) -> (4.2,4.2) -> neutral -> (1.0,1.0) -> neutral
    // ---------------------------------------------------------------------
    addSegment(PHASE_BOTH_SAME, s++, SEG_RAMP, MAX_VOLTAGE, MAX_VOLTAGE, TARGET_HOLD_MS);
    addSegment(PHASE_BOTH_SAME, s++, SEG_RAMP, X_CENTER,    Y_CENTER,    NEUTRAL_HOLD_MS);
    addSegment(PHASE_BOTH_SAME, s++, SEG_RAMP, MIN_VOLTAGE, MIN_VOLTAGE, TARGET_HOLD_MS);
    addSegment(PHASE_BOTH_SAME, s++, SEG_RAMP, X_CENTER,    Y_CENTER,    NEUTRAL_HOLD_MS);

    // Both same step improvement
    addBothSameStepTest(PHASE_BOTH_SAME, s);

    // ---------------------------------------------------------------------
    // 4) Opposite directions
    //    Case A: Y -> 4.2, X -> 1.0
    //    neutral
    //    Case B: Y -> 1.0, X -> 4.2
    //    neutral
    // ---------------------------------------------------------------------
    addSegment(PHASE_BOTH_OPPOSITE, s++, SEG_RAMP, MIN_VOLTAGE, MAX_VOLTAGE, TARGET_HOLD_MS);
    addSegment(PHASE_BOTH_OPPOSITE, s++, SEG_RAMP, X_CENTER,    Y_CENTER,    NEUTRAL_HOLD_MS);
    addSegment(PHASE_BOTH_OPPOSITE, s++, SEG_RAMP, MAX_VOLTAGE, MIN_VOLTAGE, TARGET_HOLD_MS);
    addSegment(PHASE_BOTH_OPPOSITE, s++, SEG_RAMP, X_CENTER,    Y_CENTER,    NEUTRAL_HOLD_MS);

    // Opposite step improvement
    addBothOppositeStepTest(PHASE_BOTH_OPPOSITE, s);
}

// ================================ Segment Exec =============================
void printSegmentStart(const Segment& seg) {
    Serial.print("# START_SEGMENT,phase=");
    Serial.print(seg.phase);
    Serial.print(",step_idx=");
    Serial.print(seg.step_idx);
    Serial.print(",mode=");
    Serial.print(seg.mode == SEG_RAMP ? "RAMP" : "STEP");
    Serial.print(",target_x=");
    Serial.print(seg.target_x, 4);
    Serial.print(",target_y=");
    Serial.print(seg.target_y, 4);
    Serial.print(",hold_ms=");
    Serial.println(seg.hold_ms);
}

void startCurrentSegment() {
    if (current_segment_index >= total_segments) return;

    segment_target_reached = false;
    step_applied = false;
    segment_hold_start_ms = 0;

    printSegmentStart(segments[current_segment_index]);
}

void advanceToNextSegment() {
    current_segment_index++;

    if (current_segment_index < total_segments) {
        startCurrentSegment();
    } else {
        Serial.println("# EXPERIMENT_DONE");
    }
}

void updateRampSegment(Segment &seg, unsigned long now) {
    float dt = CONTROL_PERIOD_MS / 1000.0f;
    float ramp_step = RAMP_RATE_V_PER_S * dt;

    if (!segment_target_reached) {
        x_voltage = rampToward(x_voltage, seg.target_x, ramp_step);
        y_voltage = rampToward(y_voltage, seg.target_y, ramp_step);
        applyCurrentVoltages();

        if (nearlyEqual(x_voltage, seg.target_x) && nearlyEqual(y_voltage, seg.target_y)) {
            x_voltage = seg.target_x;
            y_voltage = seg.target_y;
            applyCurrentVoltages();

            segment_target_reached = true;
            segment_hold_start_ms = now;

            Serial.print("# TARGET_REACHED,phase=");
            Serial.print(seg.phase);
            Serial.print(",step_idx=");
            Serial.println(seg.step_idx);
        }
    } else {
        if (now - segment_hold_start_ms >= seg.hold_ms) {
            advanceToNextSegment();
        }
    }
}

void updateStepSegment(Segment &seg, unsigned long now) {
    if (!step_applied) {
        x_voltage = seg.target_x;
        y_voltage = seg.target_y;
        applyCurrentVoltages();

        step_applied = true;
        segment_target_reached = true;
        segment_hold_start_ms = now;

        Serial.print("# STEP_APPLIED,phase=");
        Serial.print(seg.phase);
        Serial.print(",step_idx=");
        Serial.println(seg.step_idx);
    } else {
        if (now - segment_hold_start_ms >= seg.hold_ms) {
            advanceToNextSegment();
        }
    }
}

void updateExperiment() {
    if (current_segment_index >= total_segments) return;

    unsigned long now = millis();
    Segment &seg = segments[current_segment_index];

    if (seg.mode == SEG_RAMP) {
        updateRampSegment(seg, now);
    } else {
        updateStepSegment(seg, now);
    }
}

// ================================= Setup ===================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("# EXPERIMENT_START");

    Wire.begin(SDA_PIN, SCL_PIN);

    if (!mcp.begin()) {
        Serial.println("# ERROR: MCP4728 not found.");
        while (1) {
            delay(10);
        }
    }

    initEncoders();
    resetEncoders();

    setNeutral();

    printCSVHeader();
    buildExperimentPlan();

    boot_start_time = millis();
    last_log_time = millis();
    last_control_time = millis();

    Serial.println("# BOOT_NEUTRAL_START");
}

// ================================= Loop ====================================
void loop() {
    unsigned long now = millis();

    // Logging
    if (now - last_log_time >= LOG_PERIOD_MS) {
        last_log_time = now;

        if (!experiment_started) {
            logCSVRow(PHASE_BOOT_NEUTRAL, -1);
        } else if (current_segment_index < total_segments) {
            logCSVRow(
                segments[current_segment_index].phase,
                segments[current_segment_index].step_idx
            );
        } else {
            logCSVRow(PHASE_DONE, -1);
        }
    }

    // Boot neutral hold
    if (!experiment_started) {
        setNeutral();

        if (now - boot_start_time >= BOOT_NEUTRAL_HOLD_MS) {
            experiment_started = true;
            current_segment_index = 0;
            startCurrentSegment();
            Serial.println("# BOOT_NEUTRAL_END");
        }
        return;
    }

    // Control update
    if (now - last_control_time >= CONTROL_PERIOD_MS) {
        last_control_time = now;

        if (current_segment_index < total_segments) {
            updateExperiment();
        } else {
            setNeutral();
        }
    }
}