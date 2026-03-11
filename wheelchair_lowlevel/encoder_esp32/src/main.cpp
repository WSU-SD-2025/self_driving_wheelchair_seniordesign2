#include <Arduino.h>

// GPIO
const int LEFT_ENCODER_A = 4;
const int LEFT_ENCODER_B = 5;

const int RIGHT_ENCODER_A = 16;
const int RIGHT_ENCODER_B = 17;

// Direction Sign Correction
// Need real-life experiment to determine which sign is correct for each encoder
const int LEFT_SIGN = 1;
const int RIGHT_SIGN = -1;

// Encoder Counters
volatile long left_count = 0;
volatile long right_count = 0;


// Left Encoder Interrupt Service Routine
void IRAM_ATTR updateLeftEncoder(){
    if(digitalRead(LEFT_ENCODER_B) != digitalRead(LEFT_ENCODER_A))
        left_count += LEFT_SIGN; // Forward
    else
        left_count -= LEFT_SIGN; // Backward
}

// Right Encoder Interrupt Service Routine
void IRAM_ATTR updateRightEncoder(){
    if(digitalRead(RIGHT_ENCODER_B) != digitalRead(RIGHT_ENCODER_A))
        right_count += RIGHT_SIGN;
    else
        right_count -= RIGHT_SIGN;
}

// Encoder snapshot structure
struct EncoderSnapshot {
    long left_current;
    long left_delta;
    long right_current;
    long right_delta;
    float dt_sec;
};

// Initialize encoders
void initEncoders(){
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);
}

//Reset Encoder Counters
// Useful after boot noise or before starting measurement
void resetEncoders(){
    noInterrupts();
    left_count = 0;
    right_count = 0;
    interrupts();
}

// Read encoder values every interval ms
// Returns true only when new data is ready
bool readEncoderSnapshot(EncoderSnapshot& snap, unsigned long interval_ms = 100){
    static long last_left_count = 0;
    static long last_right_count = 0;
    static unsigned long last_time = 0;

    unsigned long now = millis();

    if(last_time == 0){
        last_time = now;
        return false;
    }

    if(now - last_time < interval_ms)
        return false;

    noInterrupts();
    long current_left = left_count;
    long current_right = right_count;
    interrupts();

    snap.left_current = current_left;
    snap.right_current = current_right;
    snap.left_delta = current_left - last_left_count;
    snap.right_delta = current_right - last_right_count;

    snap.dt_sec = (now - last_time) / 1000.0f;

    last_left_count = current_left;
    last_right_count = current_right;
    last_time = now;

    return true;
}

// Debug
void printEncoder(const EncoderSnapshot& snap){
    Serial.print("L Count: ");
    Serial.print(snap.left_current);
    Serial.print(" L Delta: ");
    Serial.print(snap.left_delta);

    Serial.print("  |   R Count: ");
    Serial.print(snap.right_current);
    Serial.print(" R Delta: ");
    Serial.print(snap.right_delta);

    Serial.print("  |   dt: ");
    Serial.println(snap.dt_sec, 3);
}

void setup(){
    Serial.begin(115200);
    initEncoders();

    resetEncoders();

    Serial.println("Dual encoder test started");
}

void loop(){
    EncoderSnapshot snap;

    // 100 = 10Hz: for debugging.
    // 50 = 20Hz: odometry
    // 20 = 50Hz: self-driving
    if(readEncoderSnapshot(snap, 100))
        printEncoder(snap);
}