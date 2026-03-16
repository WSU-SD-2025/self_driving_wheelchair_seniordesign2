#include <Arduino.h>

// GPIO
const int LEFT_ENCODER_A = 4;
const int LEFT_ENCODER_B = 5;

const int RIGHT_ENCODER_A = 16;
const int RIGHT_ENCODER_B = 17;

// Direction Sign Correction
// Need real-life experiment to determine which sign is correct for each encoder
const int LEFT_SIGN = -1;
const int RIGHT_SIGN = 1;

// Encoder Counters
volatile long left_count = 0;
volatile long right_count = 0;


// Encoder snapshot structure
struct EncoderSnapshot {
    long left_current;
    long right_current;
    unsigned long time_ms;
};



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
bool readEncoderSnapshot(EncoderSnapshot& snap, unsigned long interval_ms = 50){
    static unsigned long last_time = 0;
    unsigned long now = millis();

    if(last_time == 0){
        last_time = now;
        return false;
    }

    if(now - last_time < interval_ms)
        return false;

    noInterrupts();
    snap.left_current = left_count;
    snap.right_current = right_count;
    interrupts();

    snap.time_ms = now;
    last_time = now;

    return true;
}

// CSV output for ROS2 bridge
void printEncoder(const EncoderSnapshot& snap){
    Serial.print("ENCODER,");
    Serial.print(snap.left_current);
    Serial.print(",");
    Serial.print(snap.right_current);
    Serial.print(",");
    Serial.println(snap.time_ms);
}

void setup(){
    Serial.begin(115200);
    initEncoders();
    resetEncoders();

    Serial.println("Encoders Ready");
}

void loop(){
    EncoderSnapshot snap;

    // 100 = 10Hz: for debugging.
    // 50 = 20Hz: odometry
    // 20 = 50Hz: self-driving
    if(readEncoderSnapshot(snap, 50))
        printEncoder(snap);
}