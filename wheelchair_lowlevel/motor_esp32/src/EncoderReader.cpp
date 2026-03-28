#include "EncoderReader.h"

EncoderReader::EncoderReader(
    int left_A, int left_B, int right_A, int right_B, int left_sign, int right_sign,
    float left_cpr, float right_cpr, float wheel_radius, float wheel_separation):

    left_A(left_A), left_B(left_B), right_A(right_A), right_B(right_B), left_sign(left_sign), right_sign(right_sign),
    left_count(0), right_count(0), left_cpr(left_cpr), right_cpr(right_cpr), wheel_radius(wheel_radius),
    wheel_separation(wheel_separation), last_snapshot_time(0),vL(0.0f), vR(0.0f), vBody(0.0f), wBody(0.0f) {}


void EncoderReader::begin(){
    pinMode(left_A, INPUT_PULLUP);
    pinMode(left_B, INPUT_PULLUP);
    pinMode(right_A, INPUT_PULLUP);
    pinMode(right_B, INPUT_PULLUP);

    last_snapshot_time = 0;
    reset();
}

void EncoderReader::reset(){
    noInterrupts();
    left_count = 0;
    right_count = 0;
    interrupts();

    vL = 0.0f;
    vR = 0.0f;
    vBody = 0.0f;
    wBody = 0.0f;
}

void IRAM_ATTR EncoderReader::handleLeftISR(){
    if(digitalRead(left_B) != digitalRead(left_A))
        left_count += left_sign;
    else
        left_count -= left_sign;
}

void IRAM_ATTR EncoderReader::handleRightISR(){
    if(digitalRead(right_B) != digitalRead(right_A))
        right_count += right_sign;
    else
        right_count -= right_sign;
}

bool EncoderReader::readSnapshot(EncoderSnapshot& snap, unsigned long interval_ms){
    unsigned long now = millis();

    if(last_snapshot_time == 0){
        last_snapshot_time = now;
        return false;
    }

    if(now - last_snapshot_time < interval_ms)
        return false;

    noInterrupts();
    snap.left_current = left_count;
    snap.right_current = right_count;
    interrupts();

    snap.time_ms = now;
    last_snapshot_time = now;
    return true;
}

void EncoderReader::updateVelocitiesFromSnapshot(const EncoderSnapshot& prev_snap, const EncoderSnapshot& curr_snap){
    
    long dL = curr_snap.left_current - prev_snap.left_current;
    long dR = curr_snap.right_current - prev_snap.right_current;

    unsigned long dt_ms = curr_snap.time_ms - prev_snap.time_ms;
    if(dt_ms == 0) return;

    float dt = dt_ms / 1000.0f;
    float wheelchair_circumference = 2.0f * PI * wheel_radius;

    float left_revs = (float) dL / left_cpr;
    float right_revs = (float) dR / right_cpr;

    vL = (left_revs * wheelchair_circumference) / dt;
    vR = (right_revs * wheelchair_circumference) / dt;

    vBody = (vL + vR) / 2.0f;
    wBody = (vR - vL) / wheel_separation;
}

long EncoderReader::getLeftCount() const{
    return left_count;
}

long EncoderReader::getRightCount() const{
    return right_count;
}

float EncoderReader::getVL() const{
    return vL;
}

float EncoderReader::getVR() const{
    return vR;
}

float EncoderReader::getVBody() const{
    return vBody;
}

float EncoderReader::getWBody() const{
    return wBody;
}