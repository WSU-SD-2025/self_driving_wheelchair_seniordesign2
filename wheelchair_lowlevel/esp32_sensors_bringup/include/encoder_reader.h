#pragma once
#include <Arduino.h>

struct EncoderSnapshot {
    long left_current;
    long right_current;
    unsigned long time_ms;
};

void initEncoders();
void resetEncoders();
bool readEncoderSnapshot(EncoderSnapshot& snap, unsigned long interval_ms = 50);
void printEncoder(const EncoderSnapshot& snap);