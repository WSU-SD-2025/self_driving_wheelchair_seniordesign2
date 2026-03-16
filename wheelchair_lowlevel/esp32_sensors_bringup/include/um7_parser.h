#pragma once
#include <Arduino.h>

struct ImuSample {
    float qx, qy, qz, qw; // Orientation quaternion
    float wx, wy, wz;    // Angular velocity (rad/s) - Gyro
    float ax, ay, az;    // Linear acceleration (m/s^2) - Accel

    bool valid;
};

void um7_begin();
void um7_update();
bool um7_get_sample(ImuSample& out);