#pragma once
#include <cmath>

inline float normalizeAxis(float Voltage, float center_V, float min_V, float max_V, float deadzone){

    float offset = Voltage - center_V;

    //Dead zone check
    // If the absolute value of the offset is less than the deadzone, return 0.0f
    if (fabsf(offset) < deadzone)
        return 0.0f;

    // Normalize the offset to the range of -1.0f to 1.0f
    float span;

    if (offset > 0)
        span = max_V - center_V;

    else
        span = center_V - min_V;
    
    // Avoid division by zero
    if (span <= 0.0f)
        return 0.0f;

    float output = offset / span;

    // Clamp
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;

    return output;
}