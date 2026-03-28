#include "LookupTable.h"

namespace {
    constexpr int LUT_SIZE = 21;

    // Y-axis lookup: ref_v -> y_voltage
    static const float v_bp[LUT_SIZE] = {
        -1.0f, -0.9f, -0.8f, -0.7f, -0.6f, -0.5f, -0.4f, -0.3f, -0.2f, -0.1f,
         0.0f,
         0.1f,  0.2f,  0.3f,  0.4f,  0.5f,  0.6f,  0.7f,  0.8f,  0.9f, 1.0f
    };

    static const float y_table[LUT_SIZE] = {
        2.124f, 2.171f, 2.218f, 2.264f, 2.311f, 2.358f, 2.405f, 2.452f, 2.499f,
        2.546f,
        2.689f,
        2.832f, 2.879f, 2.926f, 2.973f, 3.020f, 3.067f, 3.114f, 3.160f, 3.207f, 3.254f
    };

    // X-axis lookup: ref_w -> x_voltage
    static const float w_bp[LUT_SIZE] = {
        -1.0f, -0.9f, -0.8f, -0.7f, -0.6f, -0.5f, -0.4f, -0.3f, -0.2f, -0.1f,
         0.0f,
         0.1f,  0.2f,  0.3f,  0.4f,  0.5f,  0.6f,  0.7f,  0.8f,  0.9f, 1.0f
    };

    static const float x_table[LUT_SIZE] = {
        2.125f, 2.172f, 2.219f, 2.265f, 2.312f, 2.359f, 2.406f, 2.453f, 2.500f, 2.547f,
        2.690f,
        2.833f, 2.880f, 2.927f, 2.974f, 3.021f, 3.068f, 3.115f, 3.161f, 3.208f, 3.255f
    };


    float interp1(const float* bp, const float* table, int size, float x){
        if(x <= bp[0]) return table[0];
        if(x >= bp[size-1]) return table[size-1];

        for(int i=0; i < size - 1; ++i){
            if(x >= bp[i] && x <= bp[i+1]){
                float t = (x - bp[i]) / (bp[i+1] - bp[i]);
                return table[i] + t * (table[i+1] - table[i]);
            }
        }
        return table[size-1];
    }
}



float lookupYVoltage(float v_ref){
    return interp1(v_bp, y_table, LUT_SIZE, v_ref);
}

float lookupXVoltage(float w_ref){
    return interp1(w_bp, x_table, LUT_SIZE, w_ref);
}