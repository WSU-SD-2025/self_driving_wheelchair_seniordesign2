#include "LookupTable.h"

namespace {
    constexpr int LUT_SIZE = 31;

    // Y-axis lookup: ref_v -> y_voltage
    static const float v_bp[LUT_SIZE] = {
        -1.0000f, -0.9000f, -0.8000f, -0.7000f, -0.6000f, -0.5000f, -0.4000f, -0.3000f,
        -0.2000f, -0.1500f, -0.1000f, -0.0800f, -0.0600f, -0.0400f, -0.0200f,
        0.0000f,
        0.0200f,  0.0400f,  0.0600f,  0.0800f,  0.1000f,  0.1500f,  0.2000f,  0.3000f,
        0.4000f,  0.5000f,  0.6000f,  0.7000f,  0.8000f,  0.9000f,  1.0000f
    };

    static const float y_table[LUT_SIZE] = {
        1.7890f, 1.8680f, 1.9470f, 2.0270f, 2.1060f, 2.1850f, 2.2640f, 2.3430f,
        2.4230f, 2.4620f, 2.5020f, 2.5180f, 2.5330f, 2.5490f, 2.5650f,
        2.6890f,
        2.8130f, 2.8290f, 2.8450f, 2.8600f, 2.8760f, 2.9160f, 2.9550f, 3.0350f,
        3.1140f, 3.1930f, 3.2720f, 3.3510f, 3.4310f, 3.5100f, 3.5890f
    };

    // X-axis lookup: ref_w -> x_voltage
    static const float w_bp[LUT_SIZE] = {
        -1.0000f, -0.9000f, -0.8000f, -0.7000f, -0.6000f, -0.5000f, -0.4000f, -0.3000f,
        -0.2000f, -0.1500f, -0.1000f, -0.0800f, -0.0600f, -0.0400f, -0.0200f,
        0.0000f,
        0.0200f,  0.0400f,  0.0600f,  0.0800f,  0.1000f,  0.1500f,  0.2000f,  0.3000f,
        0.4000f,  0.5000f,  0.6000f,  0.7000f,  0.8000f,  0.9000f,  1.0000f
    };

    static const float x_table[LUT_SIZE] = {
        1.9950f, 2.0570f, 2.1180f, 2.1800f, 2.2420f, 2.3030f, 2.3650f, 2.4260f,
        2.4880f, 2.5190f, 2.5490f, 2.5620f, 2.5740f, 2.5860f, 2.5990f,
        2.6950f,
        2.7910f, 2.8040f, 2.8160f, 2.8280f, 2.8410f, 2.8720f, 2.9020f, 2.9640f,
        3.0260f, 3.0870f, 3.1490f, 3.2100f, 3.2720f, 3.3340f, 3.3950f
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