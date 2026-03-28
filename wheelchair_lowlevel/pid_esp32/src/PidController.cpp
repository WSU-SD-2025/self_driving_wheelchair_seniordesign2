#include "PidController.h"
#include <math.h>

void PidController::begin(float kp, float ki, float kd, float out_min, float out_max) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    out_min_ = out_min;
    out_max_ = out_max;

    integral_ = 0.0f;
    prev_error_ = 0.0f;
    initialized_ = false;
}

void PidController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    initialized_ = false;
}

float PidController::update(float target, float measured, float dt){
    if(dt <= 0.0f){
        return 0.0f;
    }

    float error = target - measured;

    if(!initialized_){
        prev_error_ = error;
        initialized_ = true;
    }

    float derivative = (error - prev_error_) / dt;
    float integral_candidate = integral_ + error * dt;

    float output_unsat = kp_ * error + ki_ * integral_candidate + kd_ * derivative;

    float output = output_unsat;

    if(output > out_max_)
        output = out_max_;

    else if(output < out_min_)
        output = out_min_;
    else
        integral_ = integral_candidate;

    prev_error_ = error;
    return output;
}