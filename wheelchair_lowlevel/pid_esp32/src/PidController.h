#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>

class PidController {
    private:
        float kp_, ki_, kd_;
        float out_min_, out_max_;
        float integral_;
        float prev_error_;
        bool initialized_;

    public:
        void begin(float kp, float ki, float kd, float out_min, float out_max);
        void reset();
        float update(float target, float measured, float dt);
};

#endif