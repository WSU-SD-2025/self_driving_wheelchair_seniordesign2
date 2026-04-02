#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <Arduino.h>
#include "PidController.h"
#include "CmdVelReceiver.h"
#include "WheelchairController.h"

struct DebugFlags{
    bool print_encoder = true;
    bool print_imu = true;
    bool print_debug = true;
};

struct DebugTuning{
    float x_neutral_trim = 0.0f;
    float y_neutral_trim = 0.0f;
};

class DebugConsole{
    public:
        void begin();

        bool processLine(
            const String& line,
            DebugFlags& flags,
            DebugTuning& tuning,
            PidController& pidY,
            PidController& pidW,
            PidController& pidHeading,
            PidController& pidWAssist,
            CmdVelReceiver& cmdVelReceiver,
            WheelchairController& wheelchair,
            bool& heading_hold_active,
            const float heading_hold_max_w
        );
};

#endif