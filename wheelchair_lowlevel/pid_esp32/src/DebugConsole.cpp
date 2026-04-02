#include "DebugConsole.h"
#include <stdio.h>

void DebugConsole::begin(){}

bool DebugConsole::processLine(
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
)
{
    if(!line.startsWith("!"))   return false;

    if(line == "!dbg all"){
        flags.print_encoder = true;
        flags.print_imu = true;
        flags.print_debug = true;
        Serial.println("ACK,dbg,all");
        return true;
    }

    if(line == "!dbg enc"){
        flags.print_encoder = true;
        flags.print_imu = false;
        flags.print_debug = false;
        Serial.println("ACK,dbg,enc");
        return true;
    }

    if(line == "!dbg imu"){
        flags.print_encoder = false;
        flags.print_imu = true;
        flags.print_debug = false;
        Serial.println("ACK,dbg,imu");
        return true;
    }

    if(line == "!dbg state"){
        flags.print_encoder = false;
        flags.print_imu = false;
        flags.print_debug = true;
        Serial.println("ACK,dbg,state");
        return true;
    }

    if(line.startsWith("!trimx ")){
        tuning.x_neutral_trim = line.substring(7).toFloat();
        Serial.print("ACK,trimx,");
        Serial.println(tuning.x_neutral_trim, 4);
        return true;
    }

    if(line.startsWith("!trimy ")){
        tuning.y_neutral_trim = line.substring(7).toFloat();
        Serial.print("ACK,trimy,");
        Serial.println(tuning.y_neutral_trim, 4);
        return true;
    }

    if(line.startsWith("!kpy ")){
        float kp, ki, kd;
        if(sscanf(line.c_str(), "!kpy %f %f %f", &kp, &ki, &kd) == 3){
            pidY.begin(kp, ki, kd, -0.20f, 0.20f);
            Serial.printf("ACK,kpy,%.4f,%.4f,%.4f\n", kp, ki, kd);
            return true;
        }
    }

    if(line.startsWith("!kpw ")){
        float kp, ki, kd;
        if(sscanf(line.c_str(), "!kpw %f %f %f", &kp, &ki, &kd) == 3){
            pidW.begin(kp, ki, kd, -0.18f, 0.18f);
            Serial.printf("ACK,kpw,%.4f,%.4f,%.4f\n", kp, ki, kd);
            return true;
        }
    }

    if(line.startsWith("!khd ")){
        float kp, ki, kd;
        if(sscanf(line.c_str(), "!khd %f %f %f", &kp, &ki, &kd) == 3){
            pidHeading.begin(kp, ki, kd, -heading_hold_max_w, heading_hold_max_w);
            Serial.printf("ACK,khd,%.4f,%.4f,%.4f\n", kp, ki, kd);
            return true;
        }
    }

    if(line.startsWith("!kwa ")){
        float kp, ki, kd;
        if(sscanf(line.c_str(), "!kwa %f %f %f", &kp, &ki, &kd) == 3){
            pidWAssist.begin(kp, ki, kd, -0.10f, 0.10f);
            Serial.printf("ACK,kwa,%.4f,%.4f,%.4f\n", kp, ki, kd);
            return true;
        }
    }

    if(line == "!stop"){
        cmdVelReceiver.setZero();
        pidY.reset();
        pidW.reset();
        pidHeading.reset();
        pidWAssist.reset();
        heading_hold_active = false;
        wheelchair.setNeutral();
        Serial.println("ACK,stop");
        return true;
    }

    Serial.print("ACK,unknown,");
    Serial.println(line);
    return true;
    
}