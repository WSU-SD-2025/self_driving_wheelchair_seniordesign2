#ifndef CMD_VEL_RECEIVER_H
#define CMD_VEL_RECEIVER_H
#include <Arduino.h>

class CmdVelReceiver {
    private:
        float vRef;
        float wRef;
        unsigned long lastCmdMs;
    
    public:
        CmdVelReceiver();

        void begin();
        bool parsePacket(const String& packet, float& vOut, float& wOut);
        bool processLine(const String& line);
        float getVRef() const;
        float getWRef() const;
        unsigned long getLastCmdMs() const;
        void setZero();
};

#endif