#ifndef CMD_VEL_RECEIVER_H
#define CMD_VEL_RECEIVER_H
#include <Arduino.h>

class CmdVelReceiver {
    private:
        String rx_buffer;
        float vRef;
        float wRef;
        unsigned long lastCmdMs;
    
    public:
        CmdVelReceiver();

        void begin();
        void update();
        bool parsePacket(const String& packet, float& vOut, float& wOut);
        float getVRef() const;
        float getWRef() const;
        unsigned long getLastCmdMs() const;
        void setZero();
};

#endif