#include "CmdVelReceiver.h"

CmdVelReceiver::CmdVelReceiver(): rx_buffer(""), vRef(0.0f), wRef(0.0f), lastCmdMs(0){}

void CmdVelReceiver::begin(){
    rx_buffer = "";
    vRef = 0.0f;
    wRef = 0.0f;
    lastCmdMs = millis();
}

bool CmdVelReceiver::parsePacket(const String& packet, float& vOut, float& wOut){
    int start = packet.indexOf('<');
    int comma = packet.indexOf(',');
    int end = packet.indexOf('>');

    if(start < 0 || comma < 0 || end < 0) return false;
    if(!(start < comma && comma < end)) return false;

    String vStr = packet.substring(start + 1, comma);
    String wStr = packet.substring(comma + 1, end);

    vOut = vStr.toFloat();
    wOut = wStr.toFloat();
    return true;
}

void CmdVelReceiver::update(){
    while(Serial.available() > 0){
        char c = (char)Serial.read();

        if(c == '\n'){
            float newV = 0.0f;
            float newW = 0.0f;

            if(parsePacket(rx_buffer, newV, newW)){
                vRef = newV;
                wRef = newW;
                lastCmdMs = millis();
            }

            rx_buffer = "";
        }

        else{
            rx_buffer += c;

            if(rx_buffer.length() > 64){
                rx_buffer = "";
            }
        }
    }
}

float CmdVelReceiver::getVRef() const{
    return vRef;
}

float CmdVelReceiver::getWRef() const{
    return wRef;
}

unsigned long CmdVelReceiver::getLastCmdMs() const{
    return lastCmdMs;
}

void CmdVelReceiver::setZero(){
    vRef = 0.0f;
    wRef = 0.0f;
}