#include <Arduino.h>

// GPIO
const int ENCODER_A = 4;
const int ENCODER_B = 5;

volatile long motorPosition = 0; 

void IRAM_ATTR updateMotorPosition(){
    if(digitalRead(ENCODER_B) != digitalRead(ENCODER_A)){
        // forward
        motorPosition++;
    }
    else{
        // backward
        motorPosition--;
    }
}

void setup(){
    Serial.begin(115200);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);


}

void loop(){
    static long last_count = 0;
    static unsigned long last_time = millis();

    if(millis() - last_time >= 1000){
        long current = motorPosition;
        long diff = current - last_count;

        Serial.print("Count: ");
        Serial.print(current);

        Serial.print("  Delta: ");
        Serial.println(diff);

        last_count = current;
        last_time = millis();
    }
}