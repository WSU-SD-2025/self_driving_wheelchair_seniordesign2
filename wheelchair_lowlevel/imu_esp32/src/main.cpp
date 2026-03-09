#include <Arduino.h>
#include "um7_parser.h"

void setup() {
    Serial.begin(115200);
    delay(500);
    um7_begin();
}

void loop(){
    um7_update();

    ImuSample imu;

    if(um7_get_sample(imu)){
      Serial.print(imu.qx, 6); Serial.print(", ");
      Serial.print(imu.qy, 6); Serial.print(", ");
      Serial.print(imu.qz, 6); Serial.print(", ");
      Serial.print(imu.qw, 6); Serial.print(", ");

      Serial.print(imu.wx, 6); Serial.print(", ");
      Serial.print(imu.wy, 6); Serial.print(", ");
      Serial.print(imu.wz, 6); Serial.print(", ");

      Serial.print(imu.ax, 6); Serial.print(", ");
      Serial.print(imu.ay, 6); Serial.print(", ");
      Serial.println(imu.az, 6);
    }
}