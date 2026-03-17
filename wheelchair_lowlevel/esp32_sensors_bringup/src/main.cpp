#include <Arduino.h>
#include "encoder_reader.h"
#include "um7_parser.h"

void setup(){
    Serial.begin(115200);
    delay(500);

    initEncoders();
    resetEncoders();

    um7_begin();

    Serial.println("SENSORS_READY");
}

void loop(){
    // Always update UM7 IMU parser
    um7_update();

    // Publish encoder snapshot at 20Hz
    EncoderSnapshot encoder_snapshot;

    if(readEncoderSnapshot(encoder_snapshot, 50))
        printEncoder(encoder_snapshot);

        

    // Publish IMU data whenever a fresh data is ready
    ImuSample imu;
    if(um7_get_sample(imu)){
        Serial.print("IMU,");
        Serial.print(imu.qx, 6); Serial.print(",");
        Serial.print(imu.qy, 6); Serial.print(",");
        Serial.print(imu.qz, 6); Serial.print(",");
        Serial.print(imu.qw, 6); Serial.print(",");

        Serial.print(imu.wx, 6); Serial.print(",");
        Serial.print(imu.wy, 6); Serial.print(",");
        Serial.print(imu.wz, 6); Serial.print(",");

        Serial.print(imu.ax, 6); Serial.print(",");
        Serial.print(imu.ay, 6); Serial.print(",");
        Serial.println(imu.az, 6);
    }
}