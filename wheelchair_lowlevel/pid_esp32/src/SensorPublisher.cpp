#include "SensorPublisher.h"

void SensorPublisher::begin(){}

void SensorPublisher::publishEncoder(long left_count, long right_count, unsigned long time_ms){
    Serial.print("ENCODER,");
    Serial.print(left_count);
    Serial.print(",");
    Serial.print(right_count);
    Serial.print(",");
    Serial.println(time_ms);
}

void SensorPublisher::publishImu(const ImuSample& imu){
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

void SensorPublisher::publishDebug(
    unsigned long time_ms,
    float v_ref, float v_meas,
    float w_ref, float w_meas_for_control,
    float imu_wz_for_control,
    float y_cmd, float x_cmd,
    float v_left, float v_right
){
    Serial.print("DEBUG,");
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(v_ref, 4);
    Serial.print(",");
    Serial.print(v_meas, 4);
    Serial.print(",");
    Serial.print(w_ref, 4);
    Serial.print(",");
    Serial.print(w_meas_for_control, 4);
    Serial.print(",");
    Serial.print(imu_wz_for_control, 4);
    Serial.print(",");
    Serial.print(y_cmd, 4);
    Serial.print(",");
    Serial.print(x_cmd, 4);
    Serial.print(",");
    Serial.print(v_left, 4); Serial.print(",");
    Serial.println(v_right, 4);
}