#ifndef SENSOR_PUBLISHER_H
#define SENSOR_PUBLISHER_H

#include "um7_parser.h"
#include <Arduino.h>

class SensorPublisher{
    public:
        void begin();

        void publishEncoder(long left_count, long right_count, unsigned long time_ms);
        void publishImu(const ImuSample& sample);

        void publishDebug(
            unsigned long time_ms,
            float v_ref, float v_meas,
            float w_ref, float w_meas_for_control,
            float imu_wz_for_control,
            float y_cmd, float x_cmd,
            float v_left, float v_right
        );
};

#endif
