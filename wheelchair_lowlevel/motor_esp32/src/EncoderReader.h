#ifndef ENCODER_READER_H
#define ENCODER_READER_H
#include <Arduino.h>

struct EncoderSnapshot {
    long left_current;
    long right_current;
    unsigned long time_ms;
};

class EncoderReader {
    private:
        int left_A;
        int left_B;
        int right_A;
        int right_B;

        int left_sign;
        int right_sign;

        volatile long left_count;
        volatile long right_count;

        float left_cpr;
        float right_cpr;
        float wheel_radius;
        float wheel_separation;

        unsigned long last_snapshot_time;

        float vL;
        float vR;
        float vBody;
        float wBody;


    public:
        EncoderReader(int left_A, int left_B, int right_A, int right_B, int left_sign, int right_sign, float left_cpr, float right_cpr, float wheel_radius, float wheel_separation);

        void begin();
        void reset();

        void handleLeftISR();
        void handleRightISR();

        bool readSnapshot(EncoderSnapshot& snap, unsigned long interval_ms = 50);
        void updateVelocitiesFromSnapshot(const EncoderSnapshot& prev_snap, const EncoderSnapshot& curr_snap);

        long getLeftCount() const;
        long getRightCount() const;

        float getVL() const;
        float getVR() const;
        float getVBody() const;
        float getWBody() const;
};

#endif