#ifndef LOGGER_H
#define LOGGER_H
#include <Arduino.h>

class Logger {
    private:
        Stream& out;
        bool header_printed;

    public:
        explicit Logger(Stream& output);

        void begin();
        void printHeader();

        void logRow(unsigned long time_ms, float v_ref, float w_ref, float y_voltage, float x_voltage, float vL, float vR, float vBody, float wBody);
};
#endif