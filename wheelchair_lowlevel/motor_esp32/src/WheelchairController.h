#ifndef WHEELCHAIR_CONTROLLER_H
#define WHEELCHAIR_CONTROLLER_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

class WheelchairController {
    private:
        Adafruit_MCP4728 mcp;
        
        // MCP4728 pins
        int sda_pin;
        int scl_pin;

        const float dacVref = 5.0f;

        // DAC Channel settings
        uint8_t y_channel;
        uint8_t x_channel;

        // Voltage
        float x_voltage;
        float y_voltage;

        float x_neutral;
        float y_neutral;

        float x_min;
        float x_max;
        float y_min;
        float y_max;

        float max_linear_cmd;
        float max_angular_cmd;

        float x_span;
        float y_span;

        float x_deadband;
        float y_deadband;

        float clampValue(float v, float lo, float hi);
        uint16_t voltageToCode(float v);
        void writeChannelVoltage(uint8_t channel, float v);
        float applyDeadband(float u, float deadband);

    public:
        WheelchairController(int sda, int scl, uint8_t y_ch, uint8_t x_ch, float y_neutral_v, float x_neutral_v, float y_min_v, float y_max_v, float x_min_v, float x_max_v);

        bool begin();

        void setCommands(float max_linear, float max_angular);
        void setVoltageSpans(float y_span_v, float x_span_v);
        void setDeadbands(float y_deadband_v, float x_deadband_v);

        void commandToVoltage(float v_cmd, float w_cmd, float& y_out_v, float& x_out_v);
        void writeXYVoltages(float y, float x);
        void setNeutral();

        float getXVoltage() const;
        float getYVoltage() const;
};
#endif