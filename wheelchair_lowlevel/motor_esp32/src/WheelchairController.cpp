#include"WheelchairController.h"

WheelchairController::WheelchairController(
    int sda, int scl, uint8_t y_ch, uint8_t x_ch,
    float y_neutral_v, float x_neutral_v, float y_min_v, float y_max_v,
    float x_min_v, float x_max_v):
sda_pin(sda), scl_pin(scl), y_channel(y_ch), x_channel(x_ch),
y_neutral(y_neutral_v), x_neutral(x_neutral_v), y_min(y_min_v), y_max(y_max_v),
x_min(x_min_v), x_max(x_max_v), max_linear_cmd(1.5f), max_angular_cmd(1.5f),
y_span(0.80f), x_span(0.80f), y_deadband(0.12f), x_deadband(0.12f) {}

float WheelchairController::clampValue(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

uint16_t WheelchairController::voltageToCode(float v){
    v = clampValue(v, 0.0f, dacVref);
    return (uint16_t)((v / dacVref) * 4095.0f);
}

void WheelchairController::writeChannelVoltage(uint8_t channel, float v){
    uint16_t code = voltageToCode(v);
    
    switch(channel){
        case MCP4728_CHANNEL_A:
            mcp.setChannelValue(MCP4728_CHANNEL_A, code);
            break;
        case MCP4728_CHANNEL_B:
            mcp.setChannelValue(MCP4728_CHANNEL_B, code);
            break;
        case MCP4728_CHANNEL_C:
            mcp.setChannelValue(MCP4728_CHANNEL_C, code);
            break;
        case MCP4728_CHANNEL_D:
            mcp.setChannelValue(MCP4728_CHANNEL_D, code);
            break;
    }
}

float WheelchairController::applyDeadband(float u, float deadband){
    if (fabs(u) < 1e-4f) return 0.0f;

    if  (u > 0.0f) return u * (1.0f - deadband) + deadband;
    return u * (1.0f - deadband) - deadband;
}

bool WheelchairController::begin(){
    Wire.begin(sda_pin, scl_pin);
    
    if(!mcp.begin())    return false;

    setNeutral();
    return true;
}

void WheelchairController::setCommands(float max_linear, float max_angular){
    max_linear_cmd = max_linear;
    max_angular_cmd = max_angular;
}

void WheelchairController::setVoltageSpans(float y_span_v, float x_span_v){
    y_span = y_span_v;
    x_span = x_span_v;
}

void WheelchairController::setDeadbands(float y_deadband_v, float x_deadband_v){
    y_deadband = y_deadband_v;
    x_deadband = x_deadband_v;
}

void WheelchairController::commandToVoltage(float v_cmd, float w_cmd, float& y_out_v, float& x_out_v){
    
    float v_norm = clampValue(v_cmd / max_linear_cmd, -1.0f, 1.0f);
    float w_norm = clampValue(w_cmd / max_angular_cmd, -1.0f, 1.0f);

    float v_eff = applyDeadband(v_norm, y_deadband);
    float w_eff = applyDeadband(w_norm, x_deadband);

    y_out_v = y_neutral + (v_eff * y_span);
    x_out_v = x_neutral + (w_eff * x_span);

    y_out_v = clampValue(y_out_v, y_min, y_max);
    x_out_v = clampValue(x_out_v, x_min, x_max);
}

void WheelchairController::writeXYVoltages(float y, float x){
    y_voltage = clampValue(y, y_min, y_max);
    x_voltage = clampValue(x, x_min, x_max);

    writeChannelVoltage(y_channel, y_voltage);
    writeChannelVoltage(x_channel, x_voltage);
}

void WheelchairController::setNeutral(){
    writeXYVoltages(y_neutral, x_neutral);
}

float WheelchairController::getXVoltage() const{
    return x_voltage;
}

float WheelchairController::getYVoltage() const{
    return y_voltage;
}