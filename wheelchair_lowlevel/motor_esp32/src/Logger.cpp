#include "Logger.h"

Logger::Logger(Stream& output):out(output), header_printed(false){}

void Logger::begin(){
    printHeader();
}

void Logger::printHeader(){
    if(header_printed) return;

    out.println("time_ms, v_ref, w_ref, y_voltage, x_voltage, vL, vR, vBody, wBody");
    header_printed = true;
}

void Logger::logRow(unsigned long time_ms, float v_ref, float w_ref, float y_voltage, float x_voltage, float vL, float vR, float vBody, float wBody){
    out.print(time_ms);
    out.print(", ");
    out.print(v_ref, 3);
    out.print(", ");
    out.print(w_ref, 3);
    out.print(", ");
    out.print(y_voltage, 3);
    out.print(", ");
    out.print(x_voltage, 3);
    out.print(", ");
    out.print(vL, 4);
    out.print(", ");
    out.print(vR, 4);
    out.print(", ");
    out.print(vBody, 4);
    out.print(", ");
    out.println(wBody, 4);
}