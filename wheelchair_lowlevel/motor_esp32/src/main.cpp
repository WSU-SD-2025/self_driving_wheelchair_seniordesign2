#include<Arduino.h>
#include "drive_controller.h"

//GPIO for motor control
// X => left and right        y => forward and backward
//Should use ADC Pins
static const int PIN_X = 15;
static const int PIN_Y = 16;

float normalizeAxis(float Voltage, float center_V, float min_V, float max_V, float deadzone);

void setup(){
    Serial.begin(115200);
    delay(300);

    analogReadResolution(12); // 12-bit resolution for ADC (0-4095)
}

void loop(){
    int rawX = analogRead(PIN_X);
    int rawY = analogRead(PIN_Y);

    // ESP32 (3.3V) So should be 3.3f later. need to check first
    float x_voltage = rawX * (5.0f / 4095.0f); // Convert raw ADC value to voltage 
    float y_voltage = rawY * (5.0f / 4095.0f); // Convert raw ADC value to voltage 

    float x_normalized = normalizeAxis(x_voltage, 2.50f, 0.81f, 4.18f, 0.05f);
    float y_normalized = normalizeAxis(y_voltage, 2.50f, 1.17f, 4.23f, 0.05f);

    //Print the raw values for x and y
    Serial.print("Raw X: ");
    Serial.print(rawX);
    Serial.print(" | Raw Y: ");
    Serial.println(rawY);

    //Print the normalized values for x and y
    Serial.print("Normalized X: ");
    Serial.print(x_normalized, 4);
    Serial.print(" | Normalized Y: ");
    Serial.println(y_normalized, 4);

    delay(200);
}