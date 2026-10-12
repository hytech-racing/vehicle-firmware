/* This script enables all MCU-enabled load switches and bucks unconditionally */

#include <Arduino.h>
#include "Pins.h"

const uint32_t BUCK_ENABLES[] = {
    PDBPins::EN_12V_DTI_MCU,
    PDBPins::LIDAR_EN_MCU,
};

const uint32_t LOAD_SWITCH_ENABLES[] = {
    PDBPins::EN_CAMERAS_MCU,    PDBPins::EN_ORIN_MCU,  PDBPins::EN_INV_COOL_MCU,
    PDBPins::EN_MOTOR_COOL_MCU, PDBPins::EN_LIDAR_MCU, PDBPins::EN_DTI_MCU,
};

const uint32_t LOAD_SWITCH_FLT[] = {
    PDBPins::FLT_CAMERAS_MCU, PDBPins::FLT_ORIN_MCU, PDBPins::FLT_INV_COOL_MCU,
    PDBPins::FLT_MOTOR_COOL_MCU, PDBPins::FLT_LIDAR_MCU, PDBPins::FLT_DTI_MCU,
};

void setup()
{
    // Configure as outputs, starting low
    for (uint32_t pin : BUCK_ENABLES)        { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
    for (uint32_t pin : LOAD_SWITCH_ENABLES) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
    for (uint32_t pin : LOAD_SWITCH_FLT) { pinMode(pin, INPUT); }

    // Bucks first, then load switches once the rails have settled
    for (uint32_t pin : BUCK_ENABLES) { digitalWrite(pin, HIGH); }
    delay(10);
    for (uint32_t pin : LOAD_SWITCH_ENABLES) { digitalWrite(pin, HIGH); }
    // digitalWrite(PDBPins::EN_MOTOR_COOL_MCU, HIGH);
    // digitalWrite(PDBPins::EN_CAMERAS_MCU, HIGH);


}

void loop()
{
    int fault = digitalRead(PDBPins::FLT_LIDAR_MCU);
    int en    = digitalRead(PDBPins::EN_LIDAR_MCU);

    Serial.print("EN=");
    Serial.print(en);
    Serial.print("  FLT=");
    Serial.print(fault);
    Serial.println(fault == LOW ? "  <-- FAULT" : "  ok");

    delay(500);
}