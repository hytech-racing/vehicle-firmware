#include "DSSIInterface.hpp"

#include <Arduino.h>

DSSIInterface::DSSIInterface(uint8_t yellow_dssi_pin, uint8_t blue_dssi_pin) {
    _yellow_dssi_pin = yellow_dssi_pin;
    _blue_dssi_pin = blue_dssi_pin;
    pinMode(yellow_dssi_pin, OUTPUT);
    pinMode(blue_dssi_pin, OUTPUT);
}

void DSSIInterface::setDSSI(unsigned long curr_millis, DriverlessSystemState_e dv_state) {
    switch (dv_state) {
        case (DriverlessSystemState_e::OFF):
        case (DriverlessSystemState_e::STARTUP_NO_TS):
        case (DriverlessSystemState_e::STARTUP_TS_ACTIVE): {
            digitalWrite(_yellow_dssi_pin, LOW);
            digitalWrite(_blue_dssi_pin, LOW);
            break;
        }
        case (DriverlessSystemState_e::READY): {
            digitalWrite(_blue_dssi_pin, LOW);
            digitalWrite(_yellow_dssi_pin, HIGH);
            break;
        }
        case (DriverlessSystemState_e::DRIVING): {
            digitalWrite(_blue_dssi_pin, LOW);
            if (curr_millis - lastBlinkTime >= 500) {
                _yellowState = !(_yellowState);
                digitalWrite(_yellow_dssi_pin, _yellowState);
                lastBlinkTime = millis();
            }
            break;
        }
        case (DriverlessSystemState_e::EMERGENCY): {
            digitalWrite(_yellow_dssi_pin, LOW);
            if (curr_millis - lastBlinkTime >= 500) {
                _blueState = !(_blueState);
                digitalWrite(_blue_dssi_pin, _blueState);
                lastBlinkTime = millis();
            }
            break;
        }
        case (DriverlessSystemState_e::FINISHED): {
            digitalWrite(_yellow_dssi_pin, LOW);
            digitalWrite(_blue_dssi_pin, HIGH);
            break;
        }
        default: {
            digitalWrite(_yellow_dssi_pin, LOW);
            digitalWrite(_blue_dssi_pin, LOW);
            break;
        }
    }
}