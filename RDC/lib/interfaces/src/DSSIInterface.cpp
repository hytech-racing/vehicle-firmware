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
        case (OFF):
        case (STARTUP_NO_TS):
        case (STARTUP_TS_ACTIVE): {
            digitalWrite(_yellow_dssi_pin, LOW);
            digitalWrite(_blue_dssi_pin, LOW);
            break;
        }
        case (READY): {
            digitalWrite(_blue_dssi_pin, LOW);
            digitalWrite(_yellow_dssi_pin, HIGH);
            break;
        }
        case (DRIVING): {
            digitalWrite(_blue_dssi_pin, LOW);
            if (curr_millis - lastBlinkTime >= 500) {
                _yellowState = !(_yellowState);
                digitalWrite(_yellow_dssi_pin, _yellowState);
                lastBlinkTime = millis();
            }
            break;
        }
        case (EMERGENCY): {
            digitalWrite(_yellow_dssi_pin, LOW);
            if (curr_millis - lastBlinkTime >= 500) {
                _blueState = !(_blueState);
                digitalWrite(_blue_dssi_pin, _blueState);
                lastBlinkTime = millis();
            }
            break;
        }
        case (FINISHED): {
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