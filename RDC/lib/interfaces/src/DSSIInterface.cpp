#include "DSSIInterface.hpp"

#include <Arduino.h>

DSSIInterface::DSSIInterface(uint8_t yellow_dssi_pin, uint8_t blue_dssi_pin) {
    _yellow_dssi_pin = yellow_dssi_pin;
    _blue_dssi_pin = blue_dssi_pin;
    pinMode(yellow_dssi_pin, OUTPUT);
    pinMode(blue_dssi_pin, OUTPUT);
    digitalWrite(_yellow_dssi_pin, LOW);
    digitalWrite(_blue_dssi_pin, LOW);
    lastBlinkTime = millis();
}

void DSSIInterface::setDSSI(unsigned long curr_millis, DriverlessSystemState_e dv_state) {
    if (dv_state != _previousState) {
        _previousState = dv_state;
        lastBlinkTime = curr_millis;
        _yellowState = HIGH;
        _blueState = HIGH;
    }
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
                lastBlinkTime = curr_millis;
            }
            digitalWrite(_yellow_dssi_pin, _yellowState);
            break;
        }
        case (DriverlessSystemState_e::EMERGENCY): {
            digitalWrite(_yellow_dssi_pin, LOW);
            if (curr_millis - lastBlinkTime >= 500) {
                _blueState = !(_blueState);
                lastBlinkTime = curr_millis;
            }
            digitalWrite(_blue_dssi_pin, _blueState);
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
