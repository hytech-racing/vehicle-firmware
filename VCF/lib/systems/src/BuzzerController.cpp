#ifndef TESTING_SYSTEMS

#include "BuzzerController.hpp"


void BuzzerController::init(int pin)
{
    pinMode(pin, OUTPUT);
}

void BuzzerController::activate(unsigned long curr_millis)
{
    _last_activation_time_ms = curr_millis;
}

void BuzzerController::deactivate()
{
    _last_activation_time_ms = 0;
}

bool BuzzerController::isBuzzerActive(unsigned long curr_millis)
{
    return _last_activation_time_ms != 0 && (curr_millis - _last_activation_time_ms) < _BUZZER_PERIOD_MS;
}

#endif // TESTING_SYSTEMS