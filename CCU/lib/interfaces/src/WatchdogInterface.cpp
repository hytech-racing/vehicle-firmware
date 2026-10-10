#include "WatchdogInterface.hpp"


void WatchdogInterface::init()
{
    // Pin Congfiguration
    pinMode(_watchdog_parameters.pinout.teensy_watchdog_pin, OUTPUT);
    pinMode(_watchdog_parameters.pinout.teensy_sw_shdn_pin, OUTPUT);

    // Inital Pin States For OUTPUT Pins
    digitalWrite(_watchdog_parameters.pinout.teensy_watchdog_pin, LOW);
    digitalWrite(_watchdog_parameters.pinout.teensy_sw_shdn_pin, LOW);
}

bool WatchdogInterface::updateWatchdogState(uint32_t curr_millis)
{
    if ((curr_millis - _watchdog_time) > _watchdog_parameters.watchdog_kick_interval_ms)
    {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
        digitalWrite(_watchdog_parameters.pinout.teensy_watchdog_pin, _watchdog_state);
    }

    return _watchdog_state;
}

void WatchdogInterface::setSWShutdownPinLow()
{
    digitalWrite(_watchdog_parameters.pinout.teensy_sw_shdn_pin, LOW);
}

void WatchdogInterface::setSWShutdownPinHigh()
{
    digitalWrite(_watchdog_parameters.pinout.teensy_sw_shdn_pin, HIGH);
}