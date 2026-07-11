#include "WatchdogInterface.h"


void WatchdogInterface::init()
{
    // Pin Congfiguration
    pinMode(_watchdog_parameters.pinout.teensy_watchdog_pin, OUTPUT);
    pinMode(_watchdog_parameters.pinout.teensy_sw_shdn_pin, OUTPUT);

    // Inital Pin States For OUTPUT Pins
    digitalWrite(_watchdog_parameters.pinout.teensy_watchdog_pin, LOW);
    digitalWrite(_watchdog_parameters.pinout.teensy_sw_shdn_pin, LOW);
}

bool WatchdogInterface::update_watchdog_state(uint32_t curr_millis)
{
    if ((curr_millis - _watchdog_time) > _watchdog_parameters.watchdog_kick_interval_ms)
    {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
        digitalWrite(_watchdog_parameters.pinout.teensy_watchdog_pin, _watchdog_state);
    }

    return _watchdog_state;
}

void WatchdogInterface::set_sw_shdn_pin_low()
{
    digitalWrite(_watchdog_parameters.pinout.teensy_sw_shdn_pin, LOW);
}

void WatchdogInterface::set_sw_shdn_pin_high()
{
    digitalWrite(_watchdog_parameters.pinout.teensy_sw_shdn_pin, HIGH);
}