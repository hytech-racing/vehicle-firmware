#ifndef BUTTONINTERFACE_H
#define BUTTONINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Dependencies */
#include <Arduino.h>
#include "SharedFirmwareTypes.h"


namespace default_button_params
{
    constexpr uint8_t DEBOUNCE_MS = 100;
}

struct ButtonState_s
{
    uint32_t last_debounce_time_ms = 0;
    uint32_t press_start_time_ms = 0;

    bool last_read = false;
    bool current_state = false;
    bool last_stable_state = false; // post debounce

    bool press_event = false;
    bool release_event = false;
};

class ButtonInterface
{
public:

    ButtonInterface(size_t pin,
                    uint8_t debounce_ms = default_button_params::DEBOUNCE_MS,
                    bool active_low = true
    ) : _pin(pin),
        _debounce_ms(debounce_ms),
        _active_low(active_low)
    {
        pinMode(_pin, INPUT);
    };

    void update(unsigned long current_millis);

    bool is_pressed();

    bool is_released();

    bool is_held();

    unsigned long get_hold_duration_ms(unsigned long current_millis);

private:

    const uint8_t _pin;
    const uint8_t _debounce_ms;
    const bool _active_low;
    ButtonState_s _button_state;

};

#endif // BUTTONINTERFACE_H