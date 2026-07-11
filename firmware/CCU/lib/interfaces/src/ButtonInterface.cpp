#include "ButtonInterface.h"


void ButtonInterface::update(unsigned long current_millis)
{
    bool read = digitalRead(_pin);
    if (_active_low)
    {
        read = !read;
    }

    // Reset debounce timer if reading changed
    if (read != _button_state.last_read)
    {
        _button_state.last_debounce_time_ms = current_millis;
    }

    // Check if debounce period has elapsed
    if ((current_millis - _button_state.last_debounce_time_ms) > _debounce_ms)
    {
        // Update state if it changed
        if (read != _button_state.current_state)
        {
            _button_state.current_state = read;

            // Detect edges
            if (_button_state.current_state && !_button_state.last_stable_state)
            {
                // Rising edge - button pressed
                _button_state.press_event = true;
                _button_state.press_start_time_ms = current_millis;
            }
            else if (!_button_state.current_state && _button_state.last_stable_state)
            {
                // Falling edge - button released
                _button_state.release_event = true;
            }

            _button_state.last_stable_state = _button_state.current_state;
        }
    }

    _button_state.last_read = read;
}

bool ButtonInterface::is_pressed()
{
    if (_button_state.press_event) {
        _button_state.press_event = false;
        return true;
    }
    return false;
}

bool ButtonInterface::is_released()
{
    if (_button_state.release_event) {
        _button_state.release_event = false;
        return true;
    }
    return false;
}

bool ButtonInterface::is_held()
{
    return _button_state.current_state;
}

unsigned long ButtonInterface::get_hold_duration_ms(unsigned long current_millis)
{
    if (_button_state.current_state) {
        return current_millis - _button_state.press_start_time_ms;
    }
    return 0;
}