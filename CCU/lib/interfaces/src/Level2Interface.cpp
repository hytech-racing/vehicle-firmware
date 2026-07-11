#include "Level2Interface.h"


void Level2Interface::init()
{
    pinMode(_pinout.teensy_control_pwm_sense_pin, INPUT);
    pinMode(_pinout.teensy_start_charge_pin, OUTPUT);
}

bool Level2Interface::_is_pwm_duty_cycle_valid()
{
    unsigned long highTime = pulseIn(_pinout.teensy_control_pwm_sense_pin, HIGH, _config.pwm_pulse_in_timeout_ms);
    unsigned long lowTime  = pulseIn(_pinout.teensy_control_pwm_sense_pin, LOW, _config.pwm_pulse_in_timeout_ms);
    // Returns the length of the pulse in microseconds
    // Returns 0 if no pulse starts

    if (highTime == 0 || lowTime == 0)
    {
        // Handle 0% or 100% duty cycle (no transitions detected)
        _readings.control_pwm_duty_cycle = digitalRead(_pinout.teensy_control_pwm_sense_pin) ? 100.0 : 0.0;
        return false;
    }

    // Calculate duty cycle
    _readings.control_pwm_duty_cycle = (static_cast<float>(highTime) / static_cast<float>(highTime + lowTime)) * 100.0f;

    // Check if in valid range (9.5% to 96.5%)
    return (_readings.control_pwm_duty_cycle > _config.min_valid_pwm_duty_cycle_percent && 
            _readings.control_pwm_duty_cycle < _config.max_valid_pwm_duty_cycle_percent);
}

void Level2Interface::set_start_charge(bool state)
{
    digitalWrite(_pinout.teensy_start_charge_pin, state);
}

Level2_Data_s Level2Interface::get_level_2_data() const
{
    return _readings;
}