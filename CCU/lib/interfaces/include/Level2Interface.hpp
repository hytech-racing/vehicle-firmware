#ifndef LEVEL2INTERFACE_H
#define LEVEL2INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>
#include <cstddef>
#include <cstdint>

using pin = uint8_t;


namespace default_level2_interface_params
{
    constexpr unsigned long PWM_PULSE_IN_TIMEOUT_MS = 100000UL;
    constexpr float MIN_VALID_PWM_DUTY_CYCLE_PERCENT = 9.5F;
    constexpr float MAX_VALID_PWM_DUTY_CYCLE_PERCENT = 96.5F;
}

struct Level2_Pinout_s
{
    const pin teensy_control_pwm_sense_pin;
    const pin teensy_start_charge_pin;
};

struct Level2_Data_s
{
    uint16_t control_pwm;
    float control_pwm_duty_cycle;
};

struct Level2_Config_s
{
    unsigned long pwm_pulse_in_timeout_ms;
    float min_valid_pwm_duty_cycle_percent;
    float max_valid_pwm_duty_cycle_percent;
};

class Level2Interface
{
public:
    Level2Interface(Level2_Pinout_s pinout,
                    Level2_Config_s config =
                    {
                        .pwm_pulse_in_timeout_ms = default_level2_interface_params::PWM_PULSE_IN_TIMEOUT_MS,
                        .min_valid_pwm_duty_cycle_percent = default_level2_interface_params::MIN_VALID_PWM_DUTY_CYCLE_PERCENT,
                        .max_valid_pwm_duty_cycle_percent = default_level2_interface_params::MAX_VALID_PWM_DUTY_CYCLE_PERCENT,
                    }
    ) : _pinout(pinout),
        _config(config)
    {};

    void init();

    /**
     * Function to determine pwm dutycycle
     */
    bool _is_pwm_duty_cycle_valid();

    /**
     * Setter for START_CHARGE signal
     */
    void set_start_charge(bool state);


    Level2_Data_s get_level_2_data() const;

private:

    Level2_Pinout_s _pinout;
    Level2_Data_s _readings;
    Level2_Config_s _config;

};

using Level2InterfaceInstance = etl::singleton<Level2Interface>;

#endif