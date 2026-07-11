#include "Level2System.h"


bool Level2System::check_120_conditions(ADCInterface& adc_interface)
{
    return (adc_interface.is_control_pilot_low() &&
            adc_interface.is_proximity_pilot_high() &&
            adc_interface.read_240_enabled() &&
            !adc_interface.read_240_ok());
}


bool Level2System::check_240_conditions(ADCInterface& adc_interface)
{
    return (adc_interface.read_240_ok() && adc_interface.is_jumper_out_high());
}


bool Level2System::is_120_switched(ADCInterface& adc_interface)
{
    return ((!adc_interface.read_240_ok()) && (adc_interface.is_jumper_out_high()));
}


bool Level2System::is_240_switched(ADCInterface& adc_interface)
{
    return ((!adc_interface.read_240_ok()) && (adc_interface.is_jumper_out_low()));
}


bool Level2System::check_state_B2_conditions(ADCInterface& adc_interface, Level2Interface& level2_interface)
{
    volt control_voltage = adc_interface.read_control_pilot();
    volt proximity_voltage = adc_interface.read_proximity_pilot();

    if (control_voltage < _thresholds.state_B2_control_voltage_max &&
        proximity_voltage < _thresholds.state_B2_proximity_voltage_max &&
        level2_interface._is_pwm_duty_cycle_valid())
    {
        return true;
    }

    return false;
}


bool Level2System::check_state_C2_conditions(ADCInterface& adc_interface, Level2Interface& level2_interface)
{
    volt control_voltage = adc_interface.read_control_pilot();
    volt proximity_voltage = adc_interface.read_proximity_pilot();

    if (control_voltage < _thresholds.state_C2_control_voltage_max &&
        proximity_voltage < _thresholds.state_C2_proximity_voltage_max &&
        level2_interface._is_pwm_duty_cycle_valid())
    {
        return true;
    }

    return false;
}