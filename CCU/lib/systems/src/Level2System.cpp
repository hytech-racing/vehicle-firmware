#include "Level2System.hpp"


bool Level2System::check_120_conditions(ADCInterface& adc_interface)
{
    return (adc_interface.isControlPilotLow() &&
            adc_interface.isProximityHigh() &&
            adc_interface.read240Enabled() &&
            !adc_interface.read240OK());
}


bool Level2System::check_240_conditions(ADCInterface& adc_interface)
{
    return (adc_interface.read240OK() && adc_interface.isJumperOutHigh());
}


bool Level2System::is_120_switched(ADCInterface& adc_interface)
{
    return ((!adc_interface.read240OK()) && (adc_interface.isJumperOutHigh()));
}


bool Level2System::is_240_switched(ADCInterface& adc_interface)
{
    return ((!adc_interface.read240OK()) && (adc_interface.isJumperOutLow()));
}


bool Level2System::check_state_B2_conditions(ADCInterface& adc_interface, Level2Interface& level2_interface)
{
    volt control_voltage = adc_interface.readControlPilot();
    volt proximity_voltage = adc_interface.readProximityPilot();

    if (control_voltage < _thresholds.state_B2_control_voltage_max &&
        proximity_voltage < _thresholds.state_B2_proximity_voltage_max &&
        level2_interface._isPWMDutyCycleValid())
    {
        return true;
    }

    return false;
}


bool Level2System::check_state_C2_conditions(ADCInterface& adc_interface, Level2Interface& level2_interface)
{
    volt control_voltage = adc_interface.readControlPilot();
    volt proximity_voltage = adc_interface.readProximityPilot();

    if (control_voltage < _thresholds.state_C2_control_voltage_max &&
        proximity_voltage < _thresholds.state_C2_proximity_voltage_max &&
        level2_interface._isPWMDutyCycleValid())
    {
        return true;
    }

    return false;
}