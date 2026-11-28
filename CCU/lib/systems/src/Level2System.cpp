#include "Level2System.hpp"


bool Level2System::check120Conditions(ADCInterface& adc_interface)
{
    return (adc_interface.isControlPilotLow() && adc_interface.isProximityHigh() && adc_interface.read240Enabled() && !adc_interface.read240OK());
}


bool Level2System::check240Conditions(ADCInterface& adc_interface)
{
    return (adc_interface.read240OK() && adc_interface.isJumperOutHigh());
}


bool Level2System::is120Switched(ADCInterface& adc_interface)
{
    return ((!adc_interface.read240OK()) && (adc_interface.isJumperOutHigh()));
}


bool Level2System::is240Switched(ADCInterface& adc_interface)
{
    return ((!adc_interface.read240OK()) && (adc_interface.isJumperOutLow()));
}


bool Level2System::checkStateB2Conditions(ADCInterface& adc_interface, Level2Interface& level2_interface)
{
    Volt control_voltage = adc_interface.readControlPilot();
    Volt proximity_voltage = adc_interface.readProximityPilot();

    if (control_voltage < _thresholds.state_b2_control_voltage_max
        && proximity_voltage < _thresholds.state_b2_proximity_voltage_max
        && level2_interface.isPWMDutyCycleValid())
    {
        return true;
    }

    return false;
}


bool Level2System::checkStateC2Conditions(ADCInterface& adc_interface, Level2Interface& level2_interface)
{
    Volt control_voltage = adc_interface.readControlPilot();
    Volt proximity_voltage = adc_interface.readProximityPilot();

    if (control_voltage < _thresholds.state_c2_control_voltage_max
        && proximity_voltage < _thresholds.state_c2_proximity_voltage_max
        && level2_interface.isPWMDutyCycleValid())
    {
        return true;
    }

    return false;
}
