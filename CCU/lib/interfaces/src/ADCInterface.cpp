#include "ADCInterface.hpp"


void ADCInterface::init(uint32_t init_millis)
{
    // Pin Configuration
    pinMode(_adc_parameters.pinout.teensy_shdn_A_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_B_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_C_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_D_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_E_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_F_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_shdn_G_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_scaled_24V_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_control_pilot_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_proximity_pilot_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_240_enabled_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_240_ok_pin, INPUT);
    pinMode(_adc_parameters.pinout.teensy_jumper_out_pin, INPUT);

    _init_millis = init_millis;
}

bool ADCInterface::isShutdownAHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_A_pin);
    return out;
}

bool ADCInterface::isShutdownBHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_B_pin);
    return out;
}

bool ADCInterface::isShutdownCHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_C_pin);
    return out;
}

bool ADCInterface::isShutdownDHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_D_pin);
    return out;
}

bool ADCInterface::isShutdownEHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_E_pin);
    return out;
}

bool ADCInterface::isShutdownFHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_F_pin);
    return out;
}

bool ADCInterface::isShutdownGHigh()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_shdn_G_pin);
    return out;
}

volt ADCInterface::readGLV()
{
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_scaled_24V_pin)) * _adc_parameters.conversions.glv_conv_factor; // input before voltage divider (4.3k / (4.3k + 36k))
    return data;
}

volt ADCInterface::readControlPilot()
{
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_control_pilot_pin)) * _adc_parameters.conversions.control_pilot_conv_factor;
    return data;
}

bool ADCInterface::isControlPilotLow()
{
    return readControlPilot() < adc_default_parameters::CONTROL_PILOT_VOLTAGE_LOW_THRESHOLD;
}

volt ADCInterface::readProximityPilot()
{
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_proximity_pilot_pin)) * _adc_parameters.conversions.proximity_pilot_conv_factor;
    return data;
}

bool ADCInterface::isProximityHigh()
{
    return readProximityPilot() > adc_default_parameters::PROXIMITY_PILOT_VOLTAGE_HIGH_THRESHOLD;
}

bool ADCInterface::read240Enabled()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_240_enabled_pin);
    return out;
}

bool ADCInterface::read240OK()
{
    bool out = digitalRead(_adc_parameters.pinout.teensy_240_ok_pin);
    return out;
}

volt ADCInterface::readJumperOut()
{
    volt data = static_cast<float>(analogRead(_adc_parameters.pinout.teensy_jumper_out_pin)) * _adc_parameters.conversions.jumper_out_conv_factor;
    return data;
}

bool ADCInterface::isJumperOutHigh()
{
    return readJumperOut() > adc_default_parameters::TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH;
}

bool ADCInterface::isJumperOutLow()
{
    return readJumperOut() < adc_default_parameters::TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH;
}

bool ADCInterface::isResetErrorsButtonPressed(unsigned long current_millis)
{
    _reset_error_button.update(current_millis);
    return _reset_error_button.isPressed();
}

const ADCInterfaceParams_s& ADCInterface::getADCParams() const
{
    return _adc_parameters;
}