#include "ADCInterface.h"


void ADCInterface::init()
{
    pinMode(_adc_parameters.pinouts.brake_high_sense_pin, INPUT);
    pinMode(_adc_parameters.pinouts.current_high_sense_pin, INPUT);
}

void ADCInterface::tick_adc0() { _adc0.tick(); }

void ADCInterface::tick_adc1() { _adc1.tick(); }

const ADCInterfaceParams_s& ADCInterface::get_adc_params() const
{
    return _adc_parameters;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc0_scales()
{
    std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};

    scales.at(_adc_parameters.channels.glv_sense_channel)          = _adc_parameters.scales.glv_sense_scale;
    scales.at(_adc_parameters.channels.current_sense_channel)      = _adc_parameters.scales.current_sense_scale;
    scales.at(_adc_parameters.channels.reference_sense_channel)    = _adc_parameters.scales.reference_sense_scale;
    scales.at(_adc_parameters.channels.RL_load_cell_channel)        = _adc_parameters.scales.RL_load_cell_scale;
    scales.at(_adc_parameters.channels.RR_load_cell_channel)        = _adc_parameters.scales.RR_load_cell_scale;
    scales.at(_adc_parameters.channels.RL_sus_pot_channel)          = _adc_parameters.scales.RL_sus_pot_scale;
    scales.at(_adc_parameters.channels.RR_sus_pot_channel)          = _adc_parameters.scales.RR_sus_pot_scale;

    return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc0_offsets()
{
    std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

    offsets.at(_adc_parameters.channels.glv_sense_channel)          = _adc_parameters.offsets.glv_sense_offset;
    offsets.at(_adc_parameters.channels.current_sense_channel)      = _adc_parameters.offsets.current_sense_offset;
    offsets.at(_adc_parameters.channels.reference_sense_channel)    = _adc_parameters.offsets.reference_sense_offset;
    offsets.at(_adc_parameters.channels.RL_load_cell_channel)        = _adc_parameters.offsets.RL_load_cell_offset;
    offsets.at(_adc_parameters.channels.RR_load_cell_channel)        = _adc_parameters.offsets.RR_load_cell_offset;
    offsets.at(_adc_parameters.channels.RL_sus_pot_channel)          = _adc_parameters.offsets.RL_sus_pot_offset;
    offsets.at(_adc_parameters.channels.RR_sus_pot_channel)          = _adc_parameters.offsets.RR_sus_pot_offset;

    return offsets;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_scales()
{
    std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};

    scales.at(_adc_parameters.channels.thermistor0_channel)         = _adc_parameters.scales.thermistor0_scale;
    scales.at(_adc_parameters.channels.thermistor1_channel)         = _adc_parameters.scales.thermistor1_scale;
    scales.at(_adc_parameters.channels.thermistor2_channel)         = _adc_parameters.scales.thermistor2_scale;
    scales.at(_adc_parameters.channels.thermistor3_channel)         = _adc_parameters.scales.thermistor3_scale;
    scales.at(_adc_parameters.channels.thermistor4_channel)         = _adc_parameters.scales.thermistor4_scale;
    scales.at(_adc_parameters.channels.thermistor5_channel)         = _adc_parameters.scales.thermistor5_scale;
    scales.at(_adc_parameters.channels.thermistor6_channel)         = _adc_parameters.scales.thermistor6_scale;
    scales.at(_adc_parameters.channels.thermistor7_channel)         = _adc_parameters.scales.thermistor7_scale;

    return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_offsets()
{
    std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

    offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor0_offset;
    offsets.at(_adc_parameters.channels.thermistor1_channel)        = _adc_parameters.offsets.thermistor1_offset;
    offsets.at(_adc_parameters.channels.thermistor2_channel)        = _adc_parameters.offsets.thermistor2_offset;
    offsets.at(_adc_parameters.channels.thermistor3_channel)        = _adc_parameters.offsets.thermistor3_offset;
    offsets.at(_adc_parameters.channels.thermistor4_channel)        = _adc_parameters.offsets.thermistor4_offset;
    offsets.at(_adc_parameters.channels.thermistor5_channel)        = _adc_parameters.offsets.thermistor5_offset;
    offsets.at(_adc_parameters.channels.thermistor6_channel)        = _adc_parameters.offsets.thermistor6_offset;
    offsets.at(_adc_parameters.channels.thermistor7_channel)        = _adc_parameters.offsets.thermistor7_offset;

    return offsets;
}

AnalogConversion_s ADCInterface::get_glv() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.glv_sense_channel);
}

AnalogConversion_s ADCInterface::get_bspd_current() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.current_sense_channel);
}

AnalogConversion_s ADCInterface::get_bspd_reference_current() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.reference_sense_channel);
}

AnalogConversion_s ADCInterface::get_RL_load_cell() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.RL_load_cell_channel);
}

AnalogConversion_s ADCInterface::get_RR_load_cell() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.RR_load_cell_channel);
}

AnalogConversion_s ADCInterface::get_RL_sus_pot() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.RL_sus_pot_channel);
}

AnalogConversion_s ADCInterface::get_RR_sus_pot() const
{
    return _adc0.data.conversions.at(_adc_parameters.channels.RR_sus_pot_channel);
}

void ADCInterface::update_filtered_values(float alpha)
{
    _RL_load_cell_filtered = _apply_iir_filter(alpha,
                                            _RL_load_cell_filtered,
                                            get_RL_load_cell().conversion
    );
    _RR_load_cell_filtered = _apply_iir_filter(alpha,
                                            _RR_load_cell_filtered,
                                            get_RR_load_cell().conversion
    );
    _RL_sus_pot_filtered = _apply_iir_filter(alpha,
                                            _RL_sus_pot_filtered,
                                            get_RL_sus_pot().conversion
    );
    _RR_sus_pot_filtered = _apply_iir_filter(alpha,
                                            _RR_sus_pot_filtered,
                                            get_RR_sus_pot().conversion
    );
}

float ADCInterface::get_filtered_RL_load_cell() const
{
    return _RL_load_cell_filtered;
}

float ADCInterface::get_filtered_RR_load_cell() const
{
    return _RR_load_cell_filtered;
}

float ADCInterface::get_filtered_RL_sus_pot() const
{
    return _RL_sus_pot_filtered;
}

float ADCInterface::get_filtered_RR_sus_pot() const
{
    return _RR_sus_pot_filtered;
}

AnalogConversion_s ADCInterface::get_thermistor_0() const {
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor0_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_1() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor1_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_2() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor2_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_3() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor3_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_4() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor4_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_5() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor5_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_6() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor6_channel);
}

AnalogConversion_s ADCInterface::get_thermistor_7() const
{
    return _adc1.data.conversions.at(_adc_parameters.channels.thermistor7_channel);
}

float ADCInterface::get_thermistor_n_degrees_C(int n) const
{
    int thermistor_reading;
    switch (n)
    {
        case 0:
        {
            thermistor_reading = ADCInterface::get_thermistor_0().raw;
            break;
        }
        case 1:
        {
            thermistor_reading = ADCInterface::get_thermistor_1().raw;
            break;
        }
        case 2:
        {
            thermistor_reading = ADCInterface::get_thermistor_2().raw;
            break;
        }
        case 3:
        {
            thermistor_reading = ADCInterface::get_thermistor_3().raw;
            break;
        }
        case 4:
        {
            thermistor_reading = ADCInterface::get_thermistor_4().raw;
            break;
        }
        case 5:
        {
            thermistor_reading = ADCInterface::get_thermistor_5().raw;
            break;
        }
        case 6:
        {
            thermistor_reading = ADCInterface::get_thermistor_6().raw;
            break;
        }
        case 7:
        {
            thermistor_reading = ADCInterface::get_thermistor_7().raw;
            break;
        }
        default:
        {
            return -1.0; // default to -1 to denote incorrect thermistor number
        }
    }

    return _adc_parameters.offsets.coolant_temp_offset + (_adc_parameters.scales.coolant_temp_scale * log(thermistor_reading));
}

bool ADCInterface::is_brake_sense_high() const
{
    return analogRead(_adc_parameters.pinouts.brake_high_sense_pin) > _digital_high_threshold;
}

bool ADCInterface::is_current_sense_high() const
{
    return analogRead(_adc_parameters.pinouts.brake_high_sense_pin) > _digital_high_threshold;
}

float ADCInterface::_apply_iir_filter(float alpha, float old_value, float new_value)
{
    return (alpha * new_value) + (1.0f - alpha) * old_value;
}
