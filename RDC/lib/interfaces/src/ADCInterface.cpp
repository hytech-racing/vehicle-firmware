#include "ADCInterface.h"

bool ADCInterface::getEBSCtrlExt() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.ebs_ctrl_ext_channel).raw;
}

bool ADCInterface::getEBSSupplyRssExt() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.ebs_supply_rss_ext_channel).raw;
}

bool ADCInterface::getEBSSupplyOut() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.ebs_supply_out_channel).raw;
}

bool ADCInterface::getShdnL() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.shdn_l_channel).raw;
}

bool ADCInterface::getShdnN() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.shdn_n_channel).raw;
}

bool ADCInterface::getDsShdnM() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.ds_shdn_m_channel).raw;
}

bool ADCInterface::getDsmsActiveExt() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.dsms_active_ext_channel).raw;
}

bool ADCInterface::getShdnDsOk() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.shdn_ds_ok_channel).raw;
}

bool ADCInterface::getShdnM() const
{
    return (bool)_converted_values.at(_adc_parameters.channels.shdn_m_channel).raw;
}

const AnalogConversion_s &ADCInterface::getStrainGauge1() const
{
    return _converted_values.at(_adc_parameters.channels.strain_gauge_1_channel);
}

const AnalogConversion_s &ADCInterface::getStrainGauge2() const
{
    return _converted_values.at(_adc_parameters.channels.strain_gauge_2_channel);
}

const AnalogConversion_s &ADCInterface::getStrainGauge3() const
{
    return _converted_values.at(_adc_parameters.channels.strain_gauge_3_channel);
}

const AnalogConversion_s &ADCInterface::getStrainGauge4() const
{
    return _converted_values.at(_adc_parameters.channels.strain_gauge_4_channel);
}

const AnalogConversion_s &ADCInterface::getStrainGauge5() const
{
    return _converted_values.at(_adc_parameters.channels.strain_gauge_5_channel);
}

void ADCInterface::tick()
{
    // Create refs for readability
    const auto &channels = _adc_parameters.channels;
    const auto &scales = _adc_parameters.scales;
    const auto &offsets = _adc_parameters.offsets;
    const auto &pinouts = _adc_parameters.pinouts;

    // Read raw values for each signal
    _converted_values.at(channels.ebs_ctrl_ext_channel).raw = digitalRead(pinouts.ebs_ctrl_ext_pin);
    _converted_values.at(channels.ebs_supply_rss_ext_channel).raw = digitalRead(pinouts.ebs_supply_rss_ext_pin);
    _converted_values.at(channels.ebs_supply_out_channel).raw = digitalRead(pinouts.ebs_supply_out_pin);
    _converted_values.at(channels.shdn_l_channel).raw = digitalRead(pinouts.shdn_l_pin);
    _converted_values.at(channels.shdn_n_channel).raw = digitalRead(pinouts.shdn_n_pin);
    _converted_values.at(channels.ds_shdn_m_channel).raw = digitalRead(pinouts.ds_shdn_m_pin);
    _converted_values.at(channels.dsms_active_ext_channel).raw = digitalRead(pinouts.dsms_active_ext_pin);
    _converted_values.at(channels.shdn_ds_ok_channel).raw = digitalRead(pinouts.shdn_ds_ok_pin);
    _converted_values.at(channels.shdn_m_channel).raw = digitalRead(pinouts.shdn_m_pin);

    _converted_values.at(channels.strain_gauge_1_channel).raw = analogRead(pinouts.strain_gauge_1_pin);
    _converted_values.at(channels.strain_gauge_2_channel).raw = analogRead(pinouts.strain_gauge_2_pin);
    _converted_values.at(channels.strain_gauge_3_channel).raw = analogRead(pinouts.strain_gauge_3_pin);
    _converted_values.at(channels.strain_gauge_4_channel).raw = analogRead(pinouts.strain_gauge_4_pin);
    _converted_values.at(channels.strain_gauge_5_channel).raw = analogRead(pinouts.strain_gauge_5_pin);

    // Do conversions
    _converted_values.at(channels.ebs_ctrl_ext_channel).conversion = _converted_values.at(channels.ebs_ctrl_ext_channel).raw * scales.ebs_ctrl_ext_scale + offsets.ebs_ctrl_ext_offset;
    _converted_values.at(channels.ebs_supply_rss_ext_channel).conversion =
        _converted_values.at(channels.ebs_supply_rss_ext_channel).raw * scales.ebs_supply_rss_ext_scale + offsets.ebs_supply_rss_ext_offset;
    _converted_values.at(channels.ebs_supply_out_channel).conversion = _converted_values.at(channels.ebs_supply_out_channel).raw * scales.ebs_supply_out_scale + offsets.ebs_supply_out_offset;
    _converted_values.at(channels.shdn_l_channel).conversion = _converted_values.at(channels.shdn_l_channel).raw * scales.shdn_l_scale + offsets.shdn_l_offset;
    _converted_values.at(channels.shdn_n_channel).conversion = _converted_values.at(channels.shdn_n_channel).raw * scales.shdn_n_scale + offsets.shdn_n_offset;
    _converted_values.at(channels.ds_shdn_m_channel).conversion = _converted_values.at(channels.ds_shdn_m_channel).raw * scales.ds_shdn_m_scale + offsets.ds_shdn_m_offset;
    _converted_values.at(channels.dsms_active_ext_channel).conversion = _converted_values.at(channels.dsms_active_ext_channel).raw * scales.dsms_active_ext_scale + offsets.dsms_active_ext_offset;
    _converted_values.at(channels.shdn_ds_ok_channel).conversion = _converted_values.at(channels.shdn_ds_ok_channel).raw * scales.shdn_ds_ok_scale + offsets.shdn_ds_ok_offset;
    _converted_values.at(channels.shdn_m_channel).conversion = _converted_values.at(channels.shdn_m_channel).raw * scales.shdn_m_scale + offsets.shdn_m_offset;
    _converted_values.at(channels.strain_gauge_1_channel).conversion = _converted_values.at(channels.strain_gauge_1_channel).raw * scales.strain_gauge_1_scale + offsets.strain_gauge_1_offset;
    _converted_values.at(channels.strain_gauge_2_channel).conversion = _converted_values.at(channels.strain_gauge_2_channel).raw * scales.strain_gauge_2_scale + offsets.strain_gauge_2_offset;
    _converted_values.at(channels.strain_gauge_3_channel).conversion = _converted_values.at(channels.strain_gauge_3_channel).raw * scales.strain_gauge_3_scale + offsets.strain_gauge_3_offset;
    _converted_values.at(channels.strain_gauge_4_channel).conversion = _converted_values.at(channels.strain_gauge_4_channel).raw * scales.strain_gauge_4_scale + offsets.strain_gauge_4_offset;
    _converted_values.at(channels.strain_gauge_5_channel).conversion = _converted_values.at(channels.strain_gauge_5_channel).raw * scales.strain_gauge_5_scale + offsets.strain_gauge_5_offset;

    // Ignoring the status part of AnalogConversion_t
}

const ADCInterfaceParams_s ADCInterface::getADCParams() const
{
    return _adc_parameters;
}
