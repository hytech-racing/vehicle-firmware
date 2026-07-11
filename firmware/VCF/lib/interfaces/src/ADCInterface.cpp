#include "ADCInterface.h"

void ADCInterface::tick_adc0()
{
    _adc0.tick();
}

void ADCInterface::tick_adc1()
{
    _adc1.tick();
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc0_scales()
{
  std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};

  scales[_adc_parameters.channels.pedal_ref_channel]    = _adc_parameters.scales.pedal_ref_scale;
  scales[_adc_parameters.channels.steering_cw_channel]  = _adc_parameters.scales.steering_cw_scale;
  scales[_adc_parameters.channels.steering_ccw_channel] = _adc_parameters.scales.steering_ccw_scale;
  scales[_adc_parameters.channels.accel_1_channel]      = _adc_parameters.scales.accel_1_scale;
  scales[_adc_parameters.channels.accel_2_channel]      = _adc_parameters.scales.accel_2_scale;
  scales[_adc_parameters.channels.brake_1_channel]      = _adc_parameters.scales.brake_1_scale;
  scales[_adc_parameters.channels.brake_2_channel]      = _adc_parameters.scales.brake_2_scale;

  return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc0_offsets()
{
  std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

  offsets[_adc_parameters.channels.pedal_ref_channel]     = _adc_parameters.offsets.pedal_ref_offset;
  offsets[_adc_parameters.channels.steering_cw_channel]   = _adc_parameters.offsets.steering_cw_offset;
  offsets[_adc_parameters.channels.steering_ccw_channel]  = _adc_parameters.offsets.steering_ccw_offset;
  offsets[_adc_parameters.channels.accel_1_channel]       = _adc_parameters.offsets.accel_1_offset;
  offsets[_adc_parameters.channels.accel_2_channel]       = _adc_parameters.offsets.accel_2_offset;
  offsets[_adc_parameters.channels.brake_1_channel]       = _adc_parameters.offsets.brake_1_offset;
  offsets[_adc_parameters.channels.brake_2_channel]       = _adc_parameters.offsets.brake_2_offset;

  return offsets;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_scales()
{
  std::array<float, adc_default_parameters::channels_within_mcp_adc> scales = {};

  scales[_adc_parameters.channels.shdn_h_channel]               = _adc_parameters.scales.shdn_h_scale;
  scales[_adc_parameters.channels.shdn_d_channel]               = _adc_parameters.scales.shdn_d_scale;
  scales[_adc_parameters.channels.fl_loadcell_channel]          = _adc_parameters.scales.fl_loadcell_scale;
  scales[_adc_parameters.channels.fr_loadcell_channel]          = _adc_parameters.scales.fr_loadcell_scale;
  scales[_adc_parameters.channels.fl_suspot_channel]            = _adc_parameters.scales.fl_suspot_scale;
  scales[_adc_parameters.channels.fr_suspot_channel]            = _adc_parameters.scales.fr_suspot_scale;
  scales[_adc_parameters.channels.brake_pressure_front_channel] = _adc_parameters.scales.brake_pressure_front_scale;
  scales[_adc_parameters.channels.brake_pressure_rear_channel]  = _adc_parameters.scales.brake_pressure_rear_scale;

  return scales;
}

std::array<float, adc_default_parameters::channels_within_mcp_adc> ADCInterface::adc1_offsets()
{
  std::array<float, adc_default_parameters::channels_within_mcp_adc> offsets = {};

  offsets[_adc_parameters.channels.shdn_h_channel]                = _adc_parameters.offsets.shdn_h_offset;
  offsets[_adc_parameters.channels.shdn_d_channel]                = _adc_parameters.offsets.shdn_d_offset;
  offsets[_adc_parameters.channels.fl_loadcell_channel]           = _adc_parameters.offsets.fl_loadcell_offset;
  offsets[_adc_parameters.channels.fr_loadcell_channel]           = _adc_parameters.offsets.fr_loadcell_offset;
  offsets[_adc_parameters.channels.fl_suspot_channel]             = _adc_parameters.offsets.fl_suspot_offset;
  offsets[_adc_parameters.channels.fr_suspot_channel]             = _adc_parameters.offsets.fr_suspot_offset;
  offsets[_adc_parameters.channels.brake_pressure_front_channel]  = _adc_parameters.offsets.brake_pressure_front_offset;
  offsets[_adc_parameters.channels.brake_pressure_rear_channel]   = _adc_parameters.offsets.brake_pressure_rear_offset;

  return offsets;
}

/* -------------------- ADC 0 Functions -------------------- */

AnalogConversion_s ADCInterface::pedal_reference() {
    return _adc0.data.conversions[_adc_parameters.channels.pedal_ref_channel];
}

AnalogConversion_s ADCInterface::get_steering_degrees_cw()
{
    return _adc0.data.conversions[_adc_parameters.channels.steering_cw_channel];
}

AnalogConversion_s ADCInterface::get_steering_degrees_ccw()
{
    return _adc0.data.conversions[_adc_parameters.channels.steering_ccw_channel];
}

AnalogConversion_s ADCInterface::get_acceleration_1()
{
    return _adc0.data.conversions[_adc_parameters.channels.accel_1_channel];
}

AnalogConversion_s ADCInterface::get_acceleration_2()
{
    return _adc0.data.conversions[_adc_parameters.channels.accel_2_channel];
}

AnalogConversion_s ADCInterface::get_brake_1()
{
    return _adc0.data.conversions[_adc_parameters.channels.brake_1_channel];
}

AnalogConversion_s ADCInterface::get_brake_2()
{
    return _adc0.data.conversions[_adc_parameters.channels.brake_2_channel];
}

/* -------------------- ADC 1 Functions -------------------- */

AnalogConversion_s ADCInterface::shdn_h()
{
    return _adc1.data.conversions[_adc_parameters.channels.shdn_h_channel];
}

AnalogConversion_s ADCInterface::shdn_d()
{
    return _adc1.data.conversions[_adc_parameters.channels.shdn_d_channel];
}

AnalogConversion_s ADCInterface::get_FL_load_cell()
{
    return _adc1.data.conversions[_adc_parameters.channels.fl_loadcell_channel];
}

AnalogConversion_s ADCInterface::get_FR_load_cell()
{
    return _adc1.data.conversions[_adc_parameters.channels.fr_loadcell_channel];
}

AnalogConversion_s ADCInterface::get_FL_sus_pot()
{
    return _adc1.data.conversions[_adc_parameters.channels.fl_suspot_channel];
}

AnalogConversion_s ADCInterface::get_FR_sus_pot()
{
    return _adc1.data.conversions[_adc_parameters.channels.fr_suspot_channel];
}

AnalogConversion_s ADCInterface::get_brake_pressure_front() {
    return _adc1.data.conversions[_adc_parameters.channels.brake_pressure_front_channel];
}

AnalogConversion_s ADCInterface::get_brake_pressure_rear() {
    return _adc1.data.conversions[_adc_parameters.channels.brake_pressure_rear_channel];
}

void ADCInterface::update_filtered_values(float alpha)
{
    _FL_load_cell_filtered = _apply_iir_filter(
        alpha,
        _FL_load_cell_filtered,
        get_FL_load_cell().conversion
    );
    _FR_load_cell_filtered = _apply_iir_filter(
        alpha,
        _FR_load_cell_filtered,
        get_FR_load_cell().conversion
    );
    _FL_sus_pot_filtered = _apply_iir_filter(
        alpha,
        _FL_sus_pot_filtered,
        get_FL_sus_pot().conversion
    );
    _FR_sus_pot_filtered = _apply_iir_filter(
        alpha,
        _FR_sus_pot_filtered,
        get_FR_sus_pot().conversion
    );
}

float ADCInterface::get_filtered_FL_load_cell()
{
    return _FL_load_cell_filtered;
}

float ADCInterface::get_filtered_FR_load_cell()
{
    return _FR_load_cell_filtered;
}

float ADCInterface::get_filtered_FL_sus_pot()
{
    return _FL_sus_pot_filtered;
}

float ADCInterface::get_filtered_FR_sus_pot()
{
    return _FR_sus_pot_filtered;
}

float ADCInterface::_apply_iir_filter(float alpha, float prev_value, float new_value)
{
    return (alpha * new_value) + (1 - alpha) * (prev_value);
}