#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

#include "Arduino.h"
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

// Strain gauges are wired in reverse of the analog pin numbers:
// gauge 1 is A4, gauge 5 is A0. Pin 11 is open, so shdn_m is pin 10.
struct ADCPinout_s {
  int ebs_ctrl_ext_pin;
  int ebs_supply_rss_ext_pin;
  int ebs_supply_out_pin;
  int shdn_l_pin;
  int shdn_n_pin;
  int ds_shdn_m_pin;
  int dsms_active_ext_pin;
  int shdn_ds_ok_pin;
  int shdn_m_pin;

  int strain_gauge_1_pin;
  int strain_gauge_2_pin;
  int strain_gauge_3_pin;
  int strain_gauge_4_pin;
  int strain_gauge_5_pin;
};

struct ADCChannels_s {
  int ebs_ctrl_ext_channel;
  int ebs_supply_rss_ext_channel;
  int ebs_supply_out_channel;
  int shdn_l_channel;
  int shdn_n_channel;
  int ds_shdn_m_channel;
  int dsms_active_ext_channel;
  int shdn_ds_ok_channel;
  int shdn_m_channel;

  int strain_gauge_1_channel;
  int strain_gauge_2_channel;
  int strain_gauge_3_channel;
  int strain_gauge_4_channel;
  int strain_gauge_5_channel;
};

struct ADCScales_s {
  float ebs_ctrl_ext_scale;
  float ebs_supply_rss_ext_scale;
  float ebs_supply_out_scale;
  float shdn_l_scale;
  float shdn_n_scale;
  float ds_shdn_m_scale;
  float dsms_active_ext_scale;
  float shdn_ds_ok_scale;
  float shdn_m_scale;

  float strain_gauge_1_scale;
  float strain_gauge_2_scale;
  float strain_gauge_3_scale;
  float strain_gauge_4_scale;
  float strain_gauge_5_scale;
};

struct ADCOffsets_s {
  float ebs_ctrl_ext_offset;
  float ebs_supply_rss_ext_offset;
  float ebs_supply_out_offset;
  float shdn_l_offset;
  float shdn_n_offset;
  float ds_shdn_m_offset;
  float dsms_active_ext_offset;
  float shdn_ds_ok_offset;
  float shdn_m_offset;

  float strain_gauge_1_offset;
  float strain_gauge_2_offset;
  float strain_gauge_3_offset;
  float strain_gauge_4_offset;
  float strain_gauge_5_offset;
};

struct ADCInterfaceParams_s {
  ADCPinout_s pinouts;
  ADCChannels_s channels;
  ADCScales_s scales;
  ADCOffsets_s offsets;
};

inline ADCPinout_s schematicADCPinout() {
  return {
      0, 1, 2, 3, 4, 5, 6, 7, 10, A4, A3, A2, A1, A0,
  };
}

inline ADCChannels_s schematicADCChannels() {
  return {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
  };
}

class ADCInterface {
public:
  ADCInterface(ADCInterfaceParams_s params) : _adc_parameters(params) {
    analogReadResolution(12); // set analog reads to 12 bits
    pinMode(_adc_parameters.pinouts.ebs_ctrl_ext_pin, INPUT);
    pinMode(_adc_parameters.pinouts.ebs_supply_rss_ext_pin, INPUT);
    pinMode(_adc_parameters.pinouts.ebs_supply_out_pin, INPUT);
    pinMode(_adc_parameters.pinouts.shdn_l_pin, INPUT);
    pinMode(_adc_parameters.pinouts.shdn_n_pin, INPUT);
    pinMode(_adc_parameters.pinouts.ds_shdn_m_pin, INPUT);
    pinMode(_adc_parameters.pinouts.dsms_active_ext_pin, INPUT);
    pinMode(_adc_parameters.pinouts.shdn_ds_ok_pin, INPUT);
    pinMode(_adc_parameters.pinouts.shdn_m_pin, INPUT);
  };

  bool getEBSCtrlExt() const;
  bool getEBSSupplyRssExt() const;
  bool getEBSSupplyOut() const;
  bool getShdnL() const;
  bool getShdnN() const;
  bool getDsShdnM() const;
  bool getDsmsActiveExt() const;
  bool getShdnDsOk() const;
  bool getShdnM() const;

  const AnalogConversion_s &getStrainGauge1() const;
  const AnalogConversion_s &getStrainGauge2() const;
  const AnalogConversion_s &getStrainGauge3() const;
  const AnalogConversion_s &getStrainGauge4() const;
  const AnalogConversion_s &getStrainGauge5() const;

  void tick();

  const ADCInterfaceParams_s getADCParams() const;

private:
  ADCInterfaceParams_s _adc_parameters;
  std::array<AnalogConversion_s, 14> _converted_values;
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif // ADCINTERFACE
