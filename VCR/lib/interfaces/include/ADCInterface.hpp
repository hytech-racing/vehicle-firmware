#ifndef ADC_INTERFACE_HPP
#define ADC_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "MCP_ADC.h"

#include <array>
#include <optional>

namespace adc_default_parameters
{
    constexpr const unsigned int CHANNELS_WITHIN_MCP_ADC = 8;
}

struct ADCPinout_s
{
    int adc0_spi_cs_pin;
    int adc1_spi_cs_pin;

    int brake_high_sense_pin;
    int current_high_sense_pin;
};

struct ADCChannels_s
{
    /* ADC 0 */
    int glv_sense_channel;
    int current_sense_channel;
    int reference_sense_channel;
    int rl_load_cell_channel;
    int rr_load_cell_channel;
    int rl_sus_pot_channel;
    int rr_sus_pot_channel;

    /* ADC 1 */
    int thermistor0_channel;
    int thermistor1_channel;
    int thermistor2_channel;
    int thermistor3_channel;
    int thermistor4_channel;
    int thermistor5_channel;
    int thermistor6_channel;
    int thermistor7_channel;
};

struct ADCScales_s
{
    float glv_sense_scale;
    float current_sense_scale;
    float reference_sense_scale;
    float rl_load_cell_scale;
    float rr_load_cell_scale;
    float rl_sus_pot_scale;
    float rr_sus_pot_scale;

    float thermistor0_scale;
    float thermistor1_scale;
    float thermistor2_scale;
    float thermistor3_scale;
    float thermistor4_scale;
    float thermistor5_scale;
    float thermistor6_scale;
    float thermistor7_scale;

    float coolant_temp_scale;
};

struct ADCOffsets_s
{
    float glv_sense_offset;
    float current_sense_offset;
    float reference_sense_offset;
    float rl_load_cell_offset;
    float rr_load_cell_offset;
    float rl_sus_pot_offset;
    float rr_sus_pot_offset;

    float thermistor0_offset;
    float thermistor1_offset;
    float thermistor2_offset;
    float thermistor3_offset;
    float thermistor4_offset;
    float thermistor5_offset;
    float thermistor6_offset;
    float thermistor7_offset;

    float coolant_temp_offset;
};

struct ADCInterfaceParams_s
{
    ADCPinout_s pinouts;
    ADCChannels_s channels;
    ADCScales_s scales;
    ADCOffsets_s offsets;
};

class ADCInterface
{
  public:
    ADCInterface(ADCPinout_s pinouts, ADCChannels_s channels, ADCScales_s scales, ADCOffsets_s offsets)
        : _adc_parameters{pinouts, channels, scales, offsets},
          _adc0(
              _adc_parameters.pinouts.adc0_spi_cs_pin,
              MCP_ADC_DEFAULT_SPI_SDI,
              MCP_ADC_DEFAULT_SPI_SDO,
              MCP_ADC_DEFAULT_SPI_CLK,
              MCP_ADC_DEFAULT_SPI_SPEED,
              _adc0Scales().data(),
              _adc0Offsets().data()),
          _adc1(
              _adc_parameters.pinouts.adc1_spi_cs_pin,
              MCP_ADC_DEFAULT_SPI_SDI,
              MCP_ADC_DEFAULT_SPI_SDO,
              MCP_ADC_DEFAULT_SPI_CLK,
              MCP_ADC_DEFAULT_SPI_SPEED,
              _adc1Scales().data(),
              _adc1Offsets().data()) {};

    /**
     * Init function for pin modes
     */
    void init();

    /**
     * Samples from ADC0
     */
    void tickADC0();

    /**
     * Samples from ADC1
     */
    void tickADC1();

    const ADCInterfaceParams_s& getADCParams() const;

    /* -------------------- ADC0 -------------------- */

    /**
     * @return The reading of the 24V sensor analog channel
     */
    AnalogConversion_s getGlv() const;

    /**
     * @return The reading of the BSPD current analog channel
     */
    AnalogConversion_s getBspdCurrent() const;

    /**
     * @return The reading of the BSPD reference current analog channel
     */
    AnalogConversion_s getBspdReferenceCurrent() const;

    /**
     * @return The reading of the rear left load cell analog channel
     */
    AnalogConversion_s getRLLoadCell() const;

    /**
     * @return The reading of the rear right load cell analog channel
     */
    AnalogConversion_s getRRLoadCell() const;

    /**
     * @return The reading of the rear left suspension potentiometer analog channel
     */
    AnalogConversion_s getRLSusPot() const;

    /**
     * @return The reading of the rear right suspension potentiometer analog channel
     */
    AnalogConversion_s getRRSusPot() const;

    /**
     * Update filtered signals based on given alpha
     */
    void updateFilteredValues(float alpha);

    /**
     * @return The filtered value of the RL load cell
     */
    float getFilteredRLLoadCell() const;

    /**
     * @return The filtered value of the RR load cell
     */
    float getFilteredRRLoadCell() const;

    /**
     * @return The filtered value of the RL sus pot
     */
    float getFilteredRLSusPot() const;

    /**
     * @return The filtered value of the RR sus pot
     */
    float getFilteredRRSusPot() const;

    /* -------------------- ADC1 -------------------- */

    AnalogConversion_s getThermistor0() const;

    AnalogConversion_s getThermistor1() const;

    AnalogConversion_s getThermistor2() const;

    AnalogConversion_s getThermistor3() const;

    AnalogConversion_s getThermistor4() const;

    AnalogConversion_s getThermistor5() const;

    AnalogConversion_s getThermistor6() const;

    AnalogConversion_s getThermistor7() const;

    /**
     * Converts the output of the ADC to a temperature in degrees following the function:
     * deg C = offset + (scale * ln(raw analog value))
     * @param n which thermistor to convert into degrees
     * @return defaults to -1 if incorrect thermistor number is given
     */
    float getThermistorNDegreesC(int n) const;

    bool isBrakeSenseHigh() const;

    bool isCurrentSenseHigh() const;

  private:
    const size_t _digital_high_threshold = 2048;

    ADCInterfaceParams_s _adc_parameters = {};

    MCP_ADC<adc_default_parameters::CHANNELS_WITHIN_MCP_ADC> _adc0;
    MCP_ADC<adc_default_parameters::CHANNELS_WITHIN_MCP_ADC> _adc1;

    float _rl_load_cell_filtered;
    float _rr_load_cell_filtered;
    float _rl_sus_pot_filtered;
    float _rr_sus_pot_filtered;

    std::array<float, adc_default_parameters::CHANNELS_WITHIN_MCP_ADC> _adc0Scales();
    std::array<float, adc_default_parameters::CHANNELS_WITHIN_MCP_ADC> _adc0Offsets();
    std::array<float, adc_default_parameters::CHANNELS_WITHIN_MCP_ADC> _adc1Scales();
    std::array<float, adc_default_parameters::CHANNELS_WITHIN_MCP_ADC> _adc1Offsets();

    /**
     * @return updated filtered value based on given alpha, previous value, and new value
     */
    static float _applyIIRFilter(float alpha, float old_value, float new_value);
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif
