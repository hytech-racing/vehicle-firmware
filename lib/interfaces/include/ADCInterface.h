#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <array>
#include <optional>
#include "MCP_ADC.h"

namespace adc_default_parameters
{
    constexpr const unsigned int channels_within_mcp_adc = 8;
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
    int RL_load_cell_channel;
    int RR_load_cell_channel;
    int RL_sus_pot_channel;
    int RR_sus_pot_channel;

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
    float RL_load_cell_scale;
    float RR_load_cell_scale;
    float RL_sus_pot_scale;
    float RR_sus_pot_scale;

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
    float RL_load_cell_offset;
    float RR_load_cell_offset;
    float RL_sus_pot_offset;
    float RR_sus_pot_offset;

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

    ADCInterface(ADCPinout_s pinouts,
                ADCChannels_s channels,
                ADCScales_s scales,
                ADCOffsets_s offsets
    ) : _adc_parameters {
            pinouts,
            channels,
            scales,
            offsets
        },
        _adc0 (
            _adc_parameters.pinouts.adc0_spi_cs_pin,
            MCP_ADC_DEFAULT_SPI_SDI,
            MCP_ADC_DEFAULT_SPI_SDO,
            MCP_ADC_DEFAULT_SPI_CLK,
            MCP_ADC_DEFAULT_SPI_SPEED,
            adc0_scales().data(),
            adc0_offsets().data()
        ),
        _adc1 (
            _adc_parameters.pinouts.adc1_spi_cs_pin,
            MCP_ADC_DEFAULT_SPI_SDI,
            MCP_ADC_DEFAULT_SPI_SDO,
            MCP_ADC_DEFAULT_SPI_CLK,
            MCP_ADC_DEFAULT_SPI_SPEED,
            adc1_scales().data(),
            adc1_offsets().data()
        )
    {};

    /**
     * Init function for pin modes
    */
    void init();

    /**
    * Samples from ADC0
    */
    void tick_adc0();

    /**
    * Samples from ADC1
    */
    void tick_adc1();

    const ADCInterfaceParams_s& get_adc_params() const;

    /* -------------------- ADC0 -------------------- */

    /**
     * @return The reading of the 24V sensor analog channel
    */
    AnalogConversion_s get_glv() const;

    /**
     * @return The reading of the BSPD current analog channel
    */
    AnalogConversion_s get_bspd_current() const;

    /**
     * @return The reading of the BSPD reference current analog channel
    */
    AnalogConversion_s get_bspd_reference_current() const;

    /**
     * @return The reading of the rear left load cell analog channel
    */
    AnalogConversion_s get_RL_load_cell() const;

    /**
     * @return The reading of the rear right load cell analog channel
    */
    AnalogConversion_s get_RR_load_cell() const;

    /**
     * @return The reading of the rear left suspension potentiometer analog channel
    */
    AnalogConversion_s get_RL_sus_pot() const;

    /**
     * @return The reading of the rear right suspension potentiometer analog channel
    */
    AnalogConversion_s get_RR_sus_pot() const;

    /**
     * Update filtered signals based on given alpha
     */
    void update_filtered_values(float alpha);

    /**
     * @return The filtered value of the RL load cell
     */
    float get_filtered_RL_load_cell() const;

    /**
     * @return The filtered value of the RR load cell
     */
    float get_filtered_RR_load_cell() const;

    /**
     * @return The filtered value of the RL sus pot
     */
    float get_filtered_RL_sus_pot() const;

    /**
     * @return The filtered value of the RR sus pot
     */
    float get_filtered_RR_sus_pot() const;

    /* -------------------- ADC1 -------------------- */

    AnalogConversion_s get_thermistor_0() const;

    AnalogConversion_s get_thermistor_1() const;

    AnalogConversion_s get_thermistor_2() const;

    AnalogConversion_s get_thermistor_3() const;

    AnalogConversion_s get_thermistor_4() const;

    AnalogConversion_s get_thermistor_5() const;

    AnalogConversion_s get_thermistor_6() const;

    AnalogConversion_s get_thermistor_7() const;

    /**
     * Converts the output of the ADC to a temperature in degrees following the function:
     * deg C = offset + (scale * ln(raw analog value))
     * @param n which thermistor to convert into degrees
     * @return defaults to -1 if incorrect thermistor number is given
     */
    float get_thermistor_n_degrees_C(int n) const;

    bool is_brake_sense_high() const;

    bool is_current_sense_high() const;

private:

    const size_t _digital_high_threshold = 2048;

    ADCInterfaceParams_s _adc_parameters = {};

    MCP_ADC<adc_default_parameters::channels_within_mcp_adc> _adc0;
    MCP_ADC<adc_default_parameters::channels_within_mcp_adc> _adc1;

    float _RL_load_cell_filtered;
    float _RR_load_cell_filtered;
    float _RL_sus_pot_filtered;
    float _RR_sus_pot_filtered;

    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_scales();
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_offsets();
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_scales();
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_offsets();

    /**
     * @return updated filtered value based on given alpha, previous value, and new value
     */
    static float _apply_iir_filter(float alpha, float old_value, float new_value);

};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif
