#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include <MCP_ADC.h>

using pin = uint8_t;


namespace adc_default_parameters
{
    constexpr const unsigned int channels_within_mcp_adc = 8;
}
struct ADCPinout_s
{
    pin adc0_spi_cs_pin;
    pin adc1_spi_cs_pin;
};

struct ADCChannels_s
{
    /* ADC 0 */
    int pedal_ref_channel;
    int steering_cw_channel;
    int steering_ccw_channel;
    int accel_1_channel;
    int accel_2_channel;
    int brake_1_channel;
    int brake_2_channel;

    /* ADC 1 */
    int shdn_h_channel;
    int shdn_d_channel;
    int fl_loadcell_channel;
    int fr_loadcell_channel;
    int fr_suspot_channel;
    int fl_suspot_channel;
    int brake_pressure_front_channel;
    int brake_pressure_rear_channel;
};

struct ADCScales_s
{
    /* ADC 0 */
    float pedal_ref_scale;
    float steering_cw_scale;
    float steering_ccw_scale;
    float accel_1_scale;
    float accel_2_scale;
    float brake_1_scale;
    float brake_2_scale;

    /* ADC 1 */
    float shdn_h_scale;
    float shdn_d_scale;
    float fl_loadcell_scale;
    float fr_loadcell_scale;
    float fl_suspot_scale;
    float fr_suspot_scale;
    float brake_pressure_front_scale;
    float brake_pressure_rear_scale;
};

struct ADCOffsets_s
{
    /* ADC 0 */
    float pedal_ref_offset;
    float steering_cw_offset;
    float steering_ccw_offset;
    float accel_1_offset;
    float accel_2_offset;
    float brake_1_offset;
    float brake_2_offset;

    /* ADC 1 */
    float shdn_h_offset;
    float shdn_d_offset;
    float fl_loadcell_offset;
    float fr_loadcell_offset;
    float fl_suspot_offset;
    float fr_suspot_offset;
    float brake_pressure_front_offset;
    float brake_pressure_rear_offset;
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
            .pinouts = pinouts,
            .channels = channels,
            .scales = scales,
            .offsets = offsets
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
     * Samples from ADC0
     */
    void tick_adc0();

    /**
     * Samples from ADC1
     */
    void tick_adc1();


    /* -------------------- ADC0 -------------------- */

    /**
     * @return Pedal Reference Reading
     */
    AnalogConversion_s pedal_reference();

    /**
     * @return Analog Steering Degrees [Steering 1]
     */
    AnalogConversion_s get_steering_degrees_cw();

    /**
     * @return Analog Steering Degrees [Steering 2]
     */
    AnalogConversion_s get_steering_degrees_ccw();

    /**
     * @return Acceleration Pedal 1
     */
    AnalogConversion_s get_acceleration_1();

    /**
     * @return Acceleration Pedal 2
     */
    AnalogConversion_s get_acceleration_2();

    /**
     * @return Brake Pedal 1
     */
    AnalogConversion_s get_brake_1();

    /**
     * @return Brake Pedal 2
     */
    AnalogConversion_s get_brake_2();

    /* -------------------- ADC1 -------------------- */

    /**
     * @return SHDN H Voltage Sense
     */
    AnalogConversion_s shdn_h();

    /**
     * @return SHDN D Voltage Sense
     */
    AnalogConversion_s shdn_d();

    /**
     * @return Front Left Load Cell
     */
    AnalogConversion_s get_FL_load_cell();

    /**
     * @return Front Right Load Cell
     */
    AnalogConversion_s get_FR_load_cell();

    /**
     * @return Front Left Suspension Potentiometer Reading
     */
    AnalogConversion_s get_FL_sus_pot();

    /**
     * @return Front Right Suspension Potentiometer Reading
     */
    AnalogConversion_s get_FR_sus_pot();

    /**
     * @return Front Brake Pressure
     */
    AnalogConversion_s get_brake_pressure_front();

    /**
     * @return Rear Brake Pressure
     */
    AnalogConversion_s get_brake_pressure_rear();

    /**
     * Update the filtered values for the load cells and sus pots.
     * Uses the iir_filter method to do so.
     */
    void update_filtered_values(float alpha);

    /**
     * @return Filtered Front Left Load Cell
     */
    float get_filtered_FL_load_cell();

    /**
     * @return Filtered Front Right Load Cell
     */
    float get_filtered_FR_load_cell();

    /**
     * @return Filtered Front Left Sus Pot
     */
    float get_filtered_FL_sus_pot();

    /**
     * @return Filtered Front Right Sus Pot
     */
    float get_filtered_FR_sus_pot();

private:

    ADCInterfaceParams_s _adc_parameters = {};

    // MCP3208. ADC0 in VCF schematic. Used for steering, pedal reference, and pedal position sensors.
    MCP_ADC<adc_default_parameters::channels_within_mcp_adc> _adc0;
    // MCP3208. ADC1 in VCF schematic. Used for SHDN senses, load cells, suspension potentiometers, and brake pressure sensors.
    MCP_ADC<adc_default_parameters::channels_within_mcp_adc> _adc1;

    float _FL_load_cell_filtered;
    float _FR_load_cell_filtered;
    float _FL_sus_pot_filtered;
    float _FR_sus_pot_filtered;
    
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_scales();
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_offsets();
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_scales();
    std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_offsets();

    /**
     * @return updated filtered value based on given alpha, previous filtered value, and new measured value
     */
    static float _apply_iir_filter(float alpha, float prev_value, float new_value);

};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif /* ADC_INTERFACE_H */