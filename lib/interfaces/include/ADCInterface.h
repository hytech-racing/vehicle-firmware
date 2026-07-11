#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "ButtonInterface.h"

using pin = uint8_t;


namespace adc_default_parameters
{
    constexpr float TEENSY41_MAX_INPUT_VOLTAGE = 3.3f;
    constexpr float TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH = 0.5F;
    constexpr float TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH = 2.8F;
    constexpr float SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD = 12.0F;

    constexpr float CONTROL_PILOT_VOLTAGE_LOW_THRESHOLD = 0.5f;
    constexpr float PROXIMITY_PILOT_VOLTAGE_HIGH_THRESHOLD = 4.0f;
};

struct ADCPinout_s
{
    /**
     * NOTE: All SHDN pins are currently digital pins. However, if there is an issue where shdn voltage is fluctuating,
     *       make a transition to reading shdn C as an analog pin
     */
    pin teensy_shdn_A_pin;
    pin teensy_shdn_B_pin;
    pin teensy_shdn_C_pin;
    pin teensy_shdn_D_pin;
    pin teensy_shdn_E_pin;
    pin teensy_shdn_F_pin;
    pin teensy_shdn_G_pin;

    pin teensy_scaled_24V_pin;
    pin teensy_control_pilot_pin;
    pin teensy_proximity_pilot_pin;
    pin teensy_240_enabled_pin;
    pin teensy_240_ok_pin;

    pin teensy_jumper_out_pin;
    pin reset_error_button_pin;
};

struct ADCConfigs_s
{
    float teensy41_max_input_voltage;
};

struct ADCConversions_s
{
    float glv_conv_factor;
    float control_pilot_conv_factor;
    float proximity_pilot_conv_factor;
    float jumper_out_conv_factor;
};

struct ADCThresholds_s
{
    float teensy41_min_digital_read_voltage_thresh;
    float teensy41_max_digital_read_voltage_thresh;
    float shutdown_voltage_digital_threshold;
};

struct ADCInterfaceParams_s
{
    ADCPinout_s pinout;
    ADCConfigs_s configs;
    ADCConversions_s conversions;
    ADCThresholds_s thresholds;
    float bit_resolution;
};

class ADCInterface
{
public:

    ADCInterface(ADCPinout_s pinout,
                ADCConversions_s conversions,
                float bit_resolution,
                ADCConfigs_s configs = {
                    .teensy41_max_input_voltage = adc_default_parameters::TEENSY41_MAX_INPUT_VOLTAGE
                },
                ADCThresholds_s thresholds = {
                    .teensy41_min_digital_read_voltage_thresh = adc_default_parameters::TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH,
                    .teensy41_max_digital_read_voltage_thresh = adc_default_parameters::TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH,
                    .shutdown_voltage_digital_threshold = adc_default_parameters::SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD
                }
    ): _adc_parameters {
            pinout,
            configs,
            [=]() mutable {
                conversions.glv_conv_factor              = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.glv_conv_factor;
                conversions.control_pilot_conv_factor    = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.control_pilot_conv_factor;
                conversions.proximity_pilot_conv_factor  = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.proximity_pilot_conv_factor;
                conversions.jumper_out_conv_factor       = (configs.teensy41_max_input_voltage / bit_resolution) / conversions.jumper_out_conv_factor;
                return conversions;
            }(),
            thresholds,
            bit_resolution
        },
        _reset_error_button(pinout.reset_error_button_pin)
    {};

    /**
     * @pre constructor called and instance created
     * @post Pins on Teensy configured and written as IN/OUT
     */
    void init(uint32_t init_millis);

    /**
     * @return true if shdn A HIGH, else false
     */
    bool read_shdn_A_voltage();

    /**
     * @return true if shdn B HIGH, else false
     */
    bool read_shdn_B_voltage();

    /**
     * @return true if shdn C HIGH, else false
     */
    bool read_shdn_C_voltage();

    /**
     * @return true if shdn D HIGH, else false
     */
    bool read_shdn_D_voltage();

    /**
     * @return true if shdn E HIGH, else false
     */
    bool read_shdn_E_voltage();

    /**
     * @return true if shdn  HIGH, else false
     */
    bool read_shdn_F_voltage();

    /**
     * @return true if shdn G HIGH, else false
     */
    bool read_shdn_G_voltage();

    /**
     * @return voltage value of GLV, nominal 24V
     */
    volt read_global_lv_value();

    /**
     * @return voltage value of Control Pilot
     */
    volt read_control_pilot();

    /**
     * @return true if Control Pilot voltage below threshold, else false
     */
    bool is_control_pilot_low();

    /**
     * @return voltage value of Proximity Pilot
     */
    volt read_proximity_pilot();

    /**
     * @return true if Proximity Pilot voltage above threshold, else false
     */
    bool is_proximity_pilot_high();

    /**
     * @return true if 240_Enabled HIGH, else false
     */
    bool read_240_enabled();

    /**
     * @return true if 240_Ok HIGH, else false
     */
    bool read_240_ok();

    /**
     * @return voltage value of JP_OUT_READ
     */
    volt read_jumper_out();

    /**
     * @return true if JP_OUT_READ HIGH, else false
     */
    bool is_jumper_out_high();

    /**
     * @return true if JP_OUT_READ LOW, else false
     */
    bool is_jumper_out_low();

    /**
     * @return true if reset button pressed, else false
     */
    bool is_reset_errors_button_pressed(unsigned long current_millis);

    /**
     * @return ADC parameters
     */
    const ADCInterfaceParams_s& get_adc_params() const;

private:

    const ADCInterfaceParams_s _adc_parameters = {};
    ButtonInterface _reset_error_button;

    /**
     * @brief timestamp captured in init()
     */
    uint32_t _init_millis = 0;
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif