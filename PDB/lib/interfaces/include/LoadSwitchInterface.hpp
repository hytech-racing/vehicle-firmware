#ifndef LDSW_INTERFACE_H
#define LDSW_INTERFACE_H

#include <cstdint>
#include <Arduino.h>


/**
 * @note Formulas:
 *
 *  V_IMON = I_OUT × GAIN_IMON × R_IMON -> GAIN_IMON is constant
 *  I_OUT = V_IMON / (GAIN_IMON × R_IMON)
 *
 *  V_IMON = counts × V_REF / 2^N -> Based on ADC
 *
 *  I_OUT [mA] = [counts × (V_REF / 2^N) / (GAIN_IMON × R_IMON)] * 1000
*/

namespace loadswitch_default_params
{
    // Board-level ADC settings, shared by all switches
    constexpr uint32_t ADC_VREF_V = 3.3;     // ADC reference
    constexpr uint8_t ADC1_RESOLUTION = 16;  // ADC resolution [bits]

    // TPS2663 constants
    constexpr double IMON_GAIN = 27.9e-6;                   // I_IMON / I_OUT = 27.9 uA/A (typical)
    constexpr uint32_t IMON_MAX_VOLTAGE_UV = 4'000'000;     // "The maximum voltage for monitoring the current is limited to 4 V"
    constexpr uint32_t OVERLOAD_CURRENT_NUMERATOR = 18;     // I_OL[Ampere] = 18 / R_ILIM[kilo-ohm]

    constexpr uint32_t LATCH_RESET_LOW_US = 100; // SHDN low pulse to clear latch (> 1.5 us)
}

/**
 * @brief Struct defines the parameters/characteristics of each individual loadswitch
 * @param enable_pin is the same as SHDN; HIGH/Driven = on
 * @param fault_pin is the same as FLT; LOW/pulled down = fault
 * @param imon_pin is the same as IMON; IMON -> ADC
 * @param imon_resistor_ohms is the value of the IMON-to-GND resistor
 * @param ilim_resistor_ohms is the value of the ILIM-to-GND resistor
 * @param startup_ignore_fault_ms is the amount of time we ignore FLT after enable (turn-on delay + dVdT ramp)
*/
struct LoadSwitchParams_s {
    uint32_t fault_pin;
    uint32_t enable_pin;
    uint32_t imon_pin;
    uint32_t imon_resistor_ohms;
    uint32_t ilim_resistor_ohms;
    uint32_t startup_ignore_fault_ms;
};

/**
 * @brief Struct holds the status and data that pertains to a loadswitch
 * @param is_loadswitch_enabled is true when the loadswitch is driven HIGH by the MCU
 * @param is_loadswitch_faulted is true when the loadswitch drives the FLT line LOW
 * @param is_loadswitch_satured is true when the IMON current is capped by the ADC or the IMON voltage
 * @param _imon_current_mA is the live IMON current reading
*/
struct LoadSwitchData_s
{
    bool is_loadswitch_enabled;
    bool is_loadswitch_faulted;
    bool is_loadswitch_satured;
    uint32_t imon_current_mA;
};

class LoadSwitchInterface
{
public:

    explicit LoadSwitchInterface(const LoadSwitchParams_s& params
    ): _params(params),
      _imon_mA_per_count(_computeMilliampsPerCount(params.imon_resistor_ohms)),
      _current_limit_mA(params.ilim_resistor_ohms ? loadswitch_default_params::OVERLOAD_CURRENT_NUMERATOR / params.ilim_resistor_ohms : 0)
    {};

    void init();

    /**
     * @brief Enables the loadswitch by setting SHDN pin HIGH
     * @note Only start the ignore fault timer on an off -> on transition
    */
    void enable();


    void disable();

    /**
     * @brief Clear a latched fault (MODE pin open = latch-off) by pulsing SHDN low,
     *        then re-enable. Blocks for ldsw::LATCH_RESET_LOW_US. Leaves the switch enabled.
     */
    void reset_latched_fault();

    /**
     * @brief Sample the FLT pin
     * @note Only check when loadswitch is enabled and after the start-up ramp has finished
     * @return Nothing is returned, just update internal variable
    */
    void sampleFault();

    void sampleIMON(); // Read IMON and update current in mA.

    bool isLoadSwitchEnabled() const { return _current_data.is_loadswitch_enabled; } // commanded state, not actual rail status
    bool isLoadSwitchFaulted() const { return _current_data.is_loadswitch_faulted; } // most recent qualified FLT sample
    uint32_t getIMONCurrent() const { return _current_data.imon_current_mA; }

    uint32_t current_limit_mA() const { return _current_limit_mA; } // programmed I_OL
    float imon_full_scale_mA() const;                            // max readable current

private:

    /**
     * @brief Convert ADC counts to milliamps (IMON)
     * @return IMON current in milliamps
    */
    static float _convertCountsTomA(uint16_t counts, float mA_per_count);

    /**
     * @brief Compute milliamps per count for IMON based on given equations (8.3.9)
     * @return IMON milliamps represented by one ADC count for a given R_IMON
    */
    static float _computeMilliampsPerCount(uint32_t imon_resistor_ohms);

    LoadSwitchParams_s _params;
    LoadSwitchData_s _current_data;
    uint32_t _imon_mA_per_count;
    uint32_t _current_limit_mA;
    uint32_t _enable_time_ms = 0;
};
extern LoadSwitchInterface LDSWs[6];

#endif // LDSW_INTERFACE_H