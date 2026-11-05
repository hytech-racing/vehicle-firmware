#ifndef LDSW_INTERFACE_H
#define LDSW_INTERFACE_H

#include <cstdint>
#include <array>
#include <Arduino.h>
#include <etl/singleton.h>


/**
 * @file LoadSwitchInterface.hpp
 * @brief Driver for the PDB's TPS2663 load switches (eFuses). One instance owns every switch;
 *        each call takes a LoadSwitchIDs_e index. Pins and resistors are passed to the constructor
 *        from initializeAllInterfaces().
 *
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
    constexpr float ADC_VREF_V = 3.3f;       // ADC reference (was uint32_t, which truncated 3.3 to 3)
    constexpr uint8_t ADC1_RESOLUTION = 16;  // ADC resolution [bits]

    // TPS2663 constants
    constexpr double IMON_GAIN = 27.9e-6;                   // I_IMON / I_OUT = 27.9 uA/A (typical)
    constexpr uint32_t IMON_MAX_VOLTAGE_UV = 4'000'000;     // "The maximum voltage for monitoring the current is limited to 4 V"
    constexpr uint32_t OVERLOAD_CURRENT_NUMERATOR = 18;     // I_OL[Ampere] = 18 / R_ILIM[kilo-ohm]

    constexpr uint32_t LATCH_RESET_LOW_US = 100;        // SHDN low pulse to clear latch (> 1.5 us)
    constexpr uint32_t STARTUP_IGNORE_FAULT_MS = 100;   // Turn-on delay + dVdT ramp, see LoadSwitchInterface::sampleFault()
}

/**
 * @enum LoadSwitchIDs_e
 * @brief Index of each TPS2663 load switch. Pins and resistors are set in initializeAllInterfaces().
*/
enum LoadSwitchIDs_e : uint8_t
{
    LDSW_CAMERAS    = 0,
    LDSW_ORIN       = 1,
    LDSW_INV_COOL   = 2,
    LDSW_MOTOR_COOL = 3,
    LDSW_LIDAR      = 4,
    LDSW_DTI        = 5,
    NUM_LOAD_SWITCHES
};

/**
 * @brief Struct defines the parameters/characteristics of each individual loadswitch
 * @param id Which slot this switch fills; the constructor places it by id, so list order doesn't matter
 * @param enable_pin is the same as SHDN; HIGH/Driven = on
 * @param fault_pin is the same as FLT; LOW/pulled down = fault
 * @param imon_pin is the same as IMON; IMON -> ADC
 * @param imon_resistor_ohms is the value of the IMON-to-GND resistor
 * @param ilim_resistor_ohms is the value of the ILIM-to-GND resistor
 * @param startup_ignore_fault_ms is the amount of time we ignore FLT after enable (turn-on delay + dVdT ramp)
*/
struct LoadSwitchParams_s
{
    LoadSwitchIDs_e id;
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
 * @param imon_current_mA is the live IMON current reading
*/
struct LoadSwitchData_s
{
    bool is_loadswitch_enabled = false;
    bool is_loadswitch_faulted = false;
    bool is_loadswitch_satured = false;
    uint32_t imon_current_mA = 0;
};

class LoadSwitchInterface
{
public:

    /**
     * @brief Registers all load switches. Does not touch the GPIOs, so it is safe to construct
     *        before HAL / GPIO init.
     * @param params One entry per switch, in any order; each is placed by its id.
     *        A duplicate or out-of-range id makes the config invalid and init() refuses to run.
    */
    explicit LoadSwitchInterface(const std::array<LoadSwitchParams_s, NUM_LOAD_SWITCHES> &params);

    LoadSwitchInterface(const LoadSwitchInterface &)            = delete;
    LoadSwitchInterface &operator=(const LoadSwitchInterface &) = delete;

    /**
     * @brief Configures every switch's pins with SHDN low, so all switches start off. Call once after GPIO init.
     * @return False if the params passed to the constructor were invalid (nothing is touched)
    */
    bool init();

    /**
     * @brief Enables the loadswitch by setting SHDN pin HIGH
     * @note Only start the ignore fault timer on an off -> on transition
    */
    void enable(uint8_t index);

    void disable(uint8_t index);

    /**
     * @brief Enables / disables every switch
    */
    void enableAll();
    void disableAll();

    /**
     * @brief Clear a latched fault (MODE pin open = latch-off) by pulsing SHDN low,
     *        then re-enable. Blocks for LATCH_RESET_LOW_US. Leaves the switch enabled.
     */
    void reset_latched_fault(uint8_t index);

    /**
     * @brief Sample the FLT pin
     * @note Only check when loadswitch is enabled and after the start-up ramp has finished
     * @return Nothing is returned, just update internal variable
    */
    void sampleFault(uint8_t index);

    void sampleIMON(uint8_t index); // Read IMON and update current in mA.

    /**
     * @brief sampleFault() and sampleIMON() for every switch
    */
    void sampleAll();

    // Accessors. Return false / 0 / nullptr for an invalid index.
    const LoadSwitchData_s *getData(uint8_t index) const;
    bool isLoadSwitchEnabled(uint8_t index) const;       // commanded state, not actual rail status
    bool isLoadSwitchFaulted(uint8_t index) const;       // most recent qualified FLT sample
    uint32_t getIMONCurrent(uint8_t index) const;
    uint32_t current_limit_mA(uint8_t index) const;      // programmed I_OL
    float imon_full_scale_mA(uint8_t index) const;       // max readable current
    bool anyFault() const;
    bool isConfigValid() const { return _is_config_valid; }

private:

    /**
     * @brief Represents one load switch
     * @param params Pins and resistors, see LoadSwitchParams_s
     * @param data Live status and IMON reading, see LoadSwitchData_s
     * @param imon_mA_per_count IMON scale; float, values are well below 1 mA per count
     * @param current_limit_mA Programmed overload current I_OL
     * @param enable_time_ms millis() at the last off -> on transition; FLT is ignored until startup_ignore_fault_ms after it
    */
    struct LoadSwitch_s
    {
        LoadSwitchParams_s params;
        LoadSwitchData_s data;
        float imon_mA_per_count;
        uint32_t current_limit_mA;
        uint32_t enable_time_ms;
    };

    std::array<LoadSwitch_s, NUM_LOAD_SWITCHES> _all_load_switches = {};
    bool _is_config_valid = true;

    bool _isValidIndex(uint8_t index) const { return index < NUM_LOAD_SWITCHES; }

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
};

using LoadSwitchInterfaceInstance = etl::singleton<LoadSwitchInterface>;

#endif // LDSW_INTERFACE_H
