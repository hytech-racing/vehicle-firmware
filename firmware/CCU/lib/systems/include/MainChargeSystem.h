#ifndef MAINCHARGE_H
#define MAINCHARGE_H

/* External Includes  */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "ACUInterface.h"
#include "ADCInterface.h"
#include "Level2Interface.h"
#include "SystemTimeInterface.h"
#include "WatchdogInterface.h"

/* Local System Includes */
#include "ChargerStateMachine.h"
#include "Level2System.h"

#ifdef TEENSY_OPT_SMALLEST_CODE
#include <Arduino.h>
#endif

#ifdef TESTING_SYSTEMS
#include "mockArduino.h"
#endif

enum BalancingState_e
{
    NOT_BALANCING = 0,
    BALANCING
};

namespace charge_system_default_parameters
{
    constexpr const unsigned long STARTUP_DELAY_MS = 20000; // 20 seconds

    constexpr const float CELL_TEMP_DERATE_THRESH = 40.0F; // celsius
    constexpr const float BOARD_TEMP_DERATE_THRESH = 50.0F; // celsius
};

struct ChargeSystemData_s
{
    float calculated_charge_current;
    ChargerState_e current_charger_state;
};

struct ChargeSystemThresholds_s
{
    float cell_temp_derate_thresh;
    float board_temp_derate_thresh;
    float cell_voltage_derate_lower_thresh;
    float cell_voltage_derate_upper_thresh;
};

struct ChargeSystemConfigs_s
{
    uint32_t startup_delay_ms;
};

struct ChargeSystemParams_s
{
    ChargeSystemThresholds_s thresholds;
    ChargeSystemConfigs_s configs;
    float max_120V_current_amp;
    float max_240V_current_amp;
    float max_cell_cutoff_temp_celcius;
    float max_board_cutoff_temp_celcius;
};

class MainChargeSystem {
public:
    MainChargeSystem(float max_120V_current_amp,
                    float max_240V_current_amp,
                    float max_cell_cutoff_temp_celcius,
                    float max_board_cutoff_temp_celcius,
                    ChargeSystemThresholds_s thresholds =
                    {
                        .cell_temp_derate_thresh = charge_system_default_parameters::CELL_TEMP_DERATE_THRESH,
                        .board_temp_derate_thresh = charge_system_default_parameters::BOARD_TEMP_DERATE_THRESH,
                    },
                    ChargeSystemConfigs_s configs =
                    {
                        .startup_delay_ms = charge_system_default_parameters::STARTUP_DELAY_MS
                    }
    ): _charge_system_parameters {
        thresholds,
        configs,
        max_120V_current_amp,
        max_240V_current_amp,
        max_cell_cutoff_temp_celcius,
        max_board_cutoff_temp_celcius
    }
    {
        _charge_data.calculated_charge_current = 0.0F;
    }

    /**
     *
     */
    void init(unsigned long init_millis);

    /**
     * @brief Calculate and set the charge current based on cell state and charger state
     * @param max_pack_voltage Maximum allowable pack voltage
     * @param cell_cutoff_voltage Voltage to stop charging at
     * @param charger_current_max Maximum current (will be scaled based on 120V vs 240V state)
     * @param curr_time Current time in millis
     */
    void calculate_charge_current(float max_pack_voltage, float cell_cutoff_voltage, float dial_percent, unsigned long curr_millis);

    /**
     * @brief Determine if cell balancing should be enabled
     * @param voltage_delta_threshold Voltage difference to trigger balancing (default 0.05V)
     * @param min_balance_voltage Minimum cell voltage to start balancing (default 3.0V)
     * @return true if balancing should be enabled
     */
    bool determine_balancing_state(ChargerState_e current_state, float voltage_delta_threshold = 0.02F, float min_balance_voltage = 3.0F );

    /**
     * @brief Get calculated charge current
     */
    float get_charge_current() const { return _charge_data.calculated_charge_current; }

    /**
     * @brief Get the current charge system data
     */
    const ChargeSystemData_s& get_charge_data() const { return _charge_data; }

private:

    /**
     * @brief timestamp captured in init()
     */
    unsigned long _init_millis = 0;

    /**
     * @brief used to latch after startup current ramp is complete
     */
    bool _startup_complete = false;

    const ChargeSystemParams_s _charge_system_parameters = {};
    ChargeSystemData_s _charge_data;

    /**
     * @brief Check if it's safe to charge based on safety systems
     */
    bool _is_safety_conditions_valid();

    /**
     * @brief Apply current limiting, includes temperature and inital startup ramp-up
     */
    float _apply_current_limits(ChargerState_e state, float requested_current, unsigned long curr_millis);

    /**
     * @brief Get appropriate current based on charger state
     */
    float _get_current_for_state(ChargerState_e state);

    /**
     * @brief
     */
    float _calculate_cell_temp_derate_factor(float curr_temp);

    /**
     * @brief
     */
    float _calculate_board_temp_derate_factor(float curr_temp);

    /**
     *
     */
    float _startup_derate_factor(unsigned long current_millis);

};

using MainChargeSystemInstance = etl::singleton<MainChargeSystem>;

#endif