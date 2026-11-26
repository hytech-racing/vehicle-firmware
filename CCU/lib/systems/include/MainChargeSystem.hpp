#ifndef MAINCHARGE_H
#define MAINCHARGE_H

/* External Includes  */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ADCInterface.hpp"
#include "Level2Interface.hpp"
#include "SystemTimeInterface.h"
#include "WatchdogInterface.hpp"

/* Local System Includes */
#include "ChargerStateMachine.hpp"
#include "Level2System.hpp"

#include <Arduino.h>


enum BalancingState_e
{
    NOT_BALANCING = 0,
    BALANCING
};

namespace charge_system_default_parameters
{
    constexpr const unsigned long STARTUP_DELAY_MS = 20000; // 20 seconds

    constexpr const float CELL_TEMP_DERATE_THRESH = 40.0F;  // celsius
    constexpr const float BOARD_TEMP_DERATE_THRESH = 50.0F; // celsius
}; // namespace charge_system_default_parameters

struct ChargeSystemData_s
{
    float calculated_charge_current; // CC limit sent to the charger
    float target_pack_voltage;       // CV setpoint sent to the charger
    bool is_charging_enabled;        // false -> charger is commanded to stop
    bool is_charge_complete;         // latched once CV termination criteria are met
    ChargerState_e current_charger_state;
};

/**
 * @brief CC/CV charge profile. The Elcon charger performs CC/CV itself given a voltage setpoint (CV) and a current limit (CC).
 */
struct ChargeSystemCCCVParams_s
{
    float target_pack_voltage;     // CV setpoint, pack volts
    float cv_detect_band_voltage;  // pack is considered "in CV" when within this many volts of the target
    float termination_current_amp; // charge is complete once output current stays below this while in CV
    uint32_t termination_time_ms;  // ... for at least this long
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
    ChargeSystemCCCVParams_s cccv_params;
    float max_120V_current_amp;
    float max_240V_current_amp;
    float max_cell_cutoff_temp_celcius;
    float max_board_cutoff_temp_celcius;
};

class MainChargeSystem
{
  public:
    MainChargeSystem(
        float max_120V_current_amp,
        float max_240V_current_amp,
        float max_cell_cutoff_temp_celcius,
        float max_board_cutoff_temp_celcius,
        ChargeSystemCCCVParams_s cccv_params,
        ChargeSystemThresholds_s thresholds =
            {
                .cell_temp_derate_thresh = charge_system_default_parameters::CELL_TEMP_DERATE_THRESH,
                .board_temp_derate_thresh = charge_system_default_parameters::BOARD_TEMP_DERATE_THRESH,
            },
        ChargeSystemConfigs_s configs = {.startup_delay_ms = charge_system_default_parameters::STARTUP_DELAY_MS})
        : _charge_system_parameters{
              thresholds,
              configs,
              cccv_params,
              max_120V_current_amp,
              max_240V_current_amp,
              max_cell_cutoff_temp_celcius,
              max_board_cutoff_temp_celcius}
    {
        _charge_data.calculated_charge_current = 0.0F;
        _charge_data.target_pack_voltage = cccv_params.target_pack_voltage;
        _charge_data.is_charging_enabled = false;
        _charge_data.is_charge_complete = false;
    }

    /**
     * @brief Resets the startup ramp and clears the charge-complete latch
     */
    void init(unsigned long init_millis);

    /**
     * @brief Calculate the CC limit, CV setpoint, and charge enable based on cell state and charger state
     * @param max_pack_voltage Hard pack voltage limit; charging stops above this (safety, not the CV target)
     * @param cell_cutoff_voltage Hard cell voltage limit; charging stops at or above this (safety)
     * @param dial_percent Rotary dial percent (0-100) scaling the CC current
     * @param charger_output_current_A Output current reported by the charger, used for CV termination
     * @param is_charger_faulted True if the charger reports a fault in its status flags
     * @param curr_millis Current time in millis
     */
    void calculateChargeCurrent(
        float max_pack_voltage,
        float cell_cutoff_voltage,
        float dial_percent,
        float charger_output_current_A,
        bool is_charger_faulted,
        unsigned long curr_millis);

    /**
     * @brief CV setpoint (pack volts) to send to the charger
     */
    float getTargetPackVoltage() const { return _charge_data.target_pack_voltage; }

    /**
     * @brief Whether the charger should be commanded to charge (false -> stop)
     */
    bool isChargingEnabled() const { return _charge_data.is_charging_enabled; }

    /**
     * @brief Whether the CV termination criteria have been met (latched until init())
     */
    bool isChargeComplete() const { return _charge_data.is_charge_complete; }

    /**
     * @brief Determine if cell balancing should be enabled
     * @param voltage_delta_threshold Voltage difference to trigger balancing (default 0.05V)
     * @param min_balance_voltage Minimum cell voltage to start balancing (default 3.0V)
     * @return true if balancing should be enabled
     */
    bool determineBalancingState(ChargerState_e current_state, float voltage_delta_threshold = 0.02F, float min_balance_voltage = 3.0F);

    /**
     * @brief Get calculated charge current
     */
    float getChargeCurrent() const { return _charge_data.calculated_charge_current; }

    /**
     * @brief Get the current charge system data
     */
    const ChargeSystemData_s& getChargeData() const { return _charge_data; }

  private:
    /**
     * @brief timestamp captured in init()
     */
    unsigned long _init_millis = 0;

    /**
     * @brief used to latch after startup current ramp is complete
     */
    bool _startup_complete = false;

    /**
     * @brief timestamp when output current first dropped below the termination current while in CV
     */
    unsigned long _termination_start_millis = 0;
    bool _is_termination_timing = false;

    const ChargeSystemParams_s _charge_system_parameters = {};
    ChargeSystemData_s _charge_data;

    /**
     * @brief Check if it's safe to charge based on safety systems
     */
    bool _isSafetyConditionsValid();

    /**
     * @brief Apply current limiting, includes temperature and inital startup ramp-up
     */
    float _applyCurrentLimits(ChargerState_e state, float requested_current, unsigned long curr_millis);

    /**
     * @brief Get appropriate current based on charger state
     */
    float _getCurrentForState(ChargerState_e state);

    /**
     * @brief Latches charge complete once the pack is in CV and output current has stayed below the termination current long enough
     */
    void _checkChargeComplete(float total_pack_voltage, float charger_output_current_A, unsigned long curr_millis);

    /**
     * @brief Sets current to 0 and disables charging
     */
    void _stopCharging();

    /**
     * @brief
     */
    float _calculateCellTempDerateFactor(float curr_temp);

    /**
     * @brief
     */
    float _calculateBoardTempDerateFactor(float curr_temp);

    /**
     *
     */
    float _startupDerateFactor(unsigned long current_millis);
};

using MainChargeSystemInstance = etl::singleton<MainChargeSystem>;

#endif
