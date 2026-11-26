#include "MainChargeSystem.hpp"

#include <algorithm>

void MainChargeSystem::init(unsigned long init_millis)
{
    _init_millis = init_millis;
    _startup_complete = false;
    _is_termination_timing = false;
    _charge_data.is_charge_complete = false;
}

void MainChargeSystem::calculateChargeCurrent(
    float max_pack_voltage,
    float cell_cutoff_voltage,
    float dial_percent,
    float charger_output_current_A,
    bool is_charger_faulted,
    unsigned long curr_millis)
{
    // Get battery data from ACU
    const auto& acu_data = ACUInterfaceInstance::instance().getLatestData();
    float max_cell_voltage = acu_data.high_voltage;   // the highest voltage in any of the cells
    float total_pack_voltage = acu_data.pack_voltage; // the total voltage in the pack
    auto current_state = ChargerStateMachineInstance::instance().getState();

    // CV setpoint never exceeds the hard pack limit
    _charge_data.target_pack_voltage = std::min(_charge_system_parameters.cccv_params.target_pack_voltage, max_pack_voltage);

    // Check safety conditions first
    if (!_isSafetyConditionsValid() || is_charger_faulted)
    {
        _is_termination_timing = false;
        _stopCharging();
        return;
    }

    // Hard voltage limits (safety backstop, the charger's CV setpoint should keep us below these)
    bool is_voltage_limit_exceeded = (max_cell_voltage >= cell_cutoff_voltage) || (total_pack_voltage > max_pack_voltage);

    if (is_voltage_limit_exceeded)
    {
        _stopCharging();
        return;
    }

    // CV termination: stop once current has tapered off at the target voltage
    _checkChargeComplete(total_pack_voltage, charger_output_current_A, curr_millis);

    if (_charge_data.is_charge_complete)
    {
        _stopCharging();
        return;
    }

    // CC limit: requested current based on state, scaled by the dial
    float requested_current = _getCurrentForState(current_state) * (dial_percent / 100.0F);

    // Apply safety limits
    _charge_data.calculated_charge_current = _applyCurrentLimits(current_state, requested_current, curr_millis);
    _charge_data.is_charging_enabled = _charge_data.calculated_charge_current > 0.0F;
}

void MainChargeSystem::_checkChargeComplete(float total_pack_voltage, float charger_output_current_A, unsigned long curr_millis)
{
    const auto& cccv_params = _charge_system_parameters.cccv_params;

    bool is_in_cv = total_pack_voltage >= (_charge_data.target_pack_voltage - cccv_params.cv_detect_band_voltage);
    bool is_current_below_termination = charger_output_current_A < cccv_params.termination_current_amp;

    if (!(is_in_cv && is_current_below_termination))
    {
        _is_termination_timing = false;
        return;
    }

    if (!_is_termination_timing)
    {
        _is_termination_timing = true;
        _termination_start_millis = curr_millis;
        return;
    }

    if ((curr_millis - _termination_start_millis) >= cccv_params.termination_time_ms)
    {
        _charge_data.is_charge_complete = true;
    }
}

void MainChargeSystem::_stopCharging()
{
    _charge_data.calculated_charge_current = 0.0F;
    _charge_data.is_charging_enabled = false;
}

bool MainChargeSystem::_isSafetyConditionsValid()
{
    // Check BRB on charge cart (shutdown F is after BRB)
    bool is_shutdown_low = false;
    bool is_acu_shutdown_low = false;
    bool is_ccu_shutdown_low = false;

    is_shutdown_low = !ADCInterfaceInstance::instance().isShutdownFHigh();

    /**
     * Check ACU state: acu_state comes from the bms_status message. If shutdown is low on ACU (HVP is unplugged), acu_state = 3.
     * If acu_state = 2, we should/are safe to be charging
     * ACU States for Reference: STARTUP = 0, ACTIVE = 1, CHARGING = 2, FAULTED = 3, WELDED = 4, WELDCHECK = 5
     */
    is_acu_shutdown_low = ACUInterfaceInstance::instance().getLatestData().acu_state != ACUState_e::CHARGING; // NOLINT

    // Check for error state from state machine
    is_ccu_shutdown_low =
        !(ChargerStateMachineInstance::instance().getState() == ChargerState_e::CHARGING_120
          || ChargerStateMachineInstance::instance().getState() == ChargerState_e::CHARGING_240);

    if (is_shutdown_low || is_acu_shutdown_low || is_ccu_shutdown_low)
    {
        return false;
    }

    return true;
}

float MainChargeSystem::_applyCurrentLimits(ChargerState_e state, float requested_current, unsigned long curr_millis)
{
    float limited_current = requested_current;

    const auto& acu_data = ACUInterfaceInstance::instance().getLatestData();
    float curr_max_cell_temp = acu_data.max_cell_temp;
    float curr_max_board_temp = acu_data.max_board_temp;


    // Cell temp derate (stop completely at 45°C, 40°C start derating)
    float cell_temp_factor = _calculateCellTempDerateFactor(curr_max_cell_temp);

    // Board temper derate (stop completely 60°C, 50°C start derating)
    float board_temp_factor = _calculateBoardTempDerateFactor(curr_max_board_temp);

    // Startup derate
    float startup_delay_factor = _startupDerateFactor(curr_millis);

    // Apply all derating factors
    limited_current *= startup_delay_factor;
    limited_current *= std::min(cell_temp_factor, board_temp_factor);

    return std::max(0.0F, limited_current);
}

float MainChargeSystem::_getCurrentForState(ChargerState_e state)
{
    switch (state)
    {
    case ChargerState_e::CHARGING_120:
    {
        // 120V Charging, max set at 3.5 amps
        return _charge_system_parameters.max_120V_current_amp;
    }
    case ChargerState_e::CHARGING_240:
    {
        // 240V charging, max set at 11 amps
        return _charge_system_parameters.max_240V_current_amp;
    }
    default:
    {
        return 0.0F;
    }
    }
}

float MainChargeSystem::_calculateCellTempDerateFactor(float curr_temp)
{
    if (curr_temp < _charge_system_parameters.thresholds.cell_temp_derate_thresh)
    {
        return 1.0F;
    } // No derating
    if (curr_temp >= _charge_system_parameters.max_cell_cutoff_temp_celcius)
    {
        return 0.0F;
    } // Stop charging immediatly if above max threshold

    return 1.0F
           - std::max(
               std::min(
                   ((curr_temp - _charge_system_parameters.thresholds.cell_temp_derate_thresh)
                    / (_charge_system_parameters.max_cell_cutoff_temp_celcius - _charge_system_parameters.thresholds.cell_temp_derate_thresh)),
                   1.0F),
               0.0F);
}

float MainChargeSystem::_calculateBoardTempDerateFactor(float curr_temp)
{
    if (curr_temp < _charge_system_parameters.thresholds.board_temp_derate_thresh)
    {
        return 1.0F;
    } // No derating
    if (curr_temp >= _charge_system_parameters.max_board_cutoff_temp_celcius)
    {
        return 0.0F;
    } // Stop charging immediatly if above max threshold

    return 1.0F
           - std::max(
               std::min(
                   ((curr_temp - _charge_system_parameters.thresholds.board_temp_derate_thresh)
                    / (_charge_system_parameters.max_board_cutoff_temp_celcius - _charge_system_parameters.thresholds.board_temp_derate_thresh)),
                   1.0F),
               0.0F);
}

float MainChargeSystem::_startupDerateFactor(unsigned long current_millis)
{
    const unsigned long startup_delay = _charge_system_parameters.configs.startup_delay_ms;
    const unsigned long elapsed_ms = current_millis - _init_millis;
    if (_startup_complete || (elapsed_ms >= startup_delay))
    {
        _startup_complete = true;
        return 1.0F;
    }

    // Linear ramp from 0.0 to 1.0 over startup_delay period
    return static_cast<float>(elapsed_ms) / static_cast<float>(startup_delay);
}
