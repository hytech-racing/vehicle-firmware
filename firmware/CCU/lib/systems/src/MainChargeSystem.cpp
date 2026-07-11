#include "MainChargeSystem.h"
#include <algorithm>

void MainChargeSystem::init(unsigned long init_millis)
{
    _init_millis = init_millis;
    _startup_complete = false;
}

void MainChargeSystem::calculate_charge_current(float max_pack_voltage, float cell_cutoff_voltage, float dial_percent, unsigned long curr_millis)
{
    // Get battery data from ACU
    const auto& acu_data = ACUInterfaceInstance::instance().get_latest_data();
    float max_cell_voltage = acu_data.high_voltage; // the highest voltage in any of the cells
    float total_pack_voltage = acu_data.pack_voltage; // the total voltage in the pack
    auto current_state = ChargerStateMachineInstance::instance().get_state();

    // Check safety conditions first
    if (!_is_safety_conditions_valid())
    {
        _charge_data.calculated_charge_current = 0.0F;
        return;
    }

    // Check if voltage limits reached/exceeded
    bool is_voltage_limit_exceeded = (max_cell_voltage >= cell_cutoff_voltage) || (total_pack_voltage > max_pack_voltage);

    if (is_voltage_limit_exceeded)
    {
        _charge_data.calculated_charge_current = 0.0F;
        return;
    }

    // Determine requested current based on state
    float requested_current = _get_current_for_state(current_state) * (dial_percent / 100.0F);

    // Apply safety limits
    _charge_data.calculated_charge_current = _apply_current_limits(current_state, requested_current, curr_millis);
}

bool MainChargeSystem::_is_safety_conditions_valid()
{
    // Check BRB on charge cart (shutdown F is after BRB)
    bool is_shutdown_low = false;
    bool is_acu_shutdown_low = false;
    bool is_ccu_shutdown_low = false;

    is_shutdown_low = !ADCInterfaceInstance::instance().read_shdn_F_voltage();

    /**
     * Check ACU state: acu_state comes from the bms_status message. If shutdown is low on ACU (HVP is unplugged), acu_state = 3.
     * If acu_state = 2, we should/are safe to be charging
     * ACU States for Reference: STARTUP = 0, ACTIVE = 1, CHARGING = 2, FAULTED = 3, WELDED = 4, WELDCHECK = 5
     */
    is_acu_shutdown_low = ACUInterfaceInstance::instance().get_latest_data().acu_state != ACUState_e::CHARGING; //NOLINT

    // Check for error state from state machine
    is_ccu_shutdown_low = !(ChargerStateMachineInstance::instance().get_state() == ChargerState_e::CHARGING_120 ||
                                ChargerStateMachineInstance::instance().get_state() == ChargerState_e::CHARGING_240);

    if (is_shutdown_low || is_acu_shutdown_low || is_ccu_shutdown_low)
    {
        return false;
    }

    return true;
}

float MainChargeSystem::_apply_current_limits(ChargerState_e state, float requested_current, unsigned long curr_millis)
{
    float limited_current = requested_current;

    const auto& acu_data = ACUInterfaceInstance::instance().get_latest_data();
    float curr_max_cell_temp = acu_data.max_cell_temp;
    float curr_max_board_temp = acu_data.max_board_temp;


    // Cell temp derate (stop completely at 45°C, 40°C start derating)
    float cell_temp_factor = _calculate_cell_temp_derate_factor(curr_max_cell_temp);

    // Board temper derate (stop completely 60°C, 50°C start derating)
    float board_temp_factor = _calculate_board_temp_derate_factor(curr_max_board_temp);

    // Startup derate
    float startup_delay_factor = _startup_derate_factor(curr_millis);

    // Apply all derating factors
    limited_current *= startup_delay_factor;
    limited_current *= std::min(cell_temp_factor, board_temp_factor);

    return std::max(0.0F,limited_current);
}

float MainChargeSystem::_get_current_for_state(ChargerState_e state)
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

float MainChargeSystem::_calculate_cell_temp_derate_factor(float curr_temp)
{
    if (curr_temp < _charge_system_parameters.thresholds.cell_temp_derate_thresh) {return 1.0F;}  // No derating
    if (curr_temp >= _charge_system_parameters.max_cell_cutoff_temp_celcius) {return 0.0F;}   // Stop charging immediatly if above max threshold

    return 1.0F - std::max(std::min(((curr_temp - _charge_system_parameters.thresholds.cell_temp_derate_thresh) / (_charge_system_parameters.max_cell_cutoff_temp_celcius - _charge_system_parameters.thresholds.cell_temp_derate_thresh)), 1.0F), 0.0F);
}

float MainChargeSystem::_calculate_board_temp_derate_factor(float curr_temp)
{
    if (curr_temp < _charge_system_parameters.thresholds.board_temp_derate_thresh) {return 1.0F;}  // No derating
    if (curr_temp >= _charge_system_parameters.max_board_cutoff_temp_celcius) {return 0.0F;}   // Stop charging immediatly if above max threshold

    return 1.0F - std::max(std::min(((curr_temp - _charge_system_parameters.thresholds.board_temp_derate_thresh) / (_charge_system_parameters.max_board_cutoff_temp_celcius - _charge_system_parameters.thresholds.board_temp_derate_thresh)), 1.0F), 0.0F);
}

float MainChargeSystem::_startup_derate_factor(unsigned long current_millis)
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