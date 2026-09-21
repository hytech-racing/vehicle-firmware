#include "TorqueControllerMux.hpp"


template <size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::evaluateTCMux(ControllerMode_e requested_controller_mode,
                                                                        TorqueLimit_e torque_limit_map_val,
                                                                        const VCRData_s &input_state
)
{

    const DrivetrainCommand_s EMPTY_COMMAND = {
        .control_mode = DrivetrainControlMode_e::TORQUE,
        .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
        .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}
    };

    // Check if requested mode is valid
    bool is_requested_mode_supported = (requested_controller_mode == ControllerMode_e::MODE_0)
                                    || (requested_controller_mode == ControllerMode_e::MODE_1)
                                    || (requested_controller_mode == ControllerMode_e::MODE_3)
                                    || (requested_controller_mode == ControllerMode_e::MODE_4);

    if (!is_requested_mode_supported)
    {
        _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS;
        return EMPTY_COMMAND;
    }

    /**
     * @note Here we are going to track...
     *  1) Which mode is the driver/dashboard currently requesting
     *  2) What mode the mux is actually running right now
     *
     * We want to keep track for these reasons...
     *  1) Want to double check both the active and requested mode's slots in _controller_evals actually
     *     holds a bound function before calling either.
     *  2) We need to perform safety checks before switching modes, so we need to keep track of requested and current mode.
    */
    int requested_mode_index = static_cast<int>(requested_controller_mode);
    int active_mode_index = static_cast<int>(_curr_tc_mux_status.active_controller_mode);

    /**
     * @brief Defensive guard against a construction-time mistake.
     *
     * _controller_evals is a fixed-size std::array<std::function<...>, N>.
     *
     * Each slot is either "empty" (holds no callable) or "bound" (wraps a real function/lambda).
     * We are checking whether a real function was ever assigned into this slot in the first place.
    */
    if ((!_controller_evals[active_mode_index]) || (!_controller_evals[requested_mode_index]))
    {
        _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER;
        return EMPTY_COMMAND;
    }

    // Evaluate the currently-active mode's controller (not the requested one yet), this becomes the default
    // output for this tick unless the mode-switch safety check approves switching to the requested mode instead
    DrivetrainCommand_s active_mode_evaluate_output = _controller_evals[active_mode_index](input_state, sys_time::hal_millis());


    bool is_mode_change_requested = requested_controller_mode != _curr_tc_mux_status.active_controller_mode;

    if (is_mode_change_requested)
    {
        DrivetrainCommand_s requested_mode_evaluate_output = _controller_evals[requested_mode_index](input_state, sys_time::hal_millis());

        bool can_switch_controller = _canSwitchController(input_state.system_data.drivetrain_data,
                                                        active_mode_evaluate_output,
                                                        requested_mode_evaluate_output
        );

        if (can_switch_controller)
        {
            _curr_tc_mux_status.active_controller_mode = requested_controller_mode;
            active_mode_index = requested_mode_index;
            active_mode_evaluate_output = requested_mode_evaluate_output;
        }
    }

    if (!_mux_bypass_limits[active_mode_index])
    {
        // no bypass limit branch -> apply limits

        _curr_tc_mux_status.active_torque_limit_enum = torque_limit_map_val;

        // Desired speed is 0 (braking) -> allow regen -> apply limits so that the pack voltage doesn't spike too high
        if (active_mode_evaluate_output.desired_speeds.FL == 0.0f &&
            active_mode_evaluate_output.desired_speeds.FR == 0.0f &&
            active_mode_evaluate_output.desired_speeds.RL == 0.0f &&
            active_mode_evaluate_output.desired_speeds.RR == 0.0f
        )
        {
            active_mode_evaluate_output = _applyRegenLimit(active_mode_evaluate_output, input_state.system_data.drivetrain_data, input_state.interface_data.stamped_acu_core_data.acu_data);
        }

        if (active_mode_evaluate_output.control_mode == DrivetrainControlMode_e::TORQUE)
        {
            active_mode_evaluate_output = _applyTorqueLimit(active_mode_evaluate_output, _torque_limit_map[torque_limit_map_val]);
            _curr_tc_mux_status.active_torque_limit_value = _torque_limit_map[torque_limit_map_val];
        }
        else if (active_mode_evaluate_output.control_mode == DrivetrainControlMode_e::SPEED) // this prolly needs to be fixed lmao
        {
            active_mode_evaluate_output = _applySpeedLimit(active_mode_evaluate_output, _torque_limit_map[torque_limit_map_val]);
            _curr_tc_mux_status.active_torque_limit_value = _torque_limit_map[torque_limit_map_val];
        }

        // Apply power limit when accelerating
        if (active_mode_evaluate_output.desired_speeds.FL != 0.0f ||
            active_mode_evaluate_output.desired_speeds.FR != 0.0f ||
            active_mode_evaluate_output.desired_speeds.RL != 0.0f ||
            active_mode_evaluate_output.desired_speeds.RR != 0.0f
        )
        {
            active_mode_evaluate_output = _applyTorqueControlPowerLimit(active_mode_evaluate_output, input_state.system_data.drivetrain_data, _params.max_power_limit_watts, _torque_limit_map[torque_limit_map_val]);
        }

        _curr_tc_mux_status.output_is_bypassing_limits = false;
    }
    else
    {
        // any mode other than mode 0 = no torque, regen, or power limiting
        _curr_tc_mux_status.active_torque_limit_enum = TorqueLimit_e::TCMUX_FULL_TORQUE;
        _curr_tc_mux_status.active_torque_limit_value= dti_motor_params::MOTOR_MAX_TORQUE_NM;
        _curr_tc_mux_status.output_is_bypassing_limits = true;
    }

    return active_mode_evaluate_output;
}

template <size_t num_controllers>
bool TorqueControllerMux<num_controllers>::_canSwitchController(DrivetrainDynamicReport_s active_drivetrain_data,
                                                                DrivetrainCommand_s active_mode_evaluate_output,
                                                                DrivetrainCommand_s requested_mode_evaluate_output
)
{
    auto active_measured_speeds_array = active_drivetrain_data.measured_speeds.as_array();
    auto active_mode_eval_desired_torques_array = active_mode_evaluate_output.desired_torques.as_array();
    auto requested_mode_eval_desired_torques_array = requested_mode_evaluate_output.desired_torques.as_array();

    for (size_t i = 0; i < _params.num_motors; i++)
    {
        bool is_speed_preventing_mode_change =
            std::fabs(active_measured_speeds_array[i] * physical_motor_scales::RPM_TO_METERS_PER_SECOND) > _params.max_speed_during_mode_change;

        bool is_torque_delta_preventing_mode_change =
            std::fabs(requested_mode_eval_desired_torques_array[i] - active_mode_eval_desired_torques_array[i]) > _params.max_torque_delta_during_mode_change;

        if (is_speed_preventing_mode_change)
        {
            _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH;
            return false;
        }

        if (is_torque_delta_preventing_mode_change)
        {
            _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH;
            return false;
        }
    }

    _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::NO_ERROR;
    return true;
}

template <size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applyTorqueLimit(const DrivetrainCommand_s& dt_command, float max_avg_torque_limit)
{
    DrivetrainCommand_s command_output = dt_command;

    float avg_torque = 0.0f;
    auto desired_torques_array = command_output.desired_torques.as_array();

    for (size_t i = 0; i < desired_torques_array.size(); i++)
    {
        avg_torque += std::abs(desired_torques_array[i]);
    }

    avg_torque /= _params.num_motors;

    if (avg_torque > max_avg_torque_limit)
    {
        float scale = avg_torque / max_avg_torque_limit;

        command_output.desired_torques.FL = command_output.desired_torques.FL / scale;
        command_output.desired_torques.FR = command_output.desired_torques.FR / scale;
        command_output.desired_torques.RL = command_output.desired_torques.RL / scale;
        command_output.desired_torques.RR = command_output.desired_torques.RR / scale;
    }

    return command_output;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applyTorqueControlPowerLimit(const DrivetrainCommand_s &dt_command,
                                                                                        const DrivetrainDynamicReport_s &dynamic_report,
                                                                                        float mech_power_limit,
                                                                                        float max_torque_nm
)
{
    DrivetrainCommand_s command_output = dt_command;

    // Net desired mechanical power (T * omega) using DESIRED torque and MEASURED speed.
    float net_power_mag = command_output.desired_torques.FL * (dynamic_report.measured_speeds.FL * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                        + command_output.desired_torques.FR * (dynamic_report.measured_speeds.FR * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                        + command_output.desired_torques.RL * (dynamic_report.measured_speeds.RL * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                        + command_output.desired_torques.RR * (dynamic_report.measured_speeds.RR * physical_motor_scales::RPM_TO_RAD_PER_SECOND);

    auto scale_desired_torques = [](float desired_wheel_torque, float current_wheel_rpm, float net_power_mag, float power_limit, float max_torque_val) -> float
    {
        if (fabs(net_power_mag) < 1e-3f) // 1e-3f is an arbitary number
        {
            return 0.0;   // avoid divide-by-zero, fail safe since this should never happen
        }

        float current_wheel_omega = current_wheel_rpm * physical_motor_scales::RPM_TO_RAD_PER_SECOND;
        float current_wheel_power = fabs(desired_wheel_torque * current_wheel_omega);

        float desired_wheel_power_percentage = current_wheel_power / fabs(net_power_mag);
        float corner_power = desired_wheel_power_percentage * power_limit;

        float result = fabs(corner_power / current_wheel_omega);
        result = std::max(0.0f, std::min(result, max_torque_val));

        return (desired_wheel_torque < 0.0f) ? -result : result;
    };

    if (fabs(net_power_mag) > mech_power_limit)
    {
        command_output.desired_torques.FL = scale_desired_torques(command_output.desired_torques.FL, dynamic_report.measured_speeds.FL, net_power_mag, mech_power_limit, max_torque_nm);
        command_output.desired_torques.FR = scale_desired_torques(command_output.desired_torques.FR, dynamic_report.measured_speeds.FR, net_power_mag, mech_power_limit, max_torque_nm);
        command_output.desired_torques.RL = scale_desired_torques(command_output.desired_torques.RL, dynamic_report.measured_speeds.RL, net_power_mag, mech_power_limit, max_torque_nm);
        command_output.desired_torques.RR = scale_desired_torques(command_output.desired_torques.RR, dynamic_report.measured_speeds.RR, net_power_mag, mech_power_limit, max_torque_nm);
    }

    return command_output;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applySpeedControlPowerLimit(const DrivetrainCommand_s &dt_command,
                                                                                        const DrivetrainDynamicReport_s &dynamic_report,
                                                                                        float mech_power_limit,
                                                                                        float max_speed_rpm
)
{
    DrivetrainCommand_s command_output = dt_command;

    // use MEASURED torque and MEASURED speed to compute actual current mechanical power
    float net_measured_power = dynamic_report.measured_torques.FL * (dynamic_report.measured_speeds.FL * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                            + dynamic_report.measured_torques.FR * (dynamic_report.measured_speeds.FR * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                            + dynamic_report.measured_torques.RL * (dynamic_report.measured_speeds.RL * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                            + dynamic_report.measured_torques.RR * (dynamic_report.measured_speeds.RR * physical_motor_scales::RPM_TO_RAD_PER_SECOND);

    /**
     * @warning This is an approximation, not exact like the torque-mode correction. It assumes measured torque stays roughly constant as the speed target changes
     *          which isn't true since DTI's internal speed loop will re-derive torque at the new target
    */
    auto scale_desired_speeds = [](float desired_wheel_speed_rpm, float measured_wheel_torque, float net_power_mag, float power_limit, float max_speed_val) -> float
    {
        if (fabs(net_power_mag) < 1e-3f)
        {
            return 0.0f;   // no meaningful torque/power to base a correction on, fail safe
        }

        float current_wheel_power = fabs(measured_wheel_torque * desired_wheel_speed_rpm * physical_motor_scales::RPM_TO_RAD_PER_SECOND);

        float desired_wheel_power_percentage = current_wheel_power / fabs(net_power_mag);
        float corner_power = desired_wheel_power_percentage * power_limit;

        float corrected_omega = corner_power / fabs(measured_wheel_torque);
        float result = corrected_omega / physical_motor_scales::RPM_TO_RAD_PER_SECOND;
        result = std::max(0.0f, std::min(result, max_speed_val));

        return (desired_wheel_speed_rpm < 0.0f) ? -result : result;
    };

    if (fabs(net_measured_power) > mech_power_limit)
    {
        command_output.desired_speeds.FL = scale_desired_speeds(command_output.desired_speeds.FL, dynamic_report.measured_torques.FL, net_measured_power, mech_power_limit, max_speed_rpm);
        command_output.desired_speeds.FR = scale_desired_speeds(command_output.desired_speeds.FR, dynamic_report.measured_torques.FR, net_measured_power, mech_power_limit, max_speed_rpm);
        command_output.desired_speeds.RL = scale_desired_speeds(command_output.desired_speeds.RL, dynamic_report.measured_torques.RL, net_measured_power, mech_power_limit, max_speed_rpm);
        command_output.desired_speeds.RR = scale_desired_speeds(command_output.desired_speeds.RR, dynamic_report.measured_torques.RR, net_measured_power, mech_power_limit, max_speed_rpm);
    }

    return command_output;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applyRegenLimit(const DrivetrainCommand_s &desired_controller_out,
                                                                        const DrivetrainDynamicReport_s &dynamic_report,
                                                                        const ACUCoreData_s acu_data
)
{
    DrivetrainCommand_s command_output = desired_controller_out;

    // EV.3.3.3: "powertrain must not regenerate energy when vehicle speed is between 0 and 5 km/hr"
    const float no_regen_limit_kph = 10.0;
    const float full_regen_limit_kph = 5.0;

    // Based on DCIR
    const volt start_regen_voltage_limit = 520.0;
    const volt max_regen_voltage_limit = 530.0;

    // Mechanical power limit
    const watt start_regen_power_limit_watt = 30000.0f;
    const watt max_regen_power_limit_watt = 50000.0f;

    // Current motor and inverter behavior
    DrivetrainDynamicReport_s dt_dynamic_data = dynamic_report;
    auto current_speeds = dt_dynamic_data.measured_speeds.as_array();
    auto current_inverter_voltage = dt_dynamic_data.measured_hv_bus_voltage.as_array();
    auto command_speeds = desired_controller_out.desired_speeds.as_array();

    float max_wheel_speed = 0.0;
    volt max_inverter_voltage = 0.0;
    bool are_all_wheels_regen = true; // true when all wheels are targeting speeds below the current wheel speed

    for (size_t i = 0; i < _params.num_motors; i++)
    {
        max_wheel_speed = std::max(max_wheel_speed, static_cast<float>(fabs(current_speeds[i]) * physical_motor_scales::RPM_TO_KILOMETERS_PER_HOUR));
        max_inverter_voltage = std::max(max_inverter_voltage, current_inverter_voltage[i]);
        are_all_wheels_regen &= (command_speeds[i] < static_cast<float>(fabs(current_speeds[i])) || command_speeds[i] == 0);
    }

    // begin limiting regen at no_regen_limit_kph and completely limit regen at full_regen_limit_kph; using linear ramp
    float torque_scale_down = torque_scale_down = std::min(1.0f, std::max(0.0f, (max_wheel_speed - full_regen_limit_kph) / (no_regen_limit_kph - full_regen_limit_kph)));

    // limit torque based on overvoltage so that cells do not limit; using linear ramp
    float over_voltage_protection_scale = std::min(1.0f, std::max(0.1f, (max_inverter_voltage - start_regen_voltage_limit) / (max_regen_voltage_limit - start_regen_voltage_limit)));
    torque_scale_down *= (1.0f - over_voltage_protection_scale);

    // regen power limit (what was wrong with this??)
    // if (acu_data.tractive_system_current < 0) // we don't want to apply the regen power limit until we observe a negative
    // {
    //     float electrical_power = acu_data.max_measured_ts_out_voltage * (-1.0f * acu_data.tractive_system_current);
    //     float wheelspeed_to_power_scale = std::min(1.0f, std::max(0.0f, 1 - (max_wheel_rpm / 20000.0f)));
    //     torque_scale_down *= (1.0f - wheelspeed_to_power_scale);
    // }

    // over voltage, rules regen limit
    if (are_all_wheels_regen)
    {
        command_output.desired_torques.FL *= torque_scale_down;
        command_output.desired_torques.FR *= torque_scale_down;
        command_output.desired_torques.RL *= torque_scale_down;
        command_output.desired_torques.RR *= torque_scale_down;
    }

    return command_output;
}