#include "TorqueControllerMux.hpp"


template <size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::evaluateTCMux(ControllerMode_e requested_controller_mode, const VCRData_s& input_state)
{

    const DrivetrainCommand_s EMPTY_COMMAND = {
        .control_mode = DrivetrainControlMode_e::TORQUE,
        .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
        .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}};

    bool is_requested_mode_supported = (requested_controller_mode == ControllerMode_e::MODE_0)
                                       || (requested_controller_mode == ControllerMode_e::MODE_1)
                                       || (requested_controller_mode == ControllerMode_e::MODE_3)
                                       || (requested_controller_mode == ControllerMode_e::MODE_4);

    if (!is_requested_mode_supported)
    {
        _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS;
        return EMPTY_COMMAND;
    }

    int requested_mode_index = static_cast<int>(requested_controller_mode);
    int active_mode_index = static_cast<int>(_curr_tc_mux_status.active_controller_mode);

    if ((!_controller_evals[active_mode_index]) || (!_controller_evals[requested_mode_index]))
    {
        _curr_tc_mux_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER;
        return EMPTY_COMMAND;
    }

    DrivetrainCommand_s active_mode_evaluate_output = _controller_evals[active_mode_index](input_state, sys_time::hal_millis());

    bool is_mode_change_requested = requested_controller_mode != _curr_tc_mux_status.active_controller_mode;

    if (is_mode_change_requested)
    {
        DrivetrainCommand_s requested_mode_evaluate_output = _controller_evals[requested_mode_index](input_state, sys_time::hal_millis());

        bool can_switch_controller = _canSwitchController(input_state.system_data.drivetrain_data, active_mode_evaluate_output, requested_mode_evaluate_output);

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
        _curr_tc_mux_status.output_is_bypassing_limits = false; // FIX: reset here -- previously only ever set to true, never back to false

        if (active_mode_evaluate_output.control_mode == DrivetrainControlMode_e::TORQUE)
        {
            // Regen branch -> apply limits so that the pack voltage doesn't spike too high and regen is derated at low speed (EV.3.3.3)
            if (active_mode_evaluate_output.desired_torques.FL < 0.0f
                || active_mode_evaluate_output.desired_torques.FR < 0.0f
                || active_mode_evaluate_output.desired_torques.RL < 0.0f
                || active_mode_evaluate_output.desired_torques.RR < 0.0f)
            {
                active_mode_evaluate_output = _applyRegenLimit(
                    active_mode_evaluate_output,
                    input_state.system_data.drivetrain_data,
                    input_state.interface_data.stamped_acu_core_data.acu_data);
            }

            active_mode_evaluate_output = _applyTorqueLimit(active_mode_evaluate_output, _params.max_avg_torque_nm);
            _curr_tc_mux_status.active_torque_limit_value = _params.max_avg_torque_nm; // FIX: was never set in this branch before

            // Accel branch
            if (active_mode_evaluate_output.desired_torques.FL != 0.0f
                || active_mode_evaluate_output.desired_torques.FR != 0.0f
                || active_mode_evaluate_output.desired_torques.RL != 0.0f
                || active_mode_evaluate_output.desired_torques.RR != 0.0f)
            {
                active_mode_evaluate_output = _applyTorqueControlPowerLimit(
                    active_mode_evaluate_output,
                    input_state.system_data.drivetrain_data,
                    _params.max_power_limit_watts,
                    _params.max_avg_torque_nm);
            }
        }
        else if (active_mode_evaluate_output.control_mode == DrivetrainControlMode_e::SPEED)
        {
            // Regen branch -> apply limits so that the pack voltage doesn't spike too high
            if (active_mode_evaluate_output.desired_speeds.FL == 0.0f
                && active_mode_evaluate_output.desired_speeds.FR == 0.0f
                && active_mode_evaluate_output.desired_speeds.RL == 0.0f
                && active_mode_evaluate_output.desired_speeds.RR == 0.0f)
            {
                active_mode_evaluate_output = _applyRegenLimit(
                    active_mode_evaluate_output,
                    input_state.system_data.drivetrain_data,
                    input_state.interface_data.stamped_acu_core_data.acu_data);
            }

            active_mode_evaluate_output = _applySpeedLimit(active_mode_evaluate_output, _params.max_avg_speed_rpm);
            _curr_tc_mux_status.active_torque_limit_value = _params.max_avg_speed_rpm; // FIX: was never set in this branch before
                                                                                       // NOTE: this field is Nm-labeled but now holding an RPM
                                                                                       // value in SPEED mode -- see open item below

            // Accel branch
            if (active_mode_evaluate_output.desired_speeds.FL != 0.0f
                || active_mode_evaluate_output.desired_speeds.FR != 0.0f
                || active_mode_evaluate_output.desired_speeds.RL != 0.0f
                || active_mode_evaluate_output.desired_speeds.RR != 0.0f)
            {
                // FIX: was calling _applyTorqueControlPowerLimit (wrong function -- reads desired_torques,
                // which isn't the active field in SPEED mode, so power limiting was a silent no-op here)
                active_mode_evaluate_output = _applySpeedControlPowerLimit(
                    active_mode_evaluate_output,
                    input_state.system_data.drivetrain_data,
                    _params.max_power_limit_watts,
                    _params.max_avg_speed_rpm);
            }
        }
    }
    else
    {
        // any mode other than mode 0 = no torque, regen, or power limiting
        _curr_tc_mux_status.active_torque_limit_value = dti_motor_params::MOTOR_MAX_TORQUE_NM;
        _curr_tc_mux_status.output_is_bypassing_limits = true;
    }

    return active_mode_evaluate_output;
}

template <size_t num_controllers>
bool TorqueControllerMux<num_controllers>::_canSwitchController(
    DrivetrainDynamicReport_s active_drivetrain_data,
    DrivetrainCommand_s active_mode_evaluate_output,
    DrivetrainCommand_s requested_mode_evaluate_output)
{
    auto active_measured_speeds_array = active_drivetrain_data.measured_speeds.as_array();
    auto active_mode_eval_desired_torques_array = active_mode_evaluate_output.desired_torques.as_array();
    auto requested_mode_eval_desired_torques_array = requested_mode_evaluate_output.desired_torques.as_array();

    for (size_t i = 0; i < _params.num_motors; i++)
    {
        bool is_speed_preventing_mode_change =
            std::fabs(active_measured_speeds_array[i] * physical_motor_scales::RPM_TO_METERS_PER_SECOND) >= _params.max_speed_during_mode_change;

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

template <size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applySpeedLimit(const DrivetrainCommand_s& dt_command, float max_avg_speed_limit)
{
    DrivetrainCommand_s command_output = dt_command;

    float avg_speed = 0.0f;
    auto desired_speeds_array = command_output.desired_speeds.as_array();

    for (size_t i = 0; i < desired_speeds_array.size(); i++)
    {
        avg_speed += std::abs(desired_speeds_array[i]);
    }

    avg_speed /= _params.num_motors;

    if (avg_speed > max_avg_speed_limit)
    {
        float scale = avg_speed / max_avg_speed_limit;

        command_output.desired_speeds.FL = command_output.desired_speeds.FL / scale;
        command_output.desired_speeds.FR = command_output.desired_speeds.FR / scale;
        command_output.desired_speeds.RL = command_output.desired_speeds.RL / scale;
        command_output.desired_speeds.RR = command_output.desired_speeds.RR / scale;
    }

    return command_output;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applyTorqueControlPowerLimit(
    const DrivetrainCommand_s& dt_command,
    const DrivetrainDynamicReport_s& dynamic_report,
    float mech_power_limit,
    float max_torque_nm)
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
            return desired_wheel_torque; // no meaningful power info to scale against -- leave as-is
        }

        float current_wheel_omega = current_wheel_rpm * physical_motor_scales::RPM_TO_RAD_PER_SECOND;

        if (fabs(current_wheel_omega) < 0.1f) // FIX: near-stationary wheel -- P = T*omega ~= 0 regardless of torque,
                                              // so it isn't part of the over-limit power; just respect the hard cap
                                              // instead of dividing by ~0 below
        {
            float clamped_mag = std::min(fabs(desired_wheel_torque), max_torque_val);
            return (desired_wheel_torque < 0.0f) ? -clamped_mag : clamped_mag;
        }

        float current_wheel_power = fabs(desired_wheel_torque * current_wheel_omega);
        float desired_wheel_power_percentage = current_wheel_power / fabs(net_power_mag);
        float corner_power = desired_wheel_power_percentage * power_limit;

        float result = fabs(corner_power / current_wheel_omega);
        result = std::max(0.0f, std::min(result, max_torque_val));

        return (desired_wheel_torque < 0.0f) ? -result : result;
    };

    if (fabs(net_power_mag) > mech_power_limit)
    {
        command_output.desired_torques.FL =
            scale_desired_torques(command_output.desired_torques.FL, dynamic_report.measured_speeds.FL, net_power_mag, mech_power_limit, max_torque_nm);
        command_output.desired_torques.FR =
            scale_desired_torques(command_output.desired_torques.FR, dynamic_report.measured_speeds.FR, net_power_mag, mech_power_limit, max_torque_nm);
        command_output.desired_torques.RL =
            scale_desired_torques(command_output.desired_torques.RL, dynamic_report.measured_speeds.RL, net_power_mag, mech_power_limit, max_torque_nm);
        command_output.desired_torques.RR =
            scale_desired_torques(command_output.desired_torques.RR, dynamic_report.measured_speeds.RR, net_power_mag, mech_power_limit, max_torque_nm);
    }

    return command_output;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applySpeedControlPowerLimit(
    const DrivetrainCommand_s& dt_command,
    const DrivetrainDynamicReport_s& dynamic_report,
    float mech_power_limit,
    float max_speed_rpm)
{
    DrivetrainCommand_s command_output = dt_command;

    // use MEASURED torque and MEASURED speed to compute actual current mechanical power
    float net_measured_power = dynamic_report.measured_torques.FL * (dynamic_report.measured_speeds.FL * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                               + dynamic_report.measured_torques.FR * (dynamic_report.measured_speeds.FR * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                               + dynamic_report.measured_torques.RL * (dynamic_report.measured_speeds.RL * physical_motor_scales::RPM_TO_RAD_PER_SECOND)
                               + dynamic_report.measured_torques.RR * (dynamic_report.measured_speeds.RR * physical_motor_scales::RPM_TO_RAD_PER_SECOND);

    /**
     * @warning This is an approximation, not exact like the torque-mode correction. It assumes measured torque stays roughly constant as the speed target
     * changes which isn't true since DTI's internal speed loop will re-derive torque at the new target
     */
    auto scale_desired_speeds =
        [](float desired_wheel_speed_rpm, float measured_wheel_torque, float net_power_mag, float power_limit, float max_speed_val) -> float
    {
        if (fabs(net_power_mag) < 1e-3f || fabs(measured_wheel_torque) < 0.1f) // FIX: restored the measured_wheel_torque
                                                                               // guard -- this branch divides by it below
        {
            return 0.0f; // no meaningful torque/power to base a correction on, fail safe
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
        command_output.desired_speeds.FL =
            scale_desired_speeds(command_output.desired_speeds.FL, dynamic_report.measured_torques.FL, net_measured_power, mech_power_limit, max_speed_rpm);
        command_output.desired_speeds.FR =
            scale_desired_speeds(command_output.desired_speeds.FR, dynamic_report.measured_torques.FR, net_measured_power, mech_power_limit, max_speed_rpm);
        command_output.desired_speeds.RL =
            scale_desired_speeds(command_output.desired_speeds.RL, dynamic_report.measured_torques.RL, net_measured_power, mech_power_limit, max_speed_rpm);
        command_output.desired_speeds.RR =
            scale_desired_speeds(command_output.desired_speeds.RR, dynamic_report.measured_torques.RR, net_measured_power, mech_power_limit, max_speed_rpm);
    }

    return command_output;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_applyRegenLimit(
    const DrivetrainCommand_s& desired_controller_out,
    const DrivetrainDynamicReport_s& dynamic_report,
    const ACUCoreData_s acu_data)
{
    DrivetrainCommand_s command_output = desired_controller_out;

    auto current_speeds = dynamic_report.measured_speeds.as_array();
    auto current_inverter_voltage = dynamic_report.measured_hv_bus_voltage.as_array();

    // Bus voltage derating: reduces regen as the highest inverter bus voltage approaches the cutoff so cells do not limit
    volt max_inverter_voltage = 0.0f;
    for (size_t i = 0; i < _params.num_motors; i++)
    {
        max_inverter_voltage = std::max(max_inverter_voltage, current_inverter_voltage[i]);
    }
    float voltage_derate_range_v = tc_mux_default_params::REGEN_VOLTAGE_CUTOFF_V - tc_mux_default_params::REGEN_VOLTAGE_DERATE_START_V;
    float voltage_derate_scale =
        1.0f - std::clamp((max_inverter_voltage - tc_mux_default_params::REGEN_VOLTAGE_DERATE_START_V) / voltage_derate_range_v, 0.0f, 1.0f);

    // Speed derating: no regen at or below REGEN_ZERO_AT_OR_BELOW_KPH, full at or above REGEN_FULL_AT_OR_ABOVE_KPH, linear in between
    float speed_derate_range_kph = tc_mux_default_params::REGEN_FULL_AT_OR_ABOVE_KPH - tc_mux_default_params::REGEN_ZERO_AT_OR_BELOW_KPH;
    auto speedDerateScale = [speed_derate_range_kph](float wheel_rpm) -> float
    {
        float wheel_speed_kph = std::fabs(wheel_rpm) * physical_motor_scales::RPM_TO_KILOMETERS_PER_HOUR;
        return std::clamp((wheel_speed_kph - tc_mux_default_params::REGEN_ZERO_AT_OR_BELOW_KPH) / speed_derate_range_kph, 0.0f, 1.0f);
    };

    if (command_output.control_mode == DrivetrainControlMode_e::TORQUE)
    {
        // Per wheel, using that wheel's own speed, and only on braking torque: positive torque is never scaled here.
        // The speed and voltage derates are independent limits, so the stricter one applies (like the DTI's own limits)
        auto limitRegenTorque = [&](torque_nm desired_torque_nm, float wheel_rpm) -> torque_nm
        {
            return (desired_torque_nm < 0.0f) ? desired_torque_nm * std::min(speedDerateScale(wheel_rpm), voltage_derate_scale) : desired_torque_nm;
        };

        command_output.desired_torques.FL = limitRegenTorque(command_output.desired_torques.FL, current_speeds[0]);
        command_output.desired_torques.FR = limitRegenTorque(command_output.desired_torques.FR, current_speeds[1]);
        command_output.desired_torques.RL = limitRegenTorque(command_output.desired_torques.RL, current_speeds[2]);
        command_output.desired_torques.RR = limitRegenTorque(command_output.desired_torques.RR, current_speeds[3]);
    }
    else if (command_output.control_mode == DrivetrainControlMode_e::SPEED)
    {
        auto command_speeds = desired_controller_out.desired_speeds.as_array();

        float max_wheel_rpm = 0.0f;
        bool are_all_wheels_regen = true; // true when all wheels are targeting speeds below the current wheel speed
        for (size_t i = 0; i < _params.num_motors; i++)
        {
            max_wheel_rpm = std::max(max_wheel_rpm, std::fabs(current_speeds[i]));
            are_all_wheels_regen &= (command_speeds[i] < std::fabs(current_speeds[i]) || command_speeds[i] == 0);
        }
        float regen_derate_scale = std::min(speedDerateScale(max_wheel_rpm), voltage_derate_scale);

        if (are_all_wheels_regen)
        {
            /**
             * @note Linear interpolation used
             *       command_speeds are already below current_speeds here so scaling them further toward zero
             *       would WIDEN the speed error and command MORE aggressive braking/regen, not less.
             *       Instead we interpolate the target toward the MEASURED (current) speed as regen_derate_scale -> 0,
             *       which narrows the speed error the inner loop sees and should reduce the torque/regen it applies
             * @warning needs verification against actual DTI speed-loop behavior, ts might not work
             */
            command_output.desired_speeds.FL = current_speeds[0] + regen_derate_scale * (command_output.desired_speeds.FL - current_speeds[0]);
            command_output.desired_speeds.FR = current_speeds[1] + regen_derate_scale * (command_output.desired_speeds.FR - current_speeds[1]);
            command_output.desired_speeds.RL = current_speeds[2] + regen_derate_scale * (command_output.desired_speeds.RL - current_speeds[2]);
            command_output.desired_speeds.RR = current_speeds[3] + regen_derate_scale * (command_output.desired_speeds.RR - current_speeds[3]);
        }
    }

    return command_output;
}