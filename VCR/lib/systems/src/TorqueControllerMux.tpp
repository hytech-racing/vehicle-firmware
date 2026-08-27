#include "TorqueControllerMux.hpp"


template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::get_drivetrain_command(ControllerMode_e requested_controller_mode,
                                                                               TorqueLimit_e requested_torque_limit,
                                                                               const VCRData_s &input_state
)
{

    const DrivetrainCommand_s EMPTY_COMMAND = {.torque_setpoints = {0.0f, 0.0f, 0.0f, 0.0f}};

    /**
     * Want to make sure we are not requesting a controller that doesn't exist. The only controller modes that we using
     * are modes 0, 1, and 4. There is an argument for slip launch.
    */
    bool is_requested_mode_supported = (requested_controller_mode == ControllerMode_e::MODE_0) ||
                                    (requested_controller_mode == ControllerMode_e::MODE_1) ||
                                    (requested_controller_mode == ControllerMode_e::MODE_4);

    if (!is_requested_mode_supported)
    {
        _active_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS;
        return EMPTY_COMMAND;
    }

    /**
     * Here we are going to track...
     *  1) Which mode is the driver/dashboard currently requesting
     *  2) What mode the mux is actually running right now
     *
     * We want to keep track for these reasons...
     *  1) Want to double check both the active and requested mode's slots in _controller_evals actually
     *     holds a bound function before calling either.
     *  2) We need to perform safety checks before switching modes, so we need to keep track of requested and current mode.
     *
    */
    int requested_mode_index = static_cast<int>(requested_controller_mode);
    int active_mode_index = static_cast<int>(_active_status.active_controller_mode);

    /**
     * @brief Defensive guard against a construction-time mistake.
     *
     * _controller_evals is a fixed-size std::array<std::function<...>, N>.
     *
     * Each slot is either "empty" (holds no callable) or "bound" (wraps a real function/lambda).
     * We are checking whether a real function was ever assigned into this slot in the first place.
     *
     * Two distinct failure modes for an "invalid" std::function, depending on
     * what's actually wrong:
     *
     *   1) Empty (default-constructed, explicitly nullptr, or never assigned):
     *      compiles fine, but invoking it throws std::bad_function_call at
     *      runtime. This is the case this guard catches.
     *
     *   2) Wrong signature (the assigned callable's parameters/return type don't
     *      match std::function's declared signature): caught at compile time,
     *      never reaches runtime — not something this guard needs to handle.
     *
     * But if num_controllers ever changes, or a
     * future edit to that constructor forgets to bind one slot, that slot
     * silently stays empty — no compile error.
     */
    if ((!_controller_evals[active_mode_index]) || (!_controller_evals[requested_mode_index]))
    {
        _active_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER;
        return EMPTY_COMMAND;
    }

    // Evaluate the currently-active mode's controller (not the requested one yet), this becomes the default
    // output for this tick unless the mode-switch safety check approves switching to the requested mode instead
    DrivetrainCommand_s active_mode_output = _controller_evals[active_mode_index](input_state, sys_time::hal_millis());

    // std::cout << "output torques " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;

    bool is_mode_change_requested = requested_controller_mode != _active_status.active_controller_mode;

    if (is_mode_change_requested)
    {
        DrivetrainCommand_s proposed_output = _controller_evals[requested_mode_index](input_state, sys_time::hal_millis());

        bool can_switch_controller = _can_switch_controller(input_state.system_data.drivetrain_data,
                                                                    active_mode_output,
                                                                    proposed_output
        );

        if (can_switch_controller)
        {
            _active_status.active_controller_mode = requested_controller_mode;
            active_mode_index = requested_mode_index;
            active_mode_output = proposed_output;
        }
    }

    if (!_mux_bypass_limits[active_controller_mode_index])
    {
        _active_status.active_torque_limit_enum = requested_torque_limit;

        // Occurs when the desired speed is 0 (braking) and we want to allow regen -- need to apply limits so that the pack voltage doesn't spike too high
        if (current_output.desired_speeds.FL == 0.0f && current_output.desired_speeds.FR == 0.0f && current_output.desired_speeds.RL == 0.0f && current_output.desired_speeds.RR == 0.0f)
        {
            current_output = _apply_regen_limit(current_output, input_state.system_data.drivetrain_data, input_state.interface_data.stamped_acu_core_data.acu_data);
        }

        current_output = _apply_torque_limit(current_output, _torque_limit_map[requested_torque_limit]);
        _active_status.active_torque_limit_value = _torque_limit_map[requested_torque_limit];

        // Applied power limit when accelerating
        if (current_output.desired_speeds.FL != 0.0f || current_output.desired_speeds.FR != 0.0f || current_output.desired_speeds.RL != 0.0f || current_output.desired_speeds.RR != 0.0f)
        {
            current_output = _apply_power_limit(current_output, input_state.system_data.drivetrain_data, _max_power_limit, _torque_limit_map[requested_torque_limit]);
        }

        // std::cout << "output torques after pw " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
        current_output = _apply_positive_speed_limit(current_output);
        _active_status.output_is_bypassing_limits = false;
    }
    else
    {
        // any mode other than mode 0 = no torque, regen, or power limiting
        _active_status.active_torque_limit_enum = TorqueLimit_e::TCMUX_FULL_TORQUE;
        _active_status.active_torque_limit_value= PhysicalParameters::AMK_MAX_TORQUE;
        _active_status.output_is_bypassing_limits = true;
    }

    // std::cout << "output torques before return " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
    return current_output;
}

template <std::size_t num_controllers>
bool TorqueControllerMux<num_controllers>::_can_switch_controller(DrivetrainDynamicReport_s active_drivetrain_data,
                                                                                      DrivetrainCommand_s previous_controller_command,
                                                                                      DrivetrainCommand_s desired_controller_out
)
{
    auto measured_speeds_array = active_drivetrain_data.measured_speeds.as_array();
    auto desired_torque_setpoints_array = desired_controller_out.torque_setpoints.as_array();
    auto previous_torque_setpoints_array = previous_controller_command.torque_setpoints.as_array();

    for (size_t i = 0; i < _num_motors; i++)
    {
        bool is_speed_preventing_mode_change =
            std::fabs(measured_speeds_array[i] * RPM_TO_METERS_PER_SECOND) >= _max_speed_during_mode_change;

        bool is_torque_delta_preventing_mode_change =
            std::fabs(desired_torque_setpoints_array[i] - previous_torque_setpoints_array[i]) > _max_torque_delta_during_mode_change;

        if (is_speed_preventing_mode_change)
        {
            _active_status.active_error = TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH;
            return false;
        }

        if (is_torque_delta_preventing_mode_change)
        {
            _active_status.active_error = TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH;
            return false;
        }
    }

    _active_status.active_error = TorqueControllerMuxError_e::NO_ERROR;
    return true;
}


template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_apply_torque_limit(const DrivetrainCommand_s &command, float max_torque)
{
    DrivetrainCommand_s out = command;
    float avg_torque = 0;
    // get the average torque accross all 4 wheels
    auto torq_lims = out.torque_limits.as_array();
    for (size_t i = 0; i < torq_lims.size(); i++)
    {
        avg_torque += abs(torq_lims[i]);
    }

    avg_torque /= _num_motors;

    // if this is greather than the torque limit, scale down
    if (avg_torque > max_torque)
    {
        // get the scale of avg torque above max torque
        float scale = avg_torque / max_torque;
        // divide by scale to lower avg below max torque
        out.torque_limits.FL = out.torque_limits.FL / scale;
        out.torque_limits.FR = out.torque_limits.FR / scale;
        out.torque_limits.RL = out.torque_limits.RL / scale;
        out.torque_limits.RR = out.torque_limits.RR / scale;
    }

    return out;
}

/*
    Apply power limit such that the mechanical power of all wheels never
    exceeds the preset mechanical power limit. Scales all wheels down to
    preserve functionality of torque controllers
*/
template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_apply_power_limit(const DrivetrainCommand_s &command, const DrivetrainDynamicReport_s &drivetrain, float power_limit, float max_torque)
{
    DrivetrainCommand_s out = command;
    float net_torque_mag = 0;
    float net_power = 0;

    // uhh why not just put these all on one line? readability?
    net_torque_mag += out.torque_limits.FL;
    net_torque_mag += out.torque_limits.FR;
    net_torque_mag += out.torque_limits.RL;
    net_torque_mag += out.torque_limits.RR;

    net_power += (out.torque_limits.FL * (drivetrain.measuredSpeeds.FL * RPM_TO_RAD_PER_SECOND));
    net_power += (out.torque_limits.FR * (drivetrain.measuredSpeeds.FR * RPM_TO_RAD_PER_SECOND));
    net_power += (out.torque_limits.RL * (drivetrain.measuredSpeeds.RL * RPM_TO_RAD_PER_SECOND));
    net_power += (out.torque_limits.RR * (drivetrain.measuredSpeeds.RR * RPM_TO_RAD_PER_SECOND));
    // only evaluate power limit if current power exceeds it
    auto scale_torque_limit = [](float desired_wheel_torque, float current_wheel_rpm, float net_torque_mag, float power_limit, float max_torque) -> float
    {
        float res = desired_wheel_torque;

        float desired_wheel_torque_percentage = fabs(desired_wheel_torque / net_torque_mag);
        float corner_power = (desired_wheel_torque_percentage * power_limit);

        // power / omega (motor rad/s) to get torque per wheel
        res = fabs(corner_power / (current_wheel_rpm * RPM_TO_RAD_PER_SECOND));
        res = std::max(0.0f, std::min(res, max_torque)); // ensure torque limit is above zero and below max torque(?)

        return res;
    };

    if (net_power > power_limit)
    {
        out.torque_limits.FL  = scale_torque_limit(out.torque_limits.FL , drivetrain.measuredSpeeds.FL , net_torque_mag, power_limit, max_torque);
        out.torque_limits.FR  = scale_torque_limit(out.torque_limits.FR , drivetrain.measuredSpeeds.FR , net_torque_mag, power_limit, max_torque);
        out.torque_limits.RL  = scale_torque_limit(out.torque_limits.RL , drivetrain.measuredSpeeds.RL , net_torque_mag, power_limit, max_torque);
        out.torque_limits.RR  = scale_torque_limit(out.torque_limits.RR , drivetrain.measuredSpeeds.RR , net_torque_mag, power_limit, max_torque);
    }
    return out;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_apply_regen_limit(const DrivetrainCommand_s &command, const DrivetrainDynamicReport_s &drivetrain_data, const ACUCoreData_s acu_data)
{
    DrivetrainCommand_s out = command;
    const float no_regen_limit_kph = 10.0;
    const float full_regen_limit_kph = 5.0; // per rules EV.3.3.3

    const float start_regen_voltage_limit = 520.0;
    const float max_regen_voltage_limit = 530.0;

    const float start_regen_power_limit = 30000.0f;
    const float max_regen_power_limit = 50000.0f;

    float max_wheel_speed = 0.0;
    float torque_scale_down = 0.0;
    bool all_wheels_regen_flag = true; // true when all wheels are targeting speeds below the current wheel speed

    DrivetrainDynamicReport_s dt_data = drivetrain_data;
    auto speeds = dt_data.measuredSpeeds.as_array();
    auto command_speeds = out.desired_speeds.as_array();
    for (size_t i = 0; i < _num_motors; i++)
    {
        max_wheel_speed = std::max(max_wheel_speed, static_cast<float>(fabs(speeds[i]) * RPM_TO_KILOMETERS_PER_HOUR));
        all_wheels_regen_flag &= (command_speeds[i] < static_cast<float>(fabs(speeds[i])) || command_speeds[i] == 0);
    }

    // begin limiting regen at no_regen_limit_kph and completely limit regen at full_regen_limit_kph
    // linearly interpolate the scale factor between no_regen_limit_kph and full_regen_limit_kph
    torque_scale_down = std::min(1.0f, std::max(0.0f, (max_wheel_speed - full_regen_limit_kph) / (no_regen_limit_kph - full_regen_limit_kph)));

    // limit torque based on overvoltage so that cells do not
    float over_voltage_protection_scale = std::min(1.0f, std::max(0.1f, (dt_data.measuredInverterFLPackVoltage - start_regen_voltage_limit) / (max_regen_voltage_limit - start_regen_voltage_limit)));
    torque_scale_down *= (1.0f - over_voltage_protection_scale);

    // regen power limit
    // if (acu_data.tractive_system_current < 0) // we don't want to apply the regen power limit until we observe a negative
    // {
    //     float electrical_power = acu_data.max_measured_ts_out_voltage * (-1.0f * acu_data.tractive_system_current);
    //     float wheelspeed_to_power_scale = std::min(1.0f, std::max(0.0f, 1 - (max_wheel_rpm / 20000.0f)));
    //     torque_scale_down *= (1.0f - wheelspeed_to_power_scale);
    // }

    // over voltage, rules regen limit
    if (all_wheels_regen_flag)
    {
        out.torque_limits.FL *= torque_scale_down;
        out.torque_limits.FR *= torque_scale_down;
        out.torque_limits.RL *= torque_scale_down;
        out.torque_limits.RR *= torque_scale_down;
    }

    return out;
}