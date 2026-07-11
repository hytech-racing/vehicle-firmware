#include "TorqueControllerMux.hpp"


template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::get_drivetrain_command(ControllerMode_e requested_controller_type,
                                                                               TorqueLimit_e requested_torque_limit,
                                                                               const VCRData_s &input_state
)
{

    DrivetrainCommand_s empty_command = {.desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}, .torque_limits = {0.0f, 0.0f, 0.0f, 0.0f}};

    DrivetrainCommand_s current_output = empty_command;

    // why not use enums instead? won't need to cast to an int.
    // could be keeping enum classes to make sure only values of the same enum class can be directly compared without a cast
    int req_controller_mode_index = static_cast<int>(requested_controller_type);
    int active_controller_mode_index = static_cast<int>(_active_status.active_controller_mode);

    if ((std::size_t)req_controller_mode_index > ( _controller_evals.size() - 1 ))
    {
        _active_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS;
        return empty_command;
    }

    if( (!_controller_evals[active_controller_mode_index]) || (!_controller_evals[req_controller_mode_index]))
    {
        _active_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER;
        return empty_command;
    }

    current_output = _controller_evals[active_controller_mode_index](input_state, sys_time::hal_millis());

    // std::cout << "output torques " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;

    bool requesting_controller_change = requested_controller_type != _active_status.active_controller_mode; // if the requested mode is different than the current mode then go through the change logic

    if (requesting_controller_change)
    {
        DrivetrainCommand_s proposed_output = _controller_evals[req_controller_mode_index](input_state, sys_time::hal_millis());
        TorqueControllerMuxError_e error_state = can_switch_controller(input_state.system_data.drivetrain_data, current_output, proposed_output);

        //successful change
        if (error_state == TorqueControllerMuxError_e::NO_ERROR)
        {
            _active_status.active_controller_mode = requested_controller_type; //active = current; requested = future
            active_controller_mode_index = req_controller_mode_index;
            current_output = proposed_output;
        }
        _active_status.active_error = error_state;
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
TorqueControllerMuxError_e TorqueControllerMux<num_controllers>::can_switch_controller(DrivetrainDynamicReport_s active_drivetrain_data,
                                                                                      DrivetrainCommand_s previous_controller_command,
                                                                                      DrivetrainCommand_s desired_controller_out
)
{
    bool speedPreventsModeChange = false;

    // Check if torque delta permits mode change
    bool torqueDeltaPreventsModeChange = false;

    auto speeds = active_drivetrain_data.measuredSpeeds.as_array();
    auto desired_torq_lims = desired_controller_out.torque_limits.as_array();
    auto prev_torq_lims = previous_controller_command.torque_limits.as_array();


    // is there a specific reason why we want to switch under speed and torque differences? would it not be safer to keep it to switching under near-stationary conditions instead?
    for (size_t i = 0; i < _num_motors; i++)
    {
        // if a motor's speed >= 5 m/s, don't switch to new controller
        speedPreventsModeChange = (fabs(speeds[i] * RPM_TO_METERS_PER_SECOND) >= _max_change_speed);

        // only if the torque delta is positive do we not want to switch to the new one
        torqueDeltaPreventsModeChange = (desired_torq_lims[i] - prev_torq_lims[i]) > _max_torque_pos_change_delta;
        if (speedPreventsModeChange)
        {
            return TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH;
        }
        if (torqueDeltaPreventsModeChange)
        {
            return TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH;
        }
    }
    return TorqueControllerMuxError_e::NO_ERROR; //successful change to new controller
}

/* Apply limit such that wheelspeed never goes negative */
template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::_apply_positive_speed_limit(const DrivetrainCommand_s &command)
{
    DrivetrainCommand_s out;
    out = command;

    // I just hope HyTech never has to go back to single motor. that would be very sadge :(
    out.desired_speeds.FL = std::max(0.0f,command.desired_speeds.FL);
    out.desired_speeds.FR = std::max(0.0f,command.desired_speeds.FR);
    out.desired_speeds.RL = std::max(0.0f,command.desired_speeds.RL);
    out.desired_speeds.RR = std::max(0.0f,command.desired_speeds.RR);
    return out;
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