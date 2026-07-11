#include "controllers/DrivebrainController.h"


DrivetrainCommand_s DrivebrainController::evaluate(const VCRData_s &state, unsigned long curr_millis)
{
    auto db_telem_input = state.interface_data.latest_drivebrain_telem_command;
    auto db_auxillary_input = state.interface_data.latest_drivebrain_auxillary_command;

    _check_drivebrain_command_timing_failure(db_telem_input, curr_millis, _telem_latency_info);
    _check_drivebrain_command_timing_failure(db_auxillary_input, curr_millis, _aux_latency_info);
    bool drivebrain_reinit_button_pressed = state.interface_data.dash_input_state.mc_reset_btn_is_pressed;

    if (drivebrain_reinit_button_pressed && (!_should_run_controller))
    {
        _should_run_controller = true;
    }

    DrivetrainCommand_s output;

    if (_should_run_controller && !_telem_latency_info.timing_failure)
    {
        output = db_telem_input.get_command();
    }
    else if (_should_run_controller && !_aux_latency_info.timing_failure)
    {
        output = db_auxillary_input.get_command();
    }
    else
    {
        _should_run_controller = false;
        DrivetrainCommand_s coast_to_stop = {
            .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f},
        };
        output = coast_to_stop;
    }

    // Handle worst latency updates
    if (_last_reset_worse_latency_clock == 0)
    {
        _last_reset_worse_latency_clock = curr_millis;
    }

    if (curr_millis - _last_reset_worse_latency_clock > WORST_LATENCY_PERIOD_MS)
    {
        _last_reset_worse_latency_clock = curr_millis;
        _telem_latency_info.worst_period_millis = 0;
        _aux_latency_info.worst_period_millis = 0;
    }

    unsigned long aux_latency_millis = std::max(
        curr_millis - db_auxillary_input.desired_speeds.last_recv_millis,
        curr_millis - db_auxillary_input.torque_limits.last_recv_millis
    );

    unsigned long telem_latency_millis = std::max(
        curr_millis - db_telem_input.desired_speeds.last_recv_millis,
        curr_millis - db_telem_input.torque_limits.last_recv_millis
    );

    _aux_latency_info.worst_period_millis = std::max(_aux_latency_info.worst_period_millis, aux_latency_millis);
    _telem_latency_info.worst_period_millis = std::max(_telem_latency_info.worst_period_millis, telem_latency_millis);

    return output;
}

void DrivebrainController::_check_drivebrain_command_timing_failure(StampedDrivetrainCommand_s command, unsigned long curr_millis, MessageLatencyInfo_s& latency_info)
{
    // Cases for timing_failure:

    // 1. we have not received any messages from the db (timestamped message recvd flag initialized as false in struct def)
    bool not_all_messages_recvd = ((!command.desired_speeds.recvd) || (!command.torque_limits.recvd));

    // 2. if the time between the current VCR curr_millis time and the last millis time that we recvd a drivebrain msg is too high
    auto last_speed_setpoint_timestamp = command.desired_speeds.last_recv_millis;
    auto last_torque_lim_timestamp = command.torque_limits.last_recv_millis;

    int speed_setpoint_latency = ::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_speed_setpoint_timestamp)));
    int torque_setpoint_latency = ::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_torque_lim_timestamp)));

    bool speed_setpoint_msg_too_latent = (speed_setpoint_latency > (int)_params.allowed_latency);
    bool torque_limit_message_too_latent = (torque_setpoint_latency > (int)_params.allowed_latency);

    // 3. if the relative latency is too high (time between the message members) -> (allowed latency / 2)
    int relative_latency = ::abs((int)(static_cast<int64_t>(last_speed_setpoint_timestamp) - static_cast<int64_t>(last_torque_lim_timestamp)));
    bool latency_diff_too_high = (relative_latency > ((int)_params.allowed_latency / 2));

    bool timing_failure = (speed_setpoint_msg_too_latent || torque_limit_message_too_latent || not_all_messages_recvd || latency_diff_too_high);

    latency_info.timing_failure = timing_failure;
}