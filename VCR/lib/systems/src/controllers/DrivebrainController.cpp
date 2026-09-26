#include "controllers/DrivebrainController.hpp"


DrivetrainCommand_s DrivebrainController::evaluate(const VCRData_s &curr_state, unsigned long curr_millis)
{
    auto db_telem_input = curr_state.interface_data.latest_drivebrain_telem_command;
    auto db_raux_input = curr_state.interface_data.latest_drivebrain_auxillary_command;

    _checkDrivebrainCommandTimingFailure(db_telem_input, curr_millis, _telem_latency_info);
    _checkDrivebrainCommandTimingFailure(db_raux_input, curr_millis, _raux_latency_info);

    // MC reset -> is this necessary anymore?
    bool drivebrain_reinit_button_pressed = curr_state.interface_data.dash_input_state.mc_reset_btn_is_pressed;
    if (drivebrain_reinit_button_pressed && (!_should_run_controller))
    {
        _should_run_controller = true;
    }

    DrivetrainCommand_s msg_out;

    if (_should_run_controller && !_telem_latency_info.has_timing_failure)
    {
        msg_out = db_telem_input.getCommand();
    }
    else if (_should_run_controller && !_raux_latency_info.has_timing_failure)
    {
        msg_out = db_raux_input.getCommand();
    }
    else
    {
        // Fail safe branch
        _should_run_controller = false;
        DrivetrainCommand_s coast_to_stop = {
            .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f},
        };
        msg_out = coast_to_stop;
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
        _raux_latency_info.worst_period_millis = 0;
    }

    unsigned long aux_latency_millis = std::max(
        curr_millis - db_raux_input.desired_speeds.last_recv_millis,
        curr_millis - db_raux_input.desired_torques.last_recv_millis
    );

    unsigned long telem_latency_millis = std::max(
        curr_millis - db_telem_input.desired_speeds.last_recv_millis,
        curr_millis - db_telem_input.desired_torques.last_recv_millis
    );

    _raux_latency_info.worst_period_millis = std::max(_raux_latency_info.worst_period_millis, aux_latency_millis);
    _telem_latency_info.worst_period_millis = std::max(_telem_latency_info.worst_period_millis, telem_latency_millis);

    return msg_out;
}

void DrivebrainController::_checkDrivebrainCommandTimingFailure(StampedDrivetrainCommand_s command, unsigned long curr_millis, MessageLatencyInfo_s& latency_info)
{
    // Cases for timing_failure:

    // 1. we have not received any messages from drivebrain (timestamped message recvd flag initialized as false in struct def)
    bool not_all_messages_recvd = ((!command.desired_speeds.recvd) || (!command.desired_torques.recvd));

    // 2. if the time between the current VCR curr_millis time and the last millis time that we recvd a drivebrain msg is too high
    auto last_speed_command_timestamp = command.desired_speeds.last_recv_millis;
    auto last_torque_command_timestamp = command.desired_torques.last_recv_millis;

    int speed_command_latency = ::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_speed_command_timestamp)));
    int torque_command_latency = ::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_torque_command_timestamp)));

    bool is_speed_command_too_latent = (speed_command_latency > (int)_params.allowed_latency);
    bool is_torque_command_too_latent = (torque_command_latency > (int)_params.allowed_latency);

    // 3. if the relative latency is too high (time between the message members) -> (allowed latency / 2)
    int relative_latency = ::abs((int)(static_cast<int64_t>(last_speed_command_timestamp) - static_cast<int64_t>(last_torque_command_timestamp)));
    bool latency_diff_too_high = (relative_latency > ((int)_params.allowed_latency / 2));

    bool timing_failure = (is_speed_command_too_latent || is_torque_command_too_latent || not_all_messages_recvd || latency_diff_too_high);

    latency_info.has_timing_failure = timing_failure;
}