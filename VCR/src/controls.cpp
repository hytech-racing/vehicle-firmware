#include "controls.hpp"


void VCRControls::handleDrivetrainCommand(bool is_drive_enable_requested, bool is_torque_allowed, unsigned long curr_millis)
{
    if (_dt_system != nullptr) // if drivetrain initalized
    {
        ControllerMode_e mode = vcr_data.interface_data.dash_input_state.dial_state;

        // The mux is evaluated every tick so its controller selection stays current, but its output is only used in READY_TO_DRIVE
        auto dt_command = _tc_mux.evaluateTCMux(mode, vcr_data);
        if (!is_torque_allowed)
        {
            dt_command = {
                .control_mode = DrivetrainControlMode_e::TORQUE,
                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}};
        }
        dt_command.is_drive_enable_requested = is_drive_enable_requested; // controllers don't own enable, the VSM does
        _debug_dt_command = dt_command;
        _dt_system->evaluateDrivetrain(dt_command, curr_millis);
    }
}

bool VCRControls::isDrivebrainInControll() const
{
    auto status = _tc_mux.getTCMuxStatus();
    return (!_mode4.hasTimingFailure()) && (status.active_controller_mode == ControllerMode_e::MODE_4);
}

void VCRControls::enqueueLatencyCANData() const
{
    MessageLatencyInfo_s aux_latency_info = _mode4.getRAUXLatencyInfo();
    MessageLatencyInfo_s telem_latency_info = _mode4.getTELEMLatencyInfo();

    // Enqueue timing faults
    DRIVEBRAIN_LATENCY_STATUSES_t status_msg;
    status_msg.db_aux_timing_fault = aux_latency_info.has_timing_failure;
    status_msg.db_telem_timing_fault = telem_latency_info.has_timing_failure;
    CAN_util::enqueue_msg(&status_msg, &Pack_DRIVEBRAIN_LATENCY_STATUSES_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer);

    // Enqueue latency periods
    DRIVEBRAIN_LATENCY_TIMES_t latency_msg;
    latency_msg.aux_latency_millis = static_cast<int>(aux_latency_info.worst_period_millis);
    latency_msg.telem_latency_millis = static_cast<int>(telem_latency_info.worst_period_millis);
    CAN_util::enqueue_msg(&latency_msg, &Pack_DRIVEBRAIN_LATENCY_TIMES_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer);
}