#include "controls.h"


void VCRControls::handleDrivetrainCommand(bool ready_to_drive, unsigned long curr_millis)
{
    if (_dt_system != nullptr)
    {
        ControllerMode_e mode = vcr_data.interface_data.dash_input_state.dial_state;
        DrivetrainStatus_s drivetrain_status;

        if (ready_to_drive)
        {
            auto dt_command = _tc_mux.evaluateTCMux(mode, _torque_limit, vcr_data);
            _debug_dt_command = dt_command;
            drivetrain_status = _dt_system->evaluate_drivetrain(dt_command, curr_millis);
        }
        else
        {
            // DrivetrainInit_s dt_command = {
            //     .init_drivetrain = UNINITIALIZED
            // };
            // drivetrain_status = _dt_system->evaluate_drivetrain(dt_command);
        }
    }
}

bool VCRControls::isDrivebrainInControll() const
{
    auto status = _tc_mux.getTCMuxStatus();
    return (!_mode4.hasTimingFailure()) && (status.active_controller_mode == ControllerMode_e::MODE_4);
}

bool VCRControls::drivebrainHasTimingFailure() const
{
    return _mode4.hasTimingFailure();
}

void VCRControls::enqueueLatencyCANData()
{
    MessageLatencyInfo_s aux_latency_info = _mode4.getRAUXLatencyInfo();
    MessageLatencyInfo_s telem_latency_info = _mode4.getTELEMLatencyInfo();

    // Enqueue timing faults
    DRIVEBRAIN_LATENCY_STATUSES_t status_msg;
    status_msg.db_aux_timing_fault = aux_latency_info.has_timing_failure;
    status_msg.db_telem_timing_fault = telem_latency_info.has_timing_failure;
    CAN_util::enqueue_msg(&status_msg,
                        &Pack_DRIVEBRAIN_LATENCY_STATUSES_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer);

    // Enqueue latency periods
    DRIVEBRAIN_LATENCY_TIMES_t latency_msg;
    latency_msg.aux_latency_millis = static_cast<int>(aux_latency_info.worst_period_millis);
    latency_msg.telem_latency_millis = static_cast<int>(telem_latency_info.worst_period_millis);
    CAN_util::enqueue_msg(&latency_msg,
                        &Pack_DRIVEBRAIN_LATENCY_TIMES_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer);
}