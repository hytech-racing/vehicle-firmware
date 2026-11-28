#include "controls.hpp"


VCRControls::VCRControls(DrivetrainSystem* dt_system, uint32_t max_allowed_db_latency_ms)
    : _mode4(max_allowed_db_latency_ms),
      _tc_mux(
          {[this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
           [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode1.evaluate(state, curr_millis); },
           [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
           [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode3.evaluate(state, curr_millis); },
           [this](const VCRData_s& state, unsigned long curr_millis) -> DrivetrainCommand_s
           {
               return _mode4.evaluate(state, curr_millis);
           }},
          {false, false, false, false, true}),
      _dt_system(dt_system) {};

void VCRControls::handleDrivetrainCommand(bool wanting_ready_to_drive, bool ready_to_drive)
{
    if (_dt_system != nullptr)
    {
        ControllerMode_e mode = vcr_data.interface_data.dash_input_state.dial_state;
        DrivetrainStatus_s drivetrain_status;

        if (ready_to_drive)
        {
            auto dt_command = _tc_mux.getDrivetrainCommand(mode, _torque_limit, vcr_data);
            _debug_dt_command = dt_command;
            drivetrain_status = _dt_system->evaluateDrivetrain(dt_command);
        }
        else if (wanting_ready_to_drive)
        {
            DrivetrainInit_s dt_command = {.init_drivetrain = INIT_DRIVE_MODE};
            drivetrain_status = _dt_system->evaluateDrivetrain(dt_command);
        }
        else
        {
            DrivetrainInit_s dt_command = {.init_drivetrain = UNINITIALIZED};
            drivetrain_status = _dt_system->evaluateDrivetrain(dt_command);
        }
    }
}

bool VCRControls::drivebrainIsInControl() const
{
    auto status = _tc_mux.getTcMuxStatus();
    return (!_mode4.getTimingFailureStatus()) && (status.active_controller_mode == ControllerMode_e::MODE_4);
}

bool VCRControls::drivebrainTimingFailure() const
{
    return _mode4.getTimingFailureStatus();
}

void VCRControls::sendControlsCANMsgs()
{
    MessageLatencyInfo_s aux_latency_info = _mode4.getAuxLatencyData();
    MessageLatencyInfo_s telem_latency_info = _mode4.getTelemLatencyData();

    // Enqueue timing faults
    DRIVEBRAIN_LATENCY_STATUSES_t status_msg;

    status_msg.db_aux_timing_fault = aux_latency_info.timing_failure;
    status_msg.db_telem_timing_fault = telem_latency_info.timing_failure;

    CAN_util::enqueue_msg(&status_msg, &Pack_DRIVEBRAIN_LATENCY_STATUSES_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer);

    // Enqueue latency periods
    DRIVEBRAIN_LATENCY_TIMES_t latency_msg;

    latency_msg.aux_latency_millis = static_cast<int>(aux_latency_info.worst_period_millis);
    latency_msg.telem_latency_millis = static_cast<int>(telem_latency_info.worst_period_millis);

    CAN_util::enqueue_msg(&latency_msg, &Pack_DRIVEBRAIN_LATENCY_TIMES_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer);
}
