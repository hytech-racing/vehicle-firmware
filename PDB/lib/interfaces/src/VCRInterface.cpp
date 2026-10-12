#include "VCRInterface.hpp"


void VCRInterface::receiveVehicleStateCANMsg(const CAN_message_t &can_msg)
{
    CAR_STATES_t unpacked_msg;
    Unpack_CAR_STATES_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
    _current_vehicle_state = static_cast<VehicleState_e>(unpacked_msg.vehicle_state);
    _current_drivetrain_state = static_cast<DrivetrainState_e>(unpacked_msg.drivetrain_state);
    _is_drivebrain_in_control = unpacked_msg.drivebrain_in_control;
}

VehicleState_e VCRInterface::getCurrentVehicleState() const
{
    return _current_vehicle_state;
}

DrivetrainState_e VCRInterface::getCurrentDrivetrainState() const
{
    return _current_drivetrain_state;
}

bool VCRInterface::isDrivebrainInControl() const
{
    return _is_drivebrain_in_control;
}