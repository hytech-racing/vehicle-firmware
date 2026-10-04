
#include "VCRInterface.h"

//Only looking for the vehicle state from the VCR
void VCRInterface::receive_vehicle_state(const CAN_message_t &can_msg) {
    CAR_STATES_t unpacked_msg;
    Unpack_CAR_STATES_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    set_curr_car_state(static_cast<VehicleState_e>(unpacked_msg.vehicle_state));
    set_curr_drivetrain_state(static_cast<DrivetrainState_e>(unpacked_msg.drivetrain_state));
    set_curr_db_in_ctrl(unpacked_msg.drivebrain_in_control);
}