#include "DrivebrainInterface.h"

// void DrivebrainInterface::receiveDriverlessState(const CAN_message_t &msg, unsigned long curr_millis)
// {
//     DRIVERLESS_SYSTEM_STATUS_t driverless_state_msg;
//     Unpack_DRIVERLESS_SYSTEM_STATUS_ht_can(&driverless_state_msg, &msg.buf[0], msg.len);
//     _driverless_state = static_cast<DriverlessSystemState_e>(driverless_state_msg.driverless_state);
// }

void DrivebrainInterface::receiveVehicleState(const CAN_message_t &msg, unsigned long curr_millis)
{
    CAR_STATES_t car_state_msg;
    Unpack_CAR_STATES_ht_can(&car_state_msg, &msg.buf[0], msg.len);
    _vehicle_state = static_cast<VehicleState_e>(car_state_msg.vehicle_state);
}
