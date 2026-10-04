#ifndef VCR_INTERFACE_H
#define VCR_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech.h"

/* Local Interface Includes */
#include "CANInterface.h"

//Setters methods only exist for the VCRInterface.cpp updating from CAN message (should not be used generally)
class VCRInterface
{
public:
    void receive_vehicle_state(const CAN_message_t &can_msg);

    VehicleState_e get_curr_car_state() { return _vehicle_state_value; }
    void set_curr_car_state(VehicleState_e vehicle_state) { _vehicle_state_value = vehicle_state; }

    DrivetrainState_e get_curr_drivetrain_state() { return _drivetrain_state_value; }
    void set_curr_drivetrain_state(DrivetrainState_e drivetrain_state) { _drivetrain_state_value = drivetrain_state;  }
    
    bool get_curr_db_in_ctrl() { return _is_db_in_ctrl; }
    void set_curr_db_in_ctrl(bool db_ctrl) { _is_db_in_ctrl = db_ctrl; }

private:
    bool _is_db_in_ctrl;
    VehicleState_e _vehicle_state_value;
    DrivetrainState_e _drivetrain_state_value;
};

using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_H */