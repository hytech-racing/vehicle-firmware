#ifndef VCR_INTERFACE_HPP
#define VCR_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "ht_can.h"

/* Local Interface Includes */
#include "STM32_CANInterface.hpp"


class VCRInterface
{
public:

    void receiveVehicleStateCANMsg(const CAN_message_t &can_msg);

    VehicleState_e getCurrentVehicleState() const;

    DrivetrainState_e getCurrentDrivetrainState() const;

    bool isDrivebrainInControl() const;

private:

    bool _is_drivebrain_in_control = false;
    VehicleState_e _current_vehicle_state;
    DrivetrainState_e _current_drivetrain_state;

};

using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_HPP */