#ifndef DRIVEBRAININTERFACE_H
#define DRIVEBRAININTERFACE_H

/* ETL Library */
#include <SharedFirmwareTypes.h>
#include <etl/singleton.h>

#include "CANInterface.h"
#include "ht_can.h"
#include <FlexCAN_T4.h>
#include <sys/_types.h>

class DrivebrainInterface
{
    public:
    void receiveDriverlessState(const CAN_message_t &msg, unsigned long curr_millis);
    void receiveVehicleState(const CAN_message_t &msg, unsigned long curr_millis);

    DriverlessSystemState_e getDriverlessState() { return _driverless_state; }
    VehicleState_e getVehicleState() { return _vehicle_state; }

    private:
    DriverlessSystemState_e _driverless_state = DriverlessSystemState_e::OFF;
    VehicleState_e _vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;

#endif
