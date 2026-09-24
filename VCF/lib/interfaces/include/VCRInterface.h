#ifndef VCR_INTERFACE_H
#define VCR_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include <FlexCAN_T4.h>

/* Local System Includes */
#include "BuzzerController.h"

struct InverterErrorFlags_s
{
    veh_vec<bool> error;
};

struct InverterBusVolts_s
{
    veh_vec<int> voltage;
};

class VCRInterface
{
public:

    void receive_dash_control_data(const CAN_message_t &can_msg);

    void recieveCarStatesCANMsg(const CAN_message_t &can_msg);

    void receiveINV1StatusCANMsg(const CAN_message_t &can_msg);

    void receiveINV2StatusCANMsg(const CAN_message_t &can_msg);

    void receiveINV3StatusCANMsg(const CAN_message_t &can_msg);

    void receiveINV4StatusCANMsg(const CAN_message_t &can_msg);

    /* State Observation + Control */
    bool arePedalsCalibrating() { return _is_in_pedals_calibration_state; }

    bool isSteeringCalibrating() { return _is_in_steering_calibration_state; } //steering and pedals calibration states are the same, so we can use the same variable for both

    void disablePedalsCalibration() {_is_in_pedals_calibration_state = false;}

    void disableSteeringCalibration() {_is_in_steering_calibration_state = false;}

    /* Getters */
    VehicleState_e getVehicleState() const { return _vehicle_state_value; }

    DrivetrainState_e getDrivetrainState() const { return _drivetrain_state_value; }

    InverterBusVolts_s getDCBusVoltages() const { return _bus_voltages; }

    TorqueLimit_e getTorqueLimitMode() const { return _torque_limit; }

    bool isDrivebrainInControl() const { return _is_db_in_ctrl; }

    bool isInverterErrored();

private:

    bool _is_in_pedals_calibration_state = false;
    bool _is_in_steering_calibration_state = false;
    bool _is_db_in_ctrl;
    VehicleState_e _vehicle_state_value;
    DrivetrainState_e _drivetrain_state_value;
    TorqueLimit_e _torque_limit = TorqueLimit_e::TCMUX_LOW_TORQUE;
    InverterBusVolts_s _bus_voltages;

    // Creates object that reflects the inverter error status...the object holds the error flags for each inverter,
    // the getter above returns True if there's an error in any of the 4
    InverterErrorFlags_s _inv_error_status;

};

using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_H */