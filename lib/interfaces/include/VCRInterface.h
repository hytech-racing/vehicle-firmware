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

    void receive_car_states_data(const CAN_message_t &can_msg);

    void receive_inverter_status_1(const CAN_message_t &can_msg);

    void receive_inverter_status_2(const CAN_message_t &can_msg);

    void receive_inverter_status_3(const CAN_message_t &can_msg);

    void receive_inverter_status_4(const CAN_message_t &can_msg);

    /* State Observation + Control */
    bool is_in_pedals_calibration_state() { return _is_in_pedals_calibration_state; }

    bool is_in_steering_calibration_state() { return _is_in_steering_calibration_state; } //steering and pedals calibration states are the same, so we can use the same variable for both

    void disable_calibration_state() {_is_in_pedals_calibration_state = false;}

    void disable_steering_calibration_state() {_is_in_steering_calibration_state = false;}

    /* Getters */
    VehicleState_e get_vehicle_state() const { return _vehicle_state_value; }

    DrivetrainState_e get_drivetrain_state() const { return _drivetrain_state_value; }

    InverterBusVolts_s get_dc_bus_voltage() const { return _bus_voltages; }

    TorqueLimit_e get_torque_limit_mode() const { return _torque_limit; }

    bool get_db_in_ctrl() const { return _is_db_in_ctrl; }

    bool get_inverter_error() ;

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