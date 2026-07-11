#ifndef VCR_INTERFACE_H
#define VCR_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech.h"

/* Local Interface Includes */
#include "CANInterface.h"


struct MotorMechanics_s
{
    bool new_data : 1;
    unsigned long last_recv_millis = 0;
    float actual_power_watts; //watts
    float actual_torque_nm;   //newton meters
    float actual_speed_rpm;   //rpm
};

struct Temperature_s
{
    veh_vec<int> inverter_temps;
    veh_vec<int> motor_temps;
};

struct InverterStatus_s
{
    veh_vec<bool> error;
    veh_vec<float> dc_bus_voltage;
    veh_vec<int> error_id;
};

class VCRInterface
{
public:

    void receive_inv_dynamics(const CAN_message_t &can_msg, unsigned long curr_millis);

    void receive_vehicle_state(const CAN_message_t &can_msg);

    void receive_inverter_status_1(const CAN_message_t &can_msg);
    void receive_inverter_status_2(const CAN_message_t &can_msg);
    void receive_inverter_status_3(const CAN_message_t &can_msg);
    void receive_inverter_status_4(const CAN_message_t &can_msg);

    void receive_inverter_temperature_1(const CAN_message_t &can_msg);
    void receive_inverter_temperature_2(const CAN_message_t &can_msg);
    void receive_inverter_temperature_3(const CAN_message_t &can_msg);
    void receive_inverter_temperature_4(const CAN_message_t &can_msg);

    bool is_in_pedals_calibration_state() { return _is_in_pedals_calibration_state; }

    bool get_drivebrain_in_control() { return _is_db_in_ctrl; }

    TorqueLimit_e get_torque_limit_mode() { return _torque_limit; }

    MotorMechanics_s get_curr_wheel_data() { return _wheel_data; }

    VehicleState_e get_curr_car_state() { return _vehicle_state_value; }

    int get_inverter_max_temp() { return std::max({_temps.inverter_temps.FL,
                                                _temps.inverter_temps.FR,
                                                _temps.inverter_temps.RL,
                                                _temps.inverter_temps.RR}); }

    int get_motor_max_temp() { return std::max({_temps.motor_temps.FL,
                                                _temps.motor_temps.FR,
                                                _temps.motor_temps.RL,
                                                _temps.motor_temps.RR}); }

private:

    bool _is_in_pedals_calibration_state = false;
    bool _is_db_in_ctrl;
    TorqueLimit_e _torque_limit = TorqueLimit_e::TCMUX_LOW_TORQUE;
    MotorMechanics_s _wheel_data;
    VehicleState_e _vehicle_state_value;
    Temperature_s _temps;
    DrivetrainState_e _drivetrain_state_value;
    InverterStatus_s _inverter_status;

};

using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_H */