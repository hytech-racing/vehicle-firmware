#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H
#include <stdint.h>
namespace HTUnits
{
    using Celcius = float;
    using Watts = float;
    using Var = float;
    using TorqueNm = float;
    using SpeedRpm = float;
    using Volts = float;
}; // namespace HTUnits

/**
 * Drivetrain system accessible structs for
 * requesting change of state
 */
struct InverterControlWord_s
{
    bool inverter_enable : 1;
    bool hv_enable : 1;
    bool driver_enable : 1;
    bool remove_error : 1;
};

struct InverterControlInput_s
{
    int16_t speed_rpm_setpoint;
    float positive_torque_limit;
    float negative_torque_limit;
};

struct InverterControlParams_s
{
    uint16_t speed_control_kp;
    uint16_t speed_control_ki;
    uint16_t speed_control_kd;
};

/**
 * For the most part these are mirrors of the lower-level CAN struct data,
 * except with already fully float-ized data
 **/
struct InverterStatus_s
{
    mutable bool new_data : 1; // TODO: decide if we actually need this
    unsigned long last_recv_millis = 0;
    bool hv_present : 1;
    bool connected : 1;
    bool system_ready : 1;
    bool error : 1;
    bool warning : 1;
    bool quit_dc_on : 1;
    bool dc_on : 1;
    bool quit_inverter_on : 1;
    bool inverter_on : 1;
    bool derating_on : 1;
    HTUnits::Volts dc_bus_voltage;
    uint16_t diagnostic_number;
};

struct InverterTemps_s
{
    mutable bool new_data : 1; // TODO: decide if we actually need this
    unsigned long last_recv_millis = 0;
    HTUnits::Celcius motor_temp;
    HTUnits::Celcius inverter_temp;
    HTUnits::Celcius igbt_temp;
};

struct InverterPower_s
{
    mutable bool new_data : 1; // TODO: decide if we actually need this
    unsigned long last_recv_millis = 0;
    HTUnits::Watts active_power;
    HTUnits::Var reactive_power;
};

struct MotorMechanics_s
{
    mutable bool new_data : 1; // TODO: decide if we actually need this
    unsigned long last_recv_millis = 0;
    HTUnits::Watts actual_power;
    HTUnits::TorqueNm actual_torque;
    HTUnits::SpeedRpm actual_speed;
};

struct InverterControlFeedback_s
{
    mutable bool new_data : 1; // TODO: decide if we actually need this
    unsigned long last_recv_millis = 0;
    uint16_t speed_control_kp;
    uint16_t speed_control_ki;
    uint16_t speed_control_kd;
};

struct InverterFeedbackData_s
{
    InverterStatus_s status;
    InverterTemps_s temps;
    InverterPower_s power;
    MotorMechanics_s motor_mechanics;
    InverterControlFeedback_s control_feedback;
};

#endif // SHARED_TYPES_H
