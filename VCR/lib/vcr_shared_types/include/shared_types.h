#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H
#include <stdint.h>


namespace HTUnits
{
    using celcius = float;
    using watts = float;
    using torque_nm = float;
    using speed_rpm = float;
    using volts = float;
};

struct InverterStatus_s
{
    bool is_inverter_connected;
    bool is_inverter_enabled;
    bool is_fault_code_present;
    bool is_hv_present;
    HTUnits::volts dc_bus_voltage;
    unsigned long last_recv_millis = 0;
};

struct MotorMechanics_s
{
    HTUnits::watts actual_power;
    HTUnits::torque_nm actual_torque;
    HTUnits::speed_rpm actual_speed;
    unsigned long last_recv_millis = 0;
};

#endif // SHARED_TYPES_H