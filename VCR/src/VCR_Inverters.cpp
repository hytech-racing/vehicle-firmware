#include "VCR_Inverters.h"


InverterInterface fl_inverter_interface(1);
InverterInterface fr_inverter_interface(2);
InverterInterface rl_inverter_interface(3);
InverterInterface rr_inverter_interface(4);


static InverterInterfaceFuncts_s make_one_inverter_functs(InverterInterface& inv)
{
    return InverterInterfaceFuncts_s {
        . = [&inv](float desired_rpm, float torque_limit_nm) { inv.set_speed(desired_rpm, torque_limit_nm); },
        .set_idle = [&inv]() { inv.set_idle(); },
        .set_inverter_control_word = [&inv](InverterControlWord_s cw) { inv.set_inverter_control_word(cw); },
        .get_status = [&inv]() { return inv.get_status(); },
        .get_motor_mechanics = [&inv]() { return inv.get_motor_mechanics(); },
    };
}

veh_vec<DrivetrainSystem::InverterFuncts_s> makeInverterFuncts()
{
    return veh_vec<DrivetrainSystem::InverterFuncts_s>(
        make_one_inverter_functs(fl_inverter_interface),
        make_one_inverter_functs(fr_inverter_interface),
        make_one_inverter_functs(rl_inverter_interface),
        make_one_inverter_functs(rr_inverter_interface)
    );
}