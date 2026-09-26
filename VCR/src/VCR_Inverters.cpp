#include "VCR_Inverters.hpp"


InverterInterface fl_inverter_interface(1);
InverterInterface fr_inverter_interface(2);
InverterInterface rl_inverter_interface(3);
InverterInterface rr_inverter_interface(4);

static InverterInterfaceFuncts_s makeOneInverterFuncts(InverterInterface& inv)
{
    return InverterInterfaceFuncts_s {
        .setMotorsTorque = [&inv](torque_nm desired_torque_nm) { return inv.setMotorTorque(desired_torque_nm); },
        .setMotorsSpeed = [&inv](speed_rpm desired_speed_rpm)  { return inv.setMotorSpeed(desired_speed_rpm); },
        .setMotorsIdle = [&inv]() { return inv.setMotorIdle(); },
        .requestEnable = [&inv](bool enable) { return inv.requestEnable(enable); },
        .isReportedModeMatchingDT = [&inv](DrivetrainControlMode_e expected_mode) { return inv.isReportedModeMatchingDT(expected_mode); },
        .getInverterStatus = [&inv]() { return inv.getStatus(); },
        .getMotorMechanics = [&inv]() { return inv.getMotorMechanics(); }
    };
}

veh_vec<InverterInterfaceFuncts_s> makeInverterFuncts()
{
    return veh_vec<InverterInterfaceFuncts_s>(
        makeOneInverterFuncts(fl_inverter_interface),
        makeOneInverterFuncts(fr_inverter_interface),
        makeOneInverterFuncts(rl_inverter_interface),
        makeOneInverterFuncts(rr_inverter_interface)
    );
}