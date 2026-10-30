#include "VCR_Inverters.hpp"


InverterInterface fl_inverter_interface(dti_node_ids::FL);
InverterInterface fr_inverter_interface(dti_node_ids::FR);
InverterInterface rl_inverter_interface(dti_node_ids::RL);
InverterInterface rr_inverter_interface(dti_node_ids::RR);

static InverterInterfaceFuncts_s makeOneInverterFuncts(InverterInterface& inv)
{
    return InverterInterfaceFuncts_s{
        .setMotorsTorque = etl::delegate<void(torque_nm)>::create<InverterInterface, &InverterInterface::setMotorTorque>(inv),
        .setMotorsSpeed = etl::delegate<void(speed_rpm)>::create<InverterInterface, &InverterInterface::setMotorSpeed>(inv),
        .setMotorsIdle = etl::delegate<void()>::create<InverterInterface, &InverterInterface::setMotorIdle>(inv),
        .requestEnable = etl::delegate<void(bool)>::create<InverterInterface, &InverterInterface::requestEnable>(inv),
        .isReportedModeMatchingDT = etl::delegate<bool(DrivetrainControlMode_e)>::create<InverterInterface, &InverterInterface::isReportedModeMatchingDTS>(inv),
        .getInverterStatus = etl::delegate<InverterStatus_s(unsigned long)>::create<InverterInterface, &InverterInterface::getStatus>(inv),
        .getMotorMechanics = etl::delegate<MotorMechanics_s()>::create<InverterInterface, &InverterInterface::getMotorMechanics>(inv),
        .sendCommands = etl::delegate<void(DrivetrainControlMode_e)>::create<InverterInterface, &InverterInterface::sendControlCommands>(inv)};
}

veh_vec<InverterInterfaceFuncts_s> makeInverterFuncts()
{
    return veh_vec<InverterInterfaceFuncts_s>(
        makeOneInverterFuncts(fl_inverter_interface),
        makeOneInverterFuncts(fr_inverter_interface),
        makeOneInverterFuncts(rl_inverter_interface),
        makeOneInverterFuncts(rr_inverter_interface));
}