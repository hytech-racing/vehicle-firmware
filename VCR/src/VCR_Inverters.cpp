#include "VCR_Inverters.h"


InverterInterface fl_inverter_interface(fl_can_ids.control_word,
                                        fl_can_ids.control_input,
                                        fl_can_ids.control_parameter,
                                        {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);
InverterInterface fr_inverter_interface(fr_can_ids.control_word,
                                        fr_can_ids.control_input,
                                        fr_can_ids.control_parameter,
                                        {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);
InverterInterface rl_inverter_interface(rl_can_ids.control_word,
                                        rl_can_ids.control_input,
                                        rl_can_ids.control_parameter,
                                        {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);
InverterInterface rr_inverter_interface(rr_can_ids.control_word,
                                        rr_can_ids.control_input,
                                        rr_can_ids.control_parameter,
                                        {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);

static DrivetrainSystem::InverterFuncts_s make_one_inverter_functs(InverterInterface& inv)
{
    return DrivetrainSystem::InverterFuncts_s {
        .set_speed = [&inv](float desired_rpm, float torque_limit_nm) { inv.set_speed(desired_rpm, torque_limit_nm); },
        .set_idle = [&inv]() { inv.set_idle(); },
        .set_inverter_control_word = [&inv](InverterControlWord_s cw) { inv.set_inverter_control_word(cw); },
        .get_status = [&inv]() { return inv.get_status(); },
        .get_motor_mechanics = [&inv]() { return inv.get_motor_mechanics(); },
    };
}

veh_vec<DrivetrainSystem::InverterFuncts_s> make_inverter_functs()
{
    return veh_vec<DrivetrainSystem::InverterFuncts_s>(
        make_one_inverter_functs(fl_inverter_interface),
        make_one_inverter_functs(fr_inverter_interface),
        make_one_inverter_functs(rl_inverter_interface),
        make_one_inverter_functs(rr_inverter_interface)
    );
}