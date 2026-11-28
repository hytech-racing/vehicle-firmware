#ifndef __TEST_DRIVETRAIN__
#define __TEST_DRIVETRAIN__
#include "DrivetrainSystem.hpp"
#include "shared_types.hpp"

#include "gtest/gtest.h"

InverterStatus_s FL_status = {};
InverterStatus_s FR_status = {};
InverterStatus_s RL_status = {};
InverterStatus_s RR_status = {};

MotorMechanics_s FL_motor_mechanics = {};
MotorMechanics_s FR_motor_mechanics = {};
MotorMechanics_s RL_motor_mechanics = {};
MotorMechanics_s RR_motor_mechanics = {};

bool ef_is_active = false;
void setEfActivePin(bool value)
{
    ef_is_active = value;
}

class MockInverterInterface
{
  public:
    DrivetrainSystem::InverterFuncts_s inverter_functs;

    MockInverterInterface(InverterStatus_s& status, MotorMechanics_s& m_mech)
        : inverter_status(status), motor_mechanics(m_mech)
    {

        using namespace std::placeholders;

        inverter_functs.set_speed = std::bind(&MockInverterInterface::setSpeed, this, _1, _2);
        inverter_functs.set_idle = std::bind(&MockInverterInterface::setIdle, this);
        inverter_functs.set_inverter_control_word = std::bind(&MockInverterInterface::setInverterControlWord, this, _1);
        inverter_functs.get_status = std::bind(&MockInverterInterface::getStatus, this);
        inverter_functs.get_motor_mechanics = std::bind(&MockInverterInterface::getMotorMechanics, this);
    }

    InverterControlWord_s getControlWord() { return _last_control_word; }
    InverterControlInput_s getControlInput() { return _last_control_input; }

  private:
    InverterStatus_s& inverter_status;
    MotorMechanics_s& motor_mechanics;
    InverterControlWord_s _last_control_word;
    InverterControlInput_s _last_control_input;

    void setSpeed(float desired_rpm, float torque_limit_nm)
    {
        _last_control_input.speed_rpm_setpoint = desired_rpm;
        _last_control_input.positive_torque_limit = ::fabs(torque_limit_nm);
        _last_control_input.negative_torque_limit = -1.0f * ::fabs(torque_limit_nm);
        return;
    }

    void setIdle()
    {
        // inverter_status.speed_rpm = 0;
        return;
    }

    void setInverterControlWord(InverterControlWord_s control_word)
    {
        _last_control_word = control_word;
        return;
    }

    InverterStatus_s getStatus() { return inverter_status; }

    MotorMechanics_s getMotorMechanics() { return motor_mechanics; }
};

MockInverterInterface FL(FL_status, FL_motor_mechanics);
MockInverterInterface FR(FR_status, FR_motor_mechanics);
MockInverterInterface RL(RL_status, RL_motor_mechanics);
MockInverterInterface RR(RR_status, RR_motor_mechanics);

veh_vec<DrivetrainSystem::InverterFuncts_s> mock_inverter_functs = {FL.inverter_functs, FR.inverter_functs, RL.inverter_functs, RR.inverter_functs};

DrivetrainInit_s init = {DrivetrainModeRequest_e::INIT_DRIVE_MODE};
DrivetrainResetError_s reset = {true};
DrivetrainSystem::CmdVariant cmd = init;

void ASSERT_CONTROL_WORD_EQ(InverterControlWord_s actual, InverterControlWord_s expected)
{
    ASSERT_EQ(actual.inverter_enable, expected.inverter_enable);
    ASSERT_EQ(actual.hv_enable, expected.hv_enable);
    ASSERT_EQ(actual.driver_enable, expected.driver_enable);
    ASSERT_EQ(actual.remove_error, expected.remove_error);
}

void ASSERT_CONTROL_INPUT_EQ(InverterControlInput_s actual, InverterControlInput_s expected)
{
    ASSERT_EQ(actual.speed_rpm_setpoint, expected.speed_rpm_setpoint);
    ASSERT_EQ(actual.positive_torque_limit, expected.positive_torque_limit);
    ASSERT_EQ(actual.negative_torque_limit, expected.negative_torque_limit);
}

TEST(DrivetrainTest, test)
{
    DrivetrainSystem drivetrain = DrivetrainSystem(mock_inverter_functs, etl::delegate<void(bool)>::create<setEfActivePin>());

    FL_status.hv_present = false;
    FR_status.hv_present = false;
    RL_status.hv_present = false;
    RR_status.hv_present = false;

    FL_status.connected = false;
    FR_status.connected = false;
    RL_status.connected = false;
    RR_status.connected = false;

    FL_status.quit_dc_on = false;
    FR_status.quit_dc_on = false;
    RL_status.quit_dc_on = false;
    RR_status.quit_dc_on = false;

    FL_status.quit_inverter_on = false;
    FR_status.quit_inverter_on = false;
    RL_status.quit_inverter_on = false;
    RR_status.quit_inverter_on = false;

    FL_status.error = false;
    FR_status.error = false;
    RL_status.error = false;
    RR_status.error = false;

    // Calling init() when inverters are not connected (should stay in NOT_CONNECTED)
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_CONNECTED);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::CANNOT_INIT_NOT_CONNECTED);

    // Calling init (to disabled) when inverters are still not connected (should stay NOT_CONNECTED)
    DrivetrainInit_s init_disabled = {DrivetrainModeRequest_e::UNINITIALIZED};
    drivetrain.evaluateDrivetrain(init_disabled);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_CONNECTED);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::COMMAND_OK);

    // Calling init when only two of the inverters are connected (should stay in NOT_CONNECTED)
    FL_status.connected = true;
    FR_status.connected = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_CONNECTED);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::CANNOT_INIT_NOT_CONNECTED);

    // Calling init when the other two are connected (should stay in NOT_CONNECTED)
    FL_status.connected = false;
    FR_status.connected = false;
    RL_status.connected = true;
    RR_status.connected = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_CONNECTED);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::CANNOT_INIT_NOT_CONNECTED);

    // Calling init when all four are connected (should move to NOT_ENABLED_NO_HV_PRESENT)
    FL_status.connected = true;
    FR_status.connected = true;
    RL_status.connected = true;
    RR_status.connected = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::COMMAND_OK);

    // Calling init while HV is only present on some inverters (should stay in NOT_ENABLED_HV_PRESENT)
    FL_status.hv_present = true;
    FR_status.hv_present = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::COMMAND_OK);

    // Calling init while HV is only present on all inverters (should move to NOT_ENABLED_HV_PRESENT)
    RL_status.hv_present = true;
    RR_status.hv_present = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::COMMAND_OK);

    // Calling init() while some inverters are ready (should stay in NOT_ENABLED_HV_PRESENT)
    FL_status.system_ready = true;
    FR_status.system_ready = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::COMMAND_OK);

    // Calling init() while all inverters are ready (should move to INVERTERS_READY)
    RL_status.system_ready = true;
    RR_status.system_ready = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_EQ(drivetrain.getStatus().cmd_resp, DrivetrainCmdResponse_e::COMMAND_OK);

    // Calling init() with various combinations of requesting_init, inverters_ready, and quit_dc_on
    // (requesting init) && (inverters NOT ready) && (quit dc NOT on)
    RL_status.system_ready = false;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT); // Returns to HV_PRESENT because RL not ready
    ASSERT_EQ(ef_is_active, false);

    // (requesting init) && (inverters ready) && (quit dc NOT on)
    RL_status.system_ready = true;
    drivetrain.evaluateDrivetrain(init); // Returning to INVERTERS_READY state
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY);
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY);
    ASSERT_EQ(ef_is_active, false);

    // (NOT requesting init) && (inverters ready) && (quit dc NOT on)
    drivetrain.evaluateDrivetrain(init_disabled);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY);
    ASSERT_EQ(ef_is_active, false);

    // (requesting init) && (inverters ready) && (quit dc NOT on)
    RL_status.quit_dc_on = true;
    RR_status.quit_dc_on = true;
    FL_status.quit_dc_on = false;
    FR_status.quit_dc_on = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY); // Stays in INVERTERS_READY because NOT quit_dc_on
    ASSERT_EQ(ef_is_active, false);

    // (NOT requesting init) && (inverters ready) && (quit dc on)
    FL_status.quit_dc_on = true;
    drivetrain.evaluateDrivetrain(init_disabled);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY); // Stays in INVERTERS_READY because NOT requesting init
    ASSERT_EQ(ef_is_active, false);

    // (requesting init) && (inverters ready) && (quit dc on)
    uint32_t arbitrary_time_ms = 2001;
    sys_time::set_millis(arbitrary_time_ms);
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED); // Goes to INVERTERS_HV_ENABLED because all conditions have been met
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // Requesting init before 50ms have passed (inverters should not be enabled yet)
    sys_time::set_millis(arbitrary_time_ms + 49);
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // Requesting init after 50ms have passed (inverters should be getting enabled)
    sys_time::set_millis(arbitrary_time_ms + 51);
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED); // Goes to INVERTERS_HV_ENABLED because all conditions have been met
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // Calling init() with various combinations of (hv_enabled && inverters_ready && inverters_enabled)
    // hv_enabled, inverters ready, inverters NOT enabled
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED); // Stays in same state
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // hv_enabled, inverters NOT ready, inverters NOT enabled
    RL_status.system_ready = false;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED); // Stays in same state
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // hv NOT enabled, inverters ready, inverters NOT enabled
    RL_status.system_ready = true;
    RL_status.quit_dc_on = false;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_READY); // steps back to INVERTERS_READY
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, true, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // hv enabled, inverters ready, inverters NOT enabled
    RL_status.quit_dc_on = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED); // return to INVERTERS_HV_ENABLED

    sys_time::set_millis(arbitrary_time_ms + 150);
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED); // stays in INVERTERS_HV_ENABLED
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // hv enabled, inverters ready, inverters PARTIALLY enabled
    RL_status.quit_inverter_on = true;
    RR_status.quit_inverter_on = true;
    FL_status.quit_inverter_on = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // hv enabled, inverters ready, inverters enabled
    FR_status.quit_inverter_on = true;
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::ENABLED_DRIVE_MODE);
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // Invalid drive command, return to INVERTERS_HV_ENABLED
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    ASSERT_EQ(ef_is_active, true);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::ENABLED_DRIVE_MODE);

    // Valid drive command, stays in ENABLED_DRIVE_MODE
    DrivetrainCommand_s drive_command = {{1000, 1000, 1000, 1000}, {9999, 9999, 9999, 9999}};
    drivetrain.evaluateDrivetrain(drive_command);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::ENABLED_DRIVE_MODE);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {true, true, true, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {1000, 9999.0f, -9999.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {1000, 9999.0f, -9999.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {1000, 9999.0f, -9999.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {1000, 9999.0f, -9999.0f});

    // Error present, going to ERROR
    FR_status.error = true;
    drivetrain.evaluateDrivetrain(drive_command);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::ERROR);
    drivetrain.evaluateDrivetrain(drive_command); // Evaluate once in the ERROR state
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // Clearing errors but error still present
    drivetrain.evaluateDrivetrain(reset);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::CLEARING_ERRORS);
    drivetrain.evaluateDrivetrain(reset);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::CLEARING_ERRORS);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, true, true, true});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, true, true, true});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, true, true, true});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, true, true, true});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});

    // Clearing errors and error is gone
    FR_status.error = false;
    drivetrain.evaluateDrivetrain(reset);
    ASSERT_EQ(drivetrain.getState(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    drivetrain.evaluateDrivetrain(init);
    ASSERT_EQ(ef_is_active, false);
    ASSERT_CONTROL_WORD_EQ(FL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(FR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RL.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_WORD_EQ(RR.getControlWord(), {false, false, false, false});
    ASSERT_CONTROL_INPUT_EQ(FL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(FR.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RL.getControlInput(), {0, 0.0f, 0.0f});
    ASSERT_CONTROL_INPUT_EQ(RR.getControlInput(), {0, 0.0f, 0.0f});
}

#endif
