#ifndef TEST_DRIVETRAIN_HPP
#define TEST_DRIVETRAIN_HPP

#include "DrivetrainSystem.hpp"

#include <cmath>
#include <gtest/gtest.h>

/**
 * @brief Stands in for one InverterInterface. Tests set what it reports and read back what the drivetrain commanded.
 */
class MockInverterInterface
{
  public:
    /* What this inverter reports */
    InverterStatus_s status = {};
    bool is_mode_matching = true;

    /* What the drivetrain told it */
    float last_torque_nm = NAN;
    float last_speed_rpm = NAN;
    int idle_calls = 0;
    bool is_enable_requested = false;
    int send_calls = 0;
    DrivetrainControlMode_e last_sent_mode = DrivetrainControlMode_e::TORQUE;

    void setTorque(torque_nm torque) { last_torque_nm = torque; }
    void setSpeed(speed_rpm speed) { last_speed_rpm = speed; }
    void setIdle() { idle_calls++; }
    void requestEnable(bool enable) { is_enable_requested = enable; }
    bool isModeMatching(DrivetrainControlMode_e mode) { return is_mode_matching; }
    InverterStatus_s getStatus(unsigned long curr_millis) { return status; }
    MotorMechanics_s getMechanics() { return {}; }
    void sendCommands(DrivetrainControlMode_e mode)
    {
        send_calls++;
        last_sent_mode = mode;
    }

    InverterInterfaceFuncts_s functs()
    {
        return InverterInterfaceFuncts_s{
            .setMotorsTorque = etl::delegate<void(torque_nm)>::create<MockInverterInterface, &MockInverterInterface::setTorque>(*this),
            .setMotorsSpeed = etl::delegate<void(speed_rpm)>::create<MockInverterInterface, &MockInverterInterface::setSpeed>(*this),
            .setMotorsIdle = etl::delegate<void()>::create<MockInverterInterface, &MockInverterInterface::setIdle>(*this),
            .requestEnable = etl::delegate<void(bool)>::create<MockInverterInterface, &MockInverterInterface::requestEnable>(*this),
            .isReportedModeMatchingDT =
                etl::delegate<bool(DrivetrainControlMode_e)>::create<MockInverterInterface, &MockInverterInterface::isModeMatching>(*this),
            .getInverterStatus = etl::delegate<InverterStatus_s(unsigned long)>::create<MockInverterInterface, &MockInverterInterface::getStatus>(*this),
            .getMotorMechanics = etl::delegate<MotorMechanics_s()>::create<MockInverterInterface, &MockInverterInterface::getMechanics>(*this),
            .sendCommands = etl::delegate<void(DrivetrainControlMode_e)>::create<MockInverterInterface, &MockInverterInterface::sendCommands>(*this)};
    }
};

class DrivetrainTest : public ::testing::Test
{
  protected:
    MockInverterInterface fl, fr, rl, rr;
    bool is_hv_ok = false;
    bool isHVOk() { return is_hv_ok; }

    DrivetrainSystem drivetrain{
        veh_vec<InverterInterfaceFuncts_s>(fl.functs(), fr.functs(), rl.functs(), rr.functs()),
        etl::delegate<bool()>::create<DrivetrainTest, &DrivetrainTest::isHVOk>(*this)};

    unsigned long now = 0;

    std::array<MockInverterInterface*, 4> all() { return {&fl, &fr, &rl, &rr}; }

    void setAllConnected(bool connected)
    {
        for (auto* inv : all())
        {
            inv->status.is_inverter_connected = connected;
        }
    }

    void setAllDriveEnabled(bool enabled)
    {
        for (auto* inv : all())
        {
            inv->status.is_drive_enabled = enabled;
        }
    }

    static DrivetrainCommand_s torqueCommand(float torque_nm, bool enable)
    {
        return DrivetrainCommand_s{
            .control_mode = DrivetrainControlMode_e::TORQUE,
            .desired_torques = {torque_nm, torque_nm, torque_nm, torque_nm},
            .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f},
            .is_drive_enable_requested = enable};
    }

    static DrivetrainCommand_s speedCommand(float speed_rpm, bool enable)
    {
        return DrivetrainCommand_s{
            .control_mode = DrivetrainControlMode_e::SPEED,
            .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
            .desired_speeds = {speed_rpm, speed_rpm, speed_rpm, speed_rpm},
            .is_drive_enable_requested = enable};
    }

    DrivetrainStatus_s tick(const DrivetrainCommand_s& command, unsigned long dt_ms = 10)
    {
        now += dt_ms;
        return drivetrain.evaluateDrivetrain(command, now);
    }

    void goToHVPresent()
    {
        setAllConnected(true);
        tick(torqueCommand(0.0f, false));
        ASSERT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_ABSENT);
        is_hv_ok = true;
        tick(torqueCommand(0.0f, false));
        ASSERT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_PRESENT);
    }

    void goToReady()
    {
        goToHVPresent();
        tick(torqueCommand(0.0f, true));
        ASSERT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::WANTING_OK);
        setAllDriveEnabled(true);
        tick(torqueCommand(0.0f, true));
        ASSERT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::READY);
    }
};

/* ---------- State ladder ---------- */

TEST_F(DrivetrainTest, StartsNotConnected)
{
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::NOT_CONNECTED);
}

TEST_F(DrivetrainTest, StaysNotConnectedUntilAllFourInvertersConnect)
{
    fl.status.is_inverter_connected = true;
    fr.status.is_inverter_connected = true;
    rl.status.is_inverter_connected = true;
    tick(torqueCommand(0.0f, false));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::NOT_CONNECTED);

    rr.status.is_inverter_connected = true;
    tick(torqueCommand(0.0f, false));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_ABSENT);
}

TEST_F(DrivetrainTest, HVOkMovesToHVPresent)
{
    goToHVPresent();
}

TEST_F(DrivetrainTest, DriveEnableRequestMovesToWantingOkAndRequestsEnable)
{
    goToHVPresent();
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::WANTING_OK);
    for (auto* inv : all())
    {
        EXPECT_TRUE(inv->is_enable_requested);
    }
}

TEST_F(DrivetrainTest, AllInvertersConfirmingEnableMovesToReady)
{
    goToReady();
}

TEST_F(DrivetrainTest, WantingOkStaysUntilAllFourConfirm)
{
    goToHVPresent();
    tick(torqueCommand(0.0f, true));
    fl.status.is_drive_enabled = true;
    fr.status.is_drive_enabled = true;
    rl.status.is_drive_enabled = true;
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::WANTING_OK);
}

TEST_F(DrivetrainTest, WantingOkTimesOutBackToHVPresent)
{
    goToHVPresent();
    tick(torqueCommand(0.0f, true));
    ASSERT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::WANTING_OK);

    tick(torqueCommand(0.0f, true), drivetrain_default_params::DRIVE_ENABLE_TIMEOUT_MS + 1);
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_PRESENT);
}

TEST_F(DrivetrainTest, DroppingRequestInWantingOkReturnsToHVPresent)
{
    goToHVPresent();
    tick(torqueCommand(0.0f, true));
    tick(torqueCommand(0.0f, false));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_PRESENT);
}

TEST_F(DrivetrainTest, DroppingRequestInReadyDisablesAndIdles)
{
    goToReady();
    int idle_before = fl.idle_calls;
    tick(torqueCommand(0.0f, false));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_PRESENT);
    for (auto* inv : all())
    {
        EXPECT_FALSE(inv->is_enable_requested);
    }
    EXPECT_GT(fl.idle_calls, idle_before);
}

TEST_F(DrivetrainTest, InverterDroppingEnableLeavesReady)
{
    goToReady();
    rr.status.is_drive_enabled = false;
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_PRESENT);
}

TEST_F(DrivetrainTest, LosingHVInReadyGoesToHVAbsent)
{
    goToReady();
    is_hv_ok = false;
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::CONNECTED_HV_ABSENT);
}

TEST_F(DrivetrainTest, LosingAnInverterGoesToNotConnected)
{
    goToReady();
    fr.status.is_inverter_connected = false;
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::NOT_CONNECTED);
    for (auto* inv : all())
    {
        EXPECT_FALSE(inv->is_enable_requested);
    }
}

/* ---------- Faults ---------- */

TEST_F(DrivetrainTest, FaultCodeFromReadyGoesToFaultedAndDisables)
{
    goToReady();
    rl.status.is_fault_code_present = true;
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::FAULTED);
    for (auto* inv : all())
    {
        EXPECT_FALSE(inv->is_enable_requested);
    }
}

TEST_F(DrivetrainTest, FaultCodeWhileConnectingGoesToFaulted)
{
    rl.status.is_inverter_connected = true;
    rl.status.is_fault_code_present = true;
    tick(torqueCommand(0.0f, false));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::FAULTED);
}

TEST_F(DrivetrainTest, FaultOnDisconnectedInverterIsIgnored)
{
    // A stale fault code from an inverter we are not hearing from should not latch FAULTED
    rl.status.is_inverter_connected = false;
    rl.status.is_fault_code_present = true;
    tick(torqueCommand(0.0f, false));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::NOT_CONNECTED);
}

TEST_F(DrivetrainTest, ClearedFaultRestartsFromNotConnected)
{
    goToReady();
    rl.status.is_fault_code_present = true;
    tick(torqueCommand(0.0f, true));
    ASSERT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::FAULTED);

    rl.status.is_fault_code_present = false;
    tick(torqueCommand(0.0f, true));
    EXPECT_EQ(drivetrain.getCurrentState(), DrivetrainState_e::NOT_CONNECTED);
}

/* ---------- Commands ---------- */

TEST_F(DrivetrainTest, TorqueIsAppliedPerCornerOnlyWhenReady)
{
    goToReady();
    DrivetrainCommand_s cmd = torqueCommand(0.0f, true);
    cmd.desired_torques = {1.0f, 2.0f, 3.0f, 4.0f};
    DrivetrainStatus_s status = tick(cmd);

    EXPECT_EQ(status.cmd_response, DrivetrainCmdResponse_e::COMMAND_OK);
    EXPECT_FLOAT_EQ(fl.last_torque_nm, 1.0f);
    EXPECT_FLOAT_EQ(fr.last_torque_nm, 2.0f);
    EXPECT_FLOAT_EQ(rl.last_torque_nm, 3.0f);
    EXPECT_FLOAT_EQ(rr.last_torque_nm, 4.0f);
}

TEST_F(DrivetrainTest, NonzeroTorqueWhenNotReadyIsRejectedAndNotApplied)
{
    goToHVPresent();
    DrivetrainStatus_s status = tick(torqueCommand(10.0f, false));
    EXPECT_EQ(status.cmd_response, DrivetrainCmdResponse_e::COMMAND_INVALID);
    for (auto* inv : all())
    {
        EXPECT_TRUE(std::isnan(inv->last_torque_nm));
    }
}

TEST_F(DrivetrainTest, CommandWhileNotConnectedReportsNotConnected)
{
    DrivetrainStatus_s status = tick(torqueCommand(0.0f, false));
    EXPECT_EQ(status.cmd_response, DrivetrainCmdResponse_e::CANNOT_SEND_NOT_CONNECTED);
}

TEST_F(DrivetrainTest, NonFiniteTorqueIdlesAndIsRejected)
{
    goToReady();
    tick(torqueCommand(3.0f, true));
    int idle_before = fl.idle_calls;

    DrivetrainCommand_s cmd = torqueCommand(10.0f, true);
    cmd.desired_torques.RR = NAN;
    DrivetrainStatus_s status = tick(cmd);

    // One bad corner idles every corner rather than applying the other three
    EXPECT_EQ(status.cmd_response, DrivetrainCmdResponse_e::COMMAND_INVALID);
    EXPECT_GT(fl.idle_calls, idle_before);
    EXPECT_FLOAT_EQ(fl.last_torque_nm, 3.0f);
    EXPECT_FALSE(std::isnan(rr.last_torque_nm));
}

TEST_F(DrivetrainTest, SpeedCommandIsAppliedInSpeedMode)
{
    goToReady();
    tick(speedCommand(1500.0f, true));
    for (auto* inv : all())
    {
        EXPECT_FLOAT_EQ(inv->last_speed_rpm, 1500.0f);
    }
    tick(speedCommand(1500.0f, true));
    EXPECT_EQ(fl.last_sent_mode, DrivetrainControlMode_e::SPEED);
}

TEST_F(DrivetrainTest, CommandsAreSentToEveryInverterEveryTickInEveryState)
{
    tick(torqueCommand(0.0f, false)); // NOT_CONNECTED
    goToReady();
    for (auto* inv : all())
    {
        EXPECT_EQ(inv->send_calls, static_cast<int>(now / 10));
    }
}

/* ---------- Control mode mismatch ---------- */

TEST_F(DrivetrainTest, ModeMismatchIgnoredWithinThreshold)
{
    goToReady();
    fl.is_mode_matching = false;
    DrivetrainStatus_s status = tick(torqueCommand(0.0f, true), 10);
    EXPECT_FALSE(status.is_control_mode_mismatch_detected);
}

TEST_F(DrivetrainTest, ModeMismatchFlaggedAfterThreshold)
{
    goToReady();
    fl.is_mode_matching = false;
    DrivetrainStatus_s status = tick(torqueCommand(0.0f, true), drivetrain_default_params::CONTROL_MODE_MISMATCH_THRESHOLD_MS + 1);
    EXPECT_TRUE(status.is_control_mode_mismatch_detected);
}

TEST_F(DrivetrainTest, ModeMismatchNotReportedOutsideReady)
{
    goToHVPresent();
    fl.is_mode_matching = false;
    DrivetrainStatus_s status = tick(torqueCommand(0.0f, false), 1000);
    EXPECT_FALSE(status.is_control_mode_mismatch_detected);
}

/* ---------- Misc ---------- */

TEST_F(DrivetrainTest, StatusReportsPerCornerInverterStatuses)
{
    setAllConnected(true);
    rr.status.dc_bus_voltage = 512.0f;
    DrivetrainStatus_s status = tick(torqueCommand(0.0f, false));
    EXPECT_TRUE(status.are_all_inverters_connected);
    EXPECT_FLOAT_EQ(status.inverter_statuses.RR.dc_bus_voltage, 512.0f);
}

TEST_F(DrivetrainTest, StateNameIsNeverNull)
{
    EXPECT_NE(drivetrain.getStateName(), nullptr);
    goToReady();
    EXPECT_NE(drivetrain.getStateName(), nullptr);
}

#endif // TEST_DRIVETRAIN_HPP
