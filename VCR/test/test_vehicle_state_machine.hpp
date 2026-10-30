#ifndef TEST_VEHICLE_STATE_MACHINE_HPP
#define TEST_VEHICLE_STATE_MACHINE_HPP

#include "VehicleStateMachine.hpp"

#include <gtest/gtest.h>
#include <vector>

/**
 * @brief Stands in for every system the VSM talks to. Tests set the inputs and read back what the VSM did.
 */
class MockVehicleStateMachine
{
  public:
    struct DrivetrainCommandCall
    {
        bool is_drive_enable_requested;
        bool is_torque_allowed;
    };

    /* Inputs the VSM reads */
    bool is_latched = false;
    bool is_rtd_pressed = false;
    bool is_rss_go_pressed = false;
    bool is_brake_pressed = false;
    bool is_pedals_timed_out = false;
    bool is_steering_timed_out = false;
    bool is_pedals_recalibrate_pressed = false;
    bool is_steering_recalibrate_pressed = false;
    bool is_drivetrain_faulted = false;
    bool is_drivetrain_not_connected = false;
    bool is_drivetrain_ready = false;

    /* What the VSM did */
    int set_motors_idle_calls = 0;
    int start_buzzer_calls = 0;
    int reset_pedals_heartbeat_calls = 0;
    int reset_steering_heartbeat_calls = 0;
    int recalibrate_pedals_msgs = 0;
    int recalibrate_steering_msgs = 0;
    std::vector<DrivetrainCommandCall> drivetrain_commands; // one entry per handleDrivetrainCommand call

    void setMotorsIdle() { set_motors_idle_calls++; }
    bool isVehicleLatched() { return is_latched; }
    bool isRTDPressed() { return is_rtd_pressed; }
    void startBuzzer() { start_buzzer_calls++; }
    bool isBrakePressed() { return is_brake_pressed; }
    bool isPedalsTimedOut() { return is_pedals_timed_out; }
    bool isSteeringTimedOut() { return is_steering_timed_out; }
    void resetPedalsHeartbeat() { reset_pedals_heartbeat_calls++; }
    void resetSteeringHeartbeat() { reset_steering_heartbeat_calls++; }
    bool isPedalsRecalibratePressed() { return is_pedals_recalibrate_pressed; }
    bool isSteeringRecalibratePressed() { return is_steering_recalibrate_pressed; }
    void sendRecalibratePedalsMessage() { recalibrate_pedals_msgs++; }
    void sendRecalibrateSteeringMessage() { recalibrate_steering_msgs++; }
    bool isDrivetrainFaulted() { return is_drivetrain_faulted; }
    bool isDrivetrainNotConnected() { return is_drivetrain_not_connected; }
    bool isDrivetrainReady() { return is_drivetrain_ready; }
    void handleDrivetrainCommand(bool is_drive_enable_requested, bool is_torque_allowed, unsigned long curr_millis)
    {
        drivetrain_commands.push_back({is_drive_enable_requested, is_torque_allowed});
    }
    bool isRSSGoButtonPressed() { return is_rss_go_pressed; }
};

class VehicleStateMachineTest : public ::testing::Test
{
  protected:
    MockVehicleStateMachine mock_vehicle_state_machine;
    VehicleStateMachine vehicle_state_machine{
        etl::delegate<void()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::setMotorsIdle>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isVehicleLatched>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isRTDPressed>(mock_vehicle_state_machine),
        etl::delegate<void()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::startBuzzer>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isBrakePressed>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isPedalsTimedOut>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isSteeringTimedOut>(mock_vehicle_state_machine),
        etl::delegate<void()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::resetPedalsHeartbeat>(mock_vehicle_state_machine),
        etl::delegate<void()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::resetSteeringHeartbeat>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isPedalsRecalibratePressed>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isSteeringRecalibratePressed>(mock_vehicle_state_machine),
        etl::delegate<void()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::sendRecalibratePedalsMessage>(mock_vehicle_state_machine),
        etl::delegate<void()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::sendRecalibrateSteeringMessage>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isDrivetrainFaulted>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isDrivetrainNotConnected>(mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isDrivetrainReady>(mock_vehicle_state_machine),
        etl::delegate<void(bool, bool, unsigned long)>::create<MockVehicleStateMachine, &MockVehicleStateMachine::handleDrivetrainCommand>(
            mock_vehicle_state_machine),
        etl::delegate<bool()>::create<MockVehicleStateMachine, &MockVehicleStateMachine::isRSSGoButtonPressed>(mock_vehicle_state_machine)};

    unsigned long now = 0;

    VehicleState_e tick(unsigned long dt_ms = 10)
    {
        now += dt_ms;
        return vehicle_state_machine.tickStateMachine(now);
    }

    MockVehicleStateMachine::DrivetrainCommandCall lastDrivetrainCommand()
    {
        EXPECT_FALSE(mock_vehicle_state_machine.drivetrain_commands.empty());
        return mock_vehicle_state_machine.drivetrain_commands.empty() ? MockVehicleStateMachine::DrivetrainCommandCall{true, true}
                                                                      : mock_vehicle_state_machine.drivetrain_commands.back();
    }

    void goToTractiveSystemActive()
    {
        mock_vehicle_state_machine.is_latched = true;
        ASSERT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    }

    void goToWantingReadyToDrive()
    {
        goToTractiveSystemActive();
        mock_vehicle_state_machine.is_drivetrain_ready = true;
        ASSERT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
    }

    void goToReadyToDrive()
    {
        goToWantingReadyToDrive();
        mock_vehicle_state_machine.is_rtd_pressed = true;
        mock_vehicle_state_machine.is_brake_pressed = true;
        ASSERT_EQ(tick(), VehicleState_e::READY_TO_DRIVE);
        mock_vehicle_state_machine.is_rtd_pressed = false;
        mock_vehicle_state_machine.is_brake_pressed = false;
    }
};

/* ---------- TRACTIVE_SYSTEM_NOT_ACTIVE ---------- */

TEST_F(VehicleStateMachineTest, StartsInTractiveSystemNotActive)
{
    EXPECT_EQ(vehicle_state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

TEST_F(VehicleStateMachineTest, NotActiveIdlesAndKeepsDriveDisabled)
{
    tick();
    EXPECT_EQ(vehicle_state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    EXPECT_EQ(mock_vehicle_state_machine.set_motors_idle_calls, 1);
    EXPECT_FALSE(lastDrivetrainCommand().is_drive_enable_requested);
    EXPECT_FALSE(lastDrivetrainCommand().is_torque_allowed);
}

TEST_F(VehicleStateMachineTest, DrivetrainNotConnectedIsNotAnErrorBeforeLatching)
{
    // The inverters have not reported yet at boot
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

TEST_F(VehicleStateMachineTest, LatchingMovesToTractiveSystemActive)
{
    mock_vehicle_state_machine.is_latched = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, FaultedDrivetrainDoesNotBlockLatching)
{
    mock_vehicle_state_machine.is_drivetrain_faulted = true;
    mock_vehicle_state_machine.is_latched = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

/* ---------- TRACTIVE_SYSTEM_ACTIVE ---------- */

TEST_F(VehicleStateMachineTest, ActiveRequestsDriveEnableWithoutTorque)
{
    goToTractiveSystemActive();
    tick();
    EXPECT_TRUE(lastDrivetrainCommand().is_drive_enable_requested);
    EXPECT_FALSE(lastDrivetrainCommand().is_torque_allowed);
}

TEST_F(VehicleStateMachineTest, ActiveWaitsForDrivetrainReady)
{
    goToTractiveSystemActive();
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, ActiveMovesToWantingReadyToDriveOnceDrivetrainReady)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_drivetrain_ready = true;
    EXPECT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
}

TEST_F(VehicleStateMachineTest, RTDPressInActiveDoesNothingUntilDrivetrainReady)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_rtd_pressed = true;
    mock_vehicle_state_machine.is_brake_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, FaultedDrivetrainStaysInActive)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_drivetrain_faulted = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, UnlatchingFromActiveReturnsToNotActive)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_latched = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

TEST_F(VehicleStateMachineTest, DrivetrainNotConnectedFromActiveGoesToError)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    EXPECT_EQ(tick(), VehicleState_e::ERROR);
}

TEST_F(VehicleStateMachineTest, RecalibrationIsIgnoredWhileLatched)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = true;
    mock_vehicle_state_machine.is_steering_recalibrate_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

/* ---------- WANTING_READY_TO_DRIVE ---------- */

TEST_F(VehicleStateMachineTest, WantingReadyToDriveKeepsDriveEnabledWithoutTorque)
{
    goToWantingReadyToDrive();
    tick();
    EXPECT_TRUE(lastDrivetrainCommand().is_drive_enable_requested);
    EXPECT_FALSE(lastDrivetrainCommand().is_torque_allowed);
}

TEST_F(VehicleStateMachineTest, StartButtonAndBrakeEnterReadyToDrive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_rtd_pressed = true;
    mock_vehicle_state_machine.is_brake_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::READY_TO_DRIVE);
}

TEST_F(VehicleStateMachineTest, RSSGoButtonAndBrakeEnterReadyToDrive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_rss_go_pressed = true;
    mock_vehicle_state_machine.is_brake_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::READY_TO_DRIVE);
}

TEST_F(VehicleStateMachineTest, StartButtonWithoutBrakeDoesNotEnterReadyToDrive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_rtd_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
}

TEST_F(VehicleStateMachineTest, RSSGoButtonWithoutBrakeDoesNotEnterReadyToDrive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_rss_go_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
}

TEST_F(VehicleStateMachineTest, BrakeAloneDoesNotEnterReadyToDrive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_brake_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
}

TEST_F(VehicleStateMachineTest, DrivetrainLeavingReadyReturnsWantingToActive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_ready = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, FaultedDrivetrainReturnsWantingToActive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_faulted = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, UnlatchingFromWantingReturnsToNotActive)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_latched = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

TEST_F(VehicleStateMachineTest, DrivetrainNotConnectedFromWantingGoesToError)
{
    goToWantingReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    EXPECT_EQ(tick(), VehicleState_e::ERROR);
}

/* ---------- READY_TO_DRIVE ---------- */

TEST_F(VehicleStateMachineTest, EnteringReadyToDriveSoundsBuzzerAndResetsHeartbeats)
{
    goToReadyToDrive();
    EXPECT_EQ(mock_vehicle_state_machine.start_buzzer_calls, 1);
    EXPECT_EQ(mock_vehicle_state_machine.reset_pedals_heartbeat_calls, 1);
    EXPECT_EQ(mock_vehicle_state_machine.reset_steering_heartbeat_calls, 1);
}

TEST_F(VehicleStateMachineTest, BuzzerOnlySoundsOnEnteringReadyToDrive)
{
    goToWantingReadyToDrive();
    EXPECT_EQ(mock_vehicle_state_machine.start_buzzer_calls, 0);
}

TEST_F(VehicleStateMachineTest, ReadyToDriveAllowsTorque)
{
    goToReadyToDrive();
    tick();
    EXPECT_TRUE(lastDrivetrainCommand().is_drive_enable_requested);
    EXPECT_TRUE(lastDrivetrainCommand().is_torque_allowed);
}

TEST_F(VehicleStateMachineTest, TorqueIsOnlyEverAllowedInReadyToDrive)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_pedals_timed_out = true;
    ASSERT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    mock_vehicle_state_machine.drivetrain_commands.clear();
    mock_vehicle_state_machine.is_pedals_timed_out = false;
    mock_vehicle_state_machine.is_drivetrain_ready = false;
    tick();
    tick();
    for (const auto& call : mock_vehicle_state_machine.drivetrain_commands)
    {
        EXPECT_FALSE(call.is_torque_allowed);
    }
}

TEST_F(VehicleStateMachineTest, PedalsTimeoutDropsToActive)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_pedals_timed_out = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, SteeringTimeoutDropsToActive)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_steering_timed_out = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, FaultedDrivetrainDropsReadyToDriveToActive)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_faulted = true;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, DrivetrainLeavingReadyDropsReadyToDriveToActive)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_ready = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, UnlatchingFromReadyToDriveReturnsToNotActive)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_latched = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

TEST_F(VehicleStateMachineTest, DrivetrainNotConnectedFromReadyToDriveGoesToError)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    EXPECT_EQ(tick(), VehicleState_e::ERROR);
}

TEST_F(VehicleStateMachineTest, ReEnteringReadyToDriveNeedsAnotherDriverRequest)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_pedals_timed_out = true;
    ASSERT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    mock_vehicle_state_machine.is_pedals_timed_out = false;

    // Drivetrain is still READY, so we go back to waiting for the driver, not straight back to RTD
    EXPECT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
    EXPECT_EQ(tick(), VehicleState_e::WANTING_READY_TO_DRIVE);
}

/* ---------- ERROR ---------- */

TEST_F(VehicleStateMachineTest, ErrorIdlesAndKeepsDriveDisabled)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    ASSERT_EQ(tick(), VehicleState_e::ERROR);
    int idle_before = mock_vehicle_state_machine.set_motors_idle_calls;
    tick();
    EXPECT_GT(mock_vehicle_state_machine.set_motors_idle_calls, idle_before);
    EXPECT_FALSE(lastDrivetrainCommand().is_drive_enable_requested);
    EXPECT_FALSE(lastDrivetrainCommand().is_torque_allowed);
}

TEST_F(VehicleStateMachineTest, ErrorWaitsWhileInvertersAreDisconnected)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    ASSERT_EQ(tick(), VehicleState_e::ERROR);
    EXPECT_EQ(tick(1000), VehicleState_e::ERROR);
    EXPECT_EQ(tick(10000), VehicleState_e::ERROR);
}

TEST_F(VehicleStateMachineTest, ErrorRecoversToNotActiveOnceInvertersReturn)
{
    goToTractiveSystemActive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    ASSERT_EQ(tick(), VehicleState_e::ERROR);
    mock_vehicle_state_machine.is_drivetrain_not_connected = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);

    // Still latched, so it climbs straight back to TRACTIVE_SYSTEM_ACTIVE
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
}

TEST_F(VehicleStateMachineTest, ErrorNeverResumesReadyToDriveOnItsOwn)
{
    goToReadyToDrive();
    mock_vehicle_state_machine.is_drivetrain_not_connected = true;
    ASSERT_EQ(tick(), VehicleState_e::ERROR);
    mock_vehicle_state_machine.is_drivetrain_not_connected = false;
    mock_vehicle_state_machine.is_drivetrain_ready = true;
    for (int i = 0; i < 10; i++)
    {
        EXPECT_NE(tick(), VehicleState_e::READY_TO_DRIVE);
    }
}

/* ---------- Pedals recalibration ---------- */

TEST_F(VehicleStateMachineTest, PedalsRecalibratePressEntersWanting)
{
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
}

TEST_F(VehicleStateMachineTest, PedalsRecalibrationAllowedWhileDrivetrainFaulted)
{
    mock_vehicle_state_machine.is_drivetrain_faulted = true;
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = true;
    EXPECT_EQ(tick(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
}

TEST_F(VehicleStateMachineTest, ShortPedalsRecalibratePressReturnsToNotActive)
{
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = true;
    ASSERT_EQ(tick(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    EXPECT_EQ(mock_vehicle_state_machine.recalibrate_pedals_msgs, 0);
}

TEST_F(VehicleStateMachineTest, PedalsRecalibrateHeldPastDebounceRecalibrates)
{
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = true;
    ASSERT_EQ(tick(), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
    EXPECT_EQ(tick(1000), VehicleState_e::WANTING_RECALIBRATE_PEDALS);
    EXPECT_EQ(tick(2500), VehicleState_e::RECALIBRATING_PEDALS);

    tick();
    EXPECT_GE(mock_vehicle_state_machine.recalibrate_pedals_msgs, 1);

    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

/* ---------- Steering recalibration ---------- */

TEST_F(VehicleStateMachineTest, ShortSteeringRecalibratePressReturnsToNotActive)
{
    mock_vehicle_state_machine.is_steering_recalibrate_pressed = true;
    ASSERT_EQ(tick(), VehicleState_e::WANTING_RECALIBRATE_STEERING);
    mock_vehicle_state_machine.is_steering_recalibrate_pressed = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    EXPECT_EQ(mock_vehicle_state_machine.recalibrate_steering_msgs, 0);
}

TEST_F(VehicleStateMachineTest, SteeringRecalibrateHeldPastDebounceRecalibrates)
{
    mock_vehicle_state_machine.is_steering_recalibrate_pressed = true;
    ASSERT_EQ(tick(), VehicleState_e::WANTING_RECALIBRATE_STEERING);
    EXPECT_EQ(tick(3500), VehicleState_e::RECALIBRATING_STEERING);

    tick();
    EXPECT_GE(mock_vehicle_state_machine.recalibrate_steering_msgs, 1);

    mock_vehicle_state_machine.is_steering_recalibrate_pressed = false;
    EXPECT_EQ(tick(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

TEST_F(VehicleStateMachineTest, RecalibrationNeverEnablesTheDrivetrain)
{
    mock_vehicle_state_machine.is_pedals_recalibrate_pressed = true;
    tick();
    tick(4000);
    tick();
    for (const auto& call : mock_vehicle_state_machine.drivetrain_commands)
    {
        EXPECT_FALSE(call.is_drive_enable_requested);
        EXPECT_FALSE(call.is_torque_allowed);
    }
}

#endif // TEST_VEHICLE_STATE_MACHINE_HPP
