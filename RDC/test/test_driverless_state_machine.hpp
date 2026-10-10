#include "DriverlessStateMachine.hpp"
#include <gtest/gtest.h>

bool dsms_on = false;
VehicleState_e vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
bool startup_no_ts_ok = false;
bool startup_ts_ok = false;
bool ebs_pressure_ok = false;

int startup_no_ts_calls = 0;
int startup_ts_calls = 0;
int reset_no_ts_calls = 0;
int reset_ts_calls = 0;

void reset_state_machine_inputs()
{
    dsms_on = false;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    startup_no_ts_ok = false;
    startup_ts_ok = false;
    ebs_pressure_ok = false;
    startup_no_ts_calls = 0;
    startup_ts_calls = 0;
    reset_no_ts_calls = 0;
    reset_ts_calls = 0;
}

etl::delegate<bool()> mock_dsms_on = etl::delegate<bool()>::create([]() -> bool { return dsms_on; });

etl::delegate<VehicleState_e()> mock_vehicle_state = etl::delegate<VehicleState_e()>::create([]() -> VehicleState_e { return vehicle_state; });

etl::delegate<bool()> mock_startup_no_ts = etl::delegate<bool()>::create(
    []() -> bool
    {
        startup_no_ts_calls++;
        return startup_no_ts_ok;
    });

etl::delegate<bool()> mock_startup_ts = etl::delegate<bool()>::create(
    []() -> bool
    {
        startup_ts_calls++;
        return startup_ts_ok;
    });

etl::delegate<bool()> mock_ebs_pressure_ok = etl::delegate<bool()>::create([]() -> bool { return ebs_pressure_ok; });

etl::delegate<void()> mock_reset_no_ts = etl::delegate<void()>::create([]() -> void { reset_no_ts_calls++; });

etl::delegate<void()> mock_reset_ts = etl::delegate<void()>::create([]() -> void { reset_ts_calls++; });

DriverlessStateMachine make_driverless_state_machine()
{
    return DriverlessStateMachine(mock_dsms_on, mock_vehicle_state, mock_startup_no_ts, mock_startup_ts, mock_ebs_pressure_ok, mock_reset_no_ts, mock_reset_ts);
}

TEST(DriverlessStateMachine, InitialStateIsOff)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::OFF);
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::OFF);
    ASSERT_EQ(startup_no_ts_calls, 0);
    ASSERT_EQ(reset_no_ts_calls, 0);
    ASSERT_EQ(reset_ts_calls, 0);
}

TEST(DriverlessStateMachine, DsmsOnEntersStartupNoTs)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    dsms_on = true;
    ASSERT_EQ(state_machine.tickStateMachine(10), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_EQ(reset_no_ts_calls, 1);
    ASSERT_EQ(startup_no_ts_calls, 0);

    // Entry resets once. The startup check runs on the next evaluation, not on the transition tick.
    state_machine.tickStateMachine(20);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_EQ(reset_no_ts_calls, 1);
    ASSERT_EQ(startup_no_ts_calls, 1);

    // DSMS dropping does not leave the startup state.
    dsms_on = false;
    state_machine.tickStateMachine(30);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_EQ(startup_no_ts_calls, 2);
}

TEST(DriverlessStateMachine, StartupNoTsRequiresCheckAndTractiveSystemActive)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    dsms_on = true;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);

    // The no-TS check is evaluated even when the vehicle is not in tractive system active.
    startup_no_ts_ok = false;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_EQ(startup_no_ts_calls, 1);

    startup_no_ts_ok = true;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);

    vehicle_state = VehicleState_e::WANTING_READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);

    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_EQ(reset_ts_calls, 0);

    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(reset_ts_calls, 1);
    ASSERT_EQ(startup_ts_calls, 0);
    ASSERT_EQ(reset_no_ts_calls, 1);
}

TEST(DriverlessStateMachine, StartupTsActiveRequiresCheckAndTractiveSystemActive)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    dsms_on = true;
    startup_no_ts_ok = true;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(startup_ts_calls, 0);

    // Vehicle state is checked first, so a non-active state does not run the TS startup check.
    startup_ts_ok = true;
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(startup_ts_calls, 0);

    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(startup_ts_calls, 0);

    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    startup_ts_ok = false;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(startup_ts_calls, 1);

    startup_ts_ok = true;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);
    ASSERT_EQ(startup_ts_calls, 2);
    ASSERT_EQ(reset_ts_calls, 1);
}

TEST(DriverlessStateMachine, ReadyEntersDrivingOrEmergency)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    dsms_on = true;
    startup_no_ts_ok = true;
    startup_ts_ok = true;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    state_machine.tickStateMachine(0);
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);

    ebs_pressure_ok = true;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);

    vehicle_state = VehicleState_e::WANTING_READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);

    // Lost EBS pressure goes to emergency even if the car is ready to drive.
    ebs_pressure_ok = false;
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::EMERGENCY);

    ebs_pressure_ok = true;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::EMERGENCY);

    DriverlessStateMachine driving_machine = make_driverless_state_machine();
    ebs_pressure_ok = true;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    driving_machine.tickStateMachine(0);
    driving_machine.tickStateMachine(0);
    driving_machine.tickStateMachine(0);
    ASSERT_EQ(driving_machine.getState(), DriverlessSystemState_e::READY);

    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    driving_machine.tickStateMachine(0);
    ASSERT_EQ(driving_machine.getState(), DriverlessSystemState_e::DRIVING);

    ebs_pressure_ok = false;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    dsms_on = false;
    driving_machine.tickStateMachine(0);
    ASSERT_EQ(driving_machine.getState(), DriverlessSystemState_e::DRIVING);
}
