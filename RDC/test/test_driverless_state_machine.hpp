#include "DriverlessStateMachine.hpp"
#include <gtest/gtest.h>

// Status rules are DO.1.1. The DSF charts are only the order of the checks:
// mission, stored energy, watchdog, tractive system, then each brake path.
//
// DO.1.1.2: the emergency brake is "activated" only after the startup check
// has passed and its supply is then opened. Before that, a failed check is
// not Emergency. After that, loss of the monitored brake pressure is a DSB
// signal failure (DT.3.1.3.b, DT.3.2.2) and the status is Emergency.
// DSMS off while the emergency brake is not activated is Off, and the
// driverless system must not keep operating the brakes (DT.1.1.3).

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

void close_dsms_and_enter_startup(DriverlessStateMachine &state_machine)
{
    dsms_on = true;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::STARTUP_NO_TS);
}

void pass_no_ts_check_with_tractive_system(DriverlessStateMachine &state_machine)
{
    close_dsms_and_enter_startup(state_machine);
    startup_no_ts_ok = true;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
}

void pass_startup_into_ready(DriverlessStateMachine &state_machine)
{
    pass_no_ts_check_with_tractive_system(state_machine);
    startup_ts_ok = true;
    ebs_pressure_ok = true;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::READY);
}

TEST(DriverlessStateMachine, StaysOffUntilDsmsCloses)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::OFF);
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::OFF);
    ASSERT_EQ(startup_no_ts_calls, 0);
    ASSERT_EQ(startup_ts_calls, 0);
    ASSERT_EQ(reset_no_ts_calls, 0);
    ASSERT_EQ(reset_ts_calls, 0);
}

TEST(DriverlessStateMachine, DsmsOnStartsTheCheckAndDoesNotFinishItOnThatTick)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    close_dsms_and_enter_startup(state_machine);
    ASSERT_EQ(reset_no_ts_calls, 1);
    ASSERT_EQ(startup_no_ts_calls, 0);

    state_machine.tickStateMachine(20);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_EQ(reset_no_ts_calls, 1);
    ASSERT_EQ(startup_no_ts_calls, 1);
}

TEST(DriverlessStateMachine, DsmsOffDuringStartupIsOffAndStopsTheCheck)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    close_dsms_and_enter_startup(state_machine);
    state_machine.tickStateMachine(20);
    const int calls_before_dsms_opened = startup_no_ts_calls;

    dsms_on = false;
    ASSERT_EQ(state_machine.tickStateMachine(30), DriverlessSystemState_e::OFF);
    ASSERT_EQ(startup_no_ts_calls, calls_before_dsms_opened);
    ASSERT_EQ(state_machine.tickStateMachine(40), DriverlessSystemState_e::OFF);
    ASSERT_EQ(startup_ts_calls, 0);
}

TEST(DriverlessStateMachine, DsmsOffDuringTractiveStartupIsOffAndStopsTheCheck)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    pass_no_ts_check_with_tractive_system(state_machine);
    const int calls_before_dsms_opened = startup_ts_calls;

    dsms_on = false;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::OFF);
    ASSERT_EQ(startup_ts_calls, calls_before_dsms_opened);
}

TEST(DriverlessStateMachine, ReadyRequiresBothChecksAndTractiveSystemActive)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    close_dsms_and_enter_startup(state_machine);

    startup_no_ts_ok = false;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_NO_TS);

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
    ASSERT_EQ(startup_ts_calls, 0);

    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(reset_ts_calls, 1);
    ASSERT_EQ(startup_ts_calls, 0);

    startup_ts_ok = false;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_NE(state_machine.getState(), DriverlessSystemState_e::EMERGENCY);

    startup_ts_ok = true;
    ebs_pressure_ok = true;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);
}

TEST(DriverlessStateMachine, TractiveCheckDoesNotRunUntilTheTractiveSystemIsActive)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    pass_no_ts_check_with_tractive_system(state_machine);
    ASSERT_EQ(startup_ts_calls, 0);

    startup_ts_ok = true;
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(startup_ts_calls, 0);

    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_EQ(startup_ts_calls, 0);
}

TEST(DriverlessStateMachine, IncompleteStartupIsNotEmergency)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    close_dsms_and_enter_startup(state_machine);
    startup_no_ts_ok = false;
    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_NE(state_machine.getState(), DriverlessSystemState_e::EMERGENCY);
    ASSERT_NE(state_machine.getState(), DriverlessSystemState_e::READY);
    ASSERT_NE(state_machine.getState(), DriverlessSystemState_e::DRIVING);
}

TEST(DriverlessStateMachine, ReadyGoesDrivingOnlyWhenReadyToDriveAndBrakesHold)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    pass_startup_into_ready(state_machine);

    vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);

    vehicle_state = VehicleState_e::WANTING_READY_TO_DRIVE;
    state_machine.tickStateMachine(0);
    ASSERT_EQ(state_machine.getState(), DriverlessSystemState_e::READY);

    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::DRIVING);
}

TEST(DriverlessStateMachine, BrakePressureLossAfterReadyIsEmergencyAndStaysThere)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    pass_startup_into_ready(state_machine);
    ebs_pressure_ok = false;
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::EMERGENCY);

    ebs_pressure_ok = true;
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::EMERGENCY);
}

TEST(DriverlessStateMachine, BrakePressureLossWhileDrivingIsEmergency)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();

    pass_startup_into_ready(state_machine);
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::DRIVING);

    ebs_pressure_ok = false;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::EMERGENCY);
}

TEST(DriverlessStateMachine, DsmsOffWhileReadyIsOff)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();
    pass_startup_into_ready(state_machine);

    dsms_on = false;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::OFF);
}

//TODO: Think about this with more sleep.
TEST(DriverlessStateMachine, DsmsOffWhileDrivingIsOff)
{
    reset_state_machine_inputs();
    DriverlessStateMachine state_machine = make_driverless_state_machine();
    pass_startup_into_ready(state_machine);
    vehicle_state = VehicleState_e::READY_TO_DRIVE;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::DRIVING);

    dsms_on = false;
    ASSERT_EQ(state_machine.tickStateMachine(0), DriverlessSystemState_e::OFF);
}
