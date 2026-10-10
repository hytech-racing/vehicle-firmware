#include "DriverlessStateMachine.h"
#include "DriverlessSystem.h"
#include "RDC_Constants.h"
#include <gtest/gtest.h>
#include <stddef.h>
#include <vector>

struct FdcCommand
{
    bool toggle;
    uint8_t solenoid;
};

DriverlessMission_e mission = DriverlessMission_e::OFF;
EBSData_s ebs_data = {};
bool supervisor_ok = false;
unsigned long current_millis = 0;
BrakeFluidPressureData_s brake_data = {};
std::vector<FdcCommand> fdc_commands;

bool eval_dsms_on = false;
VehicleState_e eval_vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;

void reset_driverless_inputs()
{
    mission = DriverlessMission_e::ACCELERATION;
    ebs_data = {};
    ebs_data.pressure_1 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI + 1;
    ebs_data.pressure_2 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI + 1;
    supervisor_ok = false;
    current_millis = 0;
    brake_data = {};
    fdc_commands.clear();
    eval_dsms_on = false;
    eval_vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
}

void record_fdc_command(bool toggle, uint8_t solenoid)
{
    fdc_commands.push_back(FdcCommand{toggle, solenoid});
}

#define ASSERT_FDC(index, expected_toggle, expected_solenoid)                                                                                                                                          \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        ASSERT_LT(static_cast<size_t>(index), fdc_commands.size());                                                                                                                                    \
        ASSERT_EQ(fdc_commands[static_cast<size_t>(index)].toggle, (expected_toggle));                                                                                                                 \
        ASSERT_EQ(fdc_commands[static_cast<size_t>(index)].solenoid, static_cast<uint8_t>(expected_solenoid));                                                                                         \
    } while (0)

etl::delegate<void(bool, uint8_t)> mock_command_fdc = etl::delegate<void(bool, uint8_t)>::create<record_fdc_command>();

etl::delegate<DriverlessMission_e()> mock_mission = etl::delegate<DriverlessMission_e()>::create([]() -> DriverlessMission_e { return mission; });

etl::delegate<EBSData_s()> mock_ebs_data = etl::delegate<EBSData_s()>::create([]() -> EBSData_s { return ebs_data; });

etl::delegate<bool()> mock_supervisor_ok = etl::delegate<bool()>::create([]() -> bool { return supervisor_ok; });

etl::delegate<unsigned long()> mock_millis = etl::delegate<unsigned long()>::create([]() -> unsigned long { return current_millis; });

etl::delegate<BrakeFluidPressureData_s()> mock_brake_data = etl::delegate<BrakeFluidPressureData_s()>::create([]() -> BrakeFluidPressureData_s { return brake_data; });

etl::delegate<bool()> mock_eval_dsms_on = etl::delegate<bool()>::create([]() -> bool { return eval_dsms_on; });

etl::delegate<VehicleState_e()> mock_eval_vehicle_state = etl::delegate<VehicleState_e()>::create([]() -> VehicleState_e { return eval_vehicle_state; });

etl::delegate<bool()> mock_eval_ebs_pressure_ok =
    etl::delegate<bool()>::create([]() -> bool { return ebs_data.pressure_1 > RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI && ebs_data.pressure_2 > RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI; });

DriverlessSystem make_driverless_system()
{
    return DriverlessSystem(mock_command_fdc, mock_mission, mock_ebs_data, mock_supervisor_ok, mock_millis, mock_brake_data);
}

void set_brake_pressure(uint16_t front, uint16_t rear)
{
    brake_data.brake_fluid_pressure_data_front = front;
    brake_data.brake_fluid_pressure_data_rear = rear;
}

TEST(DriverlessSystem, StartupNoTsRejectsUnselectedMissionAndLowAirPressure)
{
    reset_driverless_inputs();
    DriverlessSystem system = make_driverless_system();

    mission = DriverlessMission_e::OFF;
    supervisor_ok = true;
    current_millis = 1000;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_TRUE(fdc_commands.empty());

    mission = DriverlessMission_e::ACCELERATION;
    ebs_data.pressure_1 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI - 1;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_TRUE(fdc_commands.empty());

    ebs_data.pressure_1 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI + 1;
    ebs_data.pressure_2 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI - 1;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_TRUE(fdc_commands.empty());

    // Pressure equal to the threshold passes the air-pressure gate and reaches the watchdog check.
    ebs_data.pressure_1 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI;
    ebs_data.pressure_2 = RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_EQ(fdc_commands.size(), 1u);
    ASSERT_FDC(0, false, 0);
}

TEST(DriverlessSystem, StartupNoTsRequiresSupervisorToDropAfterDelay)
{
    reset_driverless_inputs();
    DriverlessSystem system = make_driverless_system();

    current_millis = 50000;
    supervisor_ok = false;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_TRUE(fdc_commands.empty());

    supervisor_ok = true;
    current_millis = 1000;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_FDC(0, false, 0);

    // While the supervisor is still ok the timer restarts, and the check keeps the watchdog stopped.
    current_millis = 1000 + RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_EQ(fdc_commands.size(), 2u);
    ASSERT_FDC(1, false, 0);

    supervisor_ok = false;
    const unsigned long supervisor_dropped_at = current_millis;
    current_millis = supervisor_dropped_at + RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_EQ(fdc_commands.size(), 2u);

    current_millis = supervisor_dropped_at + RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS + 1;
    ASSERT_TRUE(system.startupCheckNoTS());
    ASSERT_EQ(fdc_commands.size(), 3u);
    ASSERT_FDC(2, true, 0);

    // A later sample that still shows the supervisor failed stays passed and commands the watchdog back on.
    current_millis += 10;
    ASSERT_TRUE(system.startupCheckNoTS());
    ASSERT_FDC(3, true, 0);
}

TEST(DriverlessSystem, ResetNoTsValuesRestartsWatchdogCheck)
{
    reset_driverless_inputs();
    DriverlessSystem system = make_driverless_system();

    supervisor_ok = true;
    current_millis = 1000;
    ASSERT_FALSE(system.startupCheckNoTS());

    supervisor_ok = false;
    current_millis = 1000 + RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS + 1;
    ASSERT_TRUE(system.startupCheckNoTS());

    system.resetNoTSValues();
    fdc_commands.clear();
    current_millis += RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS + 1;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_TRUE(fdc_commands.empty());

    supervisor_ok = true;
    ASSERT_FALSE(system.startupCheckNoTS());
    ASSERT_FDC(0, false, 0);

    supervisor_ok = false;
    current_millis += RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS;
    ASSERT_FALSE(system.startupCheckNoTS());
    current_millis += 1;
    ASSERT_TRUE(system.startupCheckNoTS());
    ASSERT_FDC(1, true, 0);
}

TEST(DriverlessSystem, StartupTsActiveChecksEachSolenoidInItsOwnWindow)
{
    reset_driverless_inputs();
    DriverlessSystem system = make_driverless_system();
    const uint16_t at_threshold = RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI;
    const uint16_t above = RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI + 1;

    // Past the old boot-anchored window. The solenoid timer has to start on this call.
    current_millis = 20000;
    set_brake_pressure(above, above);
    // Solenoid 1 has to pass on its own evaluation. High pressure on the first call does not finish the check.
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_EQ(fdc_commands.size(), 1u);
    ASSERT_FDC(0, true, 1);

    system.resetTSActiveValues();
    fdc_commands.clear();

    set_brake_pressure(at_threshold, above);
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_FDC(0, true, 1);

    set_brake_pressure(above, at_threshold);
    current_millis += 10;
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_FDC(1, true, 1);

    set_brake_pressure(above, above);
    current_millis += 10;
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_FDC(2, true, 1);

    current_millis += 10;
    ASSERT_TRUE(system.startupCheckTSActive());
    ASSERT_FDC(3, true, 2);

    fdc_commands.clear();
    ASSERT_TRUE(system.startupCheckTSActive());
    ASSERT_TRUE(fdc_commands.empty());
}

TEST(DriverlessSystem, StartupTsActiveWindowsExpireUntilReset)
{
    reset_driverless_inputs();
    DriverlessSystem system = make_driverless_system();
    const uint16_t above = RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI + 1;

    current_millis = 30000;
    set_brake_pressure(0, 0);
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_FDC(0, true, 1);

    current_millis += RDCConstants::SINGLE_SOLENOID_VALIDATION_DELAY_MS;
    fdc_commands.clear();
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_FDC(0, true, 1);

    current_millis += 1;
    fdc_commands.clear();
    set_brake_pressure(above, above);
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_TRUE(fdc_commands.empty());

    system.resetTSActiveValues();
    current_millis = 80000;
    ASSERT_FALSE(system.startupCheckTSActive());
    ASSERT_FDC(0, true, 1);
    ASSERT_TRUE(system.startupCheckTSActive());
    ASSERT_FDC(1, true, 2);

    // Solenoid 2 has its own window measured from the call that passed solenoid 1.
    DriverlessSystem second_solenoid = make_driverless_system();
    fdc_commands.clear();
    current_millis = 90000;
    set_brake_pressure(above, above);
    ASSERT_FALSE(second_solenoid.startupCheckTSActive());
    ASSERT_FDC(0, true, 1);

    set_brake_pressure(0, 0);
    current_millis += RDCConstants::SINGLE_SOLENOID_VALIDATION_DELAY_MS;
    ASSERT_FALSE(second_solenoid.startupCheckTSActive());
    ASSERT_FDC(1, true, 2);

    current_millis += 1;
    fdc_commands.clear();
    set_brake_pressure(above, above);
    ASSERT_FALSE(second_solenoid.startupCheckTSActive());
    ASSERT_TRUE(fdc_commands.empty());
}

TEST(DriverlessEvaluation, StartupSequenceReachesDriving)
{
    reset_driverless_inputs();
    DriverlessSystem system = make_driverless_system();
    DriverlessStateMachine state_machine = DriverlessStateMachine(
        mock_eval_dsms_on,
        mock_eval_vehicle_state,
        etl::delegate<bool()>::create<DriverlessSystem, &DriverlessSystem::startupCheckNoTS>(system),
        etl::delegate<bool()>::create<DriverlessSystem, &DriverlessSystem::startupCheckTSActive>(system),
        mock_eval_ebs_pressure_ok,
        etl::delegate<void()>::create<DriverlessSystem, &DriverlessSystem::resetNoTSValues>(system),
        etl::delegate<void()>::create<DriverlessSystem, &DriverlessSystem::resetTSActiveValues>(system));

    eval_dsms_on = true;
    eval_vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    supervisor_ok = true;
    current_millis = 1000;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_TRUE(fdc_commands.empty());

    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_FDC(0, false, 0);

    supervisor_ok = false;
    current_millis = 1000 + RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::STARTUP_NO_TS);

    current_millis += 1;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::STARTUP_NO_TS);
    ASSERT_FDC(fdc_commands.size() - 1, true, 0);

    eval_vehicle_state = VehicleState_e::TRACTIVE_SYSTEM_ACTIVE;
    current_millis += 50;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::STARTUP_TS_ACTIVE);

    const uint16_t above = RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI + 1;
    set_brake_pressure(above, above);
    fdc_commands.clear();
    current_millis += 100;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::STARTUP_TS_ACTIVE);
    ASSERT_FDC(0, true, 1);

    current_millis += 100;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::READY);
    ASSERT_FDC(1, true, 2);

    eval_vehicle_state = VehicleState_e::READY_TO_DRIVE;
    ASSERT_EQ(state_machine.tickStateMachine(current_millis), DriverlessSystemState_e::DRIVING);
}
