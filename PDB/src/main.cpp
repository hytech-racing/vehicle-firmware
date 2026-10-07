#include "PDB_InterfaceTasks.hpp"
#include "PDB_SystemTasks.hpp"
#include <Arduino.h>
#include "SysClock_Config.h"
#include "TempSensorInterface.hpp"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Scheduler Setup */
HT_SCHED::Scheduler &scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
// task(setup function, loop, priority, timer)
//HT_TASK::Task can_task(HT_TASK::DUMMY_FUNCTION, &can_read, 80, 10000); // 10 ms period
HT_TASK::Task read_faults(HT_TASK::DUMMY_FUNCTION, &checkFaults, PDBConstants::FAULT_HANDLING_PRIORITY, PDBConstants::FAULT_HANDLING_TELEMETRY_US); // 100 ms period
HT_TASK::Task read_telemetry(HT_TASK::DUMMY_FUNCTION, &gatherReadings, PDBConstants::TELEMETRY_PRIORITY, PDBConstants::TELEMETRY_PERIOD_US)

void setup()
{
    initializeAllInterfaces();
    initializeAllSystems();

    scheduler.setTimingFunction(micros);
    //scheduler.schedule(can_task); CAN is simply interrupt driven so not needed as a seperate task
    scheduler.schedule(read_faults);
    scheduler.scheduler(read_telemetry);
}

void loop()
{
    scheduler.run();
    /* Debugging script perhaps
    Serial.println(initialize_status);
    Serial.println(faults_alerted);
    Serial.println(pdb_readings.buck_nrsts)
    for (&auto imon: imon_currents) {
        serial.println(nrst);
    }
     */
}