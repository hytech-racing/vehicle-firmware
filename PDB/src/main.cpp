#include "PDB_Config.hpp"
#include "PDB_InterfaceTasks.hpp"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* Scheduler Setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
HT_TASK::Task updateBuckConverters(HT_TASK::DUMMY_FUNCTION, &updateBuckConvertersTask, PDBConstants::BUCK_UPDATE_PRIORITY, PDBConstants::BUCK_UPDATE_PERIOD_US);
HT_TASK::Task readTempSensors(HT_TASK::DUMMY_FUNCTION, &readTempSensorsTask, PDBConstants::TEMP_SENSOR_READ_PRIORITY, PDBConstants::TEMP_SENSOR_READ_PERIOD_US);
HT_TASK::Task updateHotSwap(HT_TASK::DUMMY_FUNCTION, &updateHotSwapTask, PDBConstants::HOTSWAP_UPDATE_PRIORITY, PDBConstants::HOTSWAP_UPDATE_PERIOD_US);
HT_TASK::Task sampleLoadSwitches(HT_TASK::DUMMY_FUNCTION, &sampleLoadSwitchesTask, PDBConstants::LOAD_SWITCH_SAMPLE_PRIORITY, PDBConstants::LOAD_SWITCH_SAMPLE_PERIOD_US);
HT_TASK::Task debugPrints(HT_TASK::DUMMY_FUNCTION, &debugPrintsTask, PDBConstants::DEBUG_PRINT_PRIORITY, PDBConstants::DEBUG_PRINT_PERIOD_US);


void setup()
{
    MPU_Config();

    analogReadResolution(PDBInterfaces::ANALOG_READ_RESOLUTION);
    Serial.begin(PDBInterfaces::SERIAL_BAUDRATE);

    initializeAllPins();
    initializeAllInterfaces();

    // Setup scheduler
    scheduler.setTimingFunction(micros);

    // Schedule Tasks
    scheduler.schedule(updateBuckConverters);
    scheduler.schedule(readTempSensors);
    scheduler.schedule(updateHotSwap);
    scheduler.schedule(sampleLoadSwitches);
    scheduler.schedule(debugPrints);
}

void loop()
{
    scheduler.run();

    // Initial Bring-up: enable every MCU-controlled rail
    // Turns on the Lidar and DTI bucks and all 6 load switches once.
    // Set PDBConstants::DEBUG_ENABLE_ALL_RAILS to false for normal operation.
    static bool are_rails_enabled = false;
    if (PDBConstants::DEBUG_ENABLE_ALL_RAILS && !are_rails_enabled)
    {
        enableAllRails();
        are_rails_enabled = true;
    }

}