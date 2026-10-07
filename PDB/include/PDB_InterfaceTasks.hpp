#ifndef PDB_INTERFACETASKS
#define PDB_INTERFACETASKS

#include "PDB_Constants.hpp"

/* External Includes */
#include <ht_task.hpp>
#include "CANInterface.h"
#include "fdcan.h"

/* Local Interface Includes */

extern uint16_t initialize_status;
extern uint16_t faults_alerted;

struct PDB_readings {
    uint8_t bucks_nrsts; //bucks
    float[8] imon_currents;
}

extern PDB_readings pdb_readings;

/* Local System Includes */


void initializeAllInterfaces();

void SystemClock_config();
void MPU_Config();
void Error_Handler();

#endif

