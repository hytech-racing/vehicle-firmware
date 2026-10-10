#ifndef PDB_INTERFACETASKS
#define PDB_INTERFACETASKS

#include "PDB_Constants.hpp"

/* External Includes */
#include <ht_task.hpp>
#include "STM32_CANInterface.hpp"
#include "STM32_I2CInterface.hpp"

/* Local Interface Includes */
#include "BuckConverterInterface.hpp"
#include "HotSwapInterface.hpp"
#include "LoadSwitchInterface.hpp"
#include "TempSensorInterface.hpp"
#include "VCRInterface.hpp"
#include "PDBCANInterface.hpp"


/**
 * @brief Sets every MCU pin to a known state at boot: all enable outputs LOW (everything off), status lines as inputs.
 * @note Replaces CubeMX's MX_GPIO_Init(). Call first in setup(), before any interface uses its pins.
*/
void initializeAllPins();

void initializeAllInterfaces();

/**
 * @brief Enables every MCU-controlled rail: the bucks with an EN pin (Lidar, DTI) and all load switches.
 * @note Bring-up helper only!
*/
void enableAllRails();

/**
 * @brief Samples and debounces every buck's PG pin and advances its state machine
 */
HT_TASK::TaskResponse updateBuckConvertersTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Reads all eight ADT75 temp sensors over I2C and updates their overtemp flags.
*/
HT_TASK::TaskResponse readTempSensorsTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Handles a hotswap alert if SMBA is asserted (reads the fault, refreshes telemetry, clears faults),
 *        otherwise reads telemetry (VIN, VOUT, IIN, PIN, temperature, PGD).
*/
HT_TASK::TaskResponse updateHotSwapTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Samples every load switch's FLT pin and IMON current.
*/
HT_TASK::TaskResponse sampleLoadSwitchesTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Debug prints.
*/
HT_TASK::TaskResponse debugPrintsTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

#endif