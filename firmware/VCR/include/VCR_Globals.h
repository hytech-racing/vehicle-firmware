#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <array>
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "InverterInterface.h"

/* Interface and System Data Structs */
extern VCRData_s vcr_data; // NOLINT

extern unsigned long pulseCount; // NOLINT

#endif /* VCR_GLOBALS */
