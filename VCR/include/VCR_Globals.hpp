#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

#include <array>

/* Local Interface Includes */
#include "InverterInterface.hpp"

/* Interface and System Data Structs */
extern VCRData_s vcr_data; // NOLINT

extern unsigned long pulseCount; // NOLINT

#endif /* VCR_GLOBALS */
