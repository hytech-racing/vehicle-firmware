#ifndef DASH_GLOBALS_H
#define DASH_GLOBALS_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Interface and System Data Structs */
using VCFData_sInstance = etl::singleton<VCFData_s>;
using VCRData_sInstance = etl::singleton<VCRData_s>;

#endif