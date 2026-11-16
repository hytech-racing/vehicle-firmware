#ifndef VCF_STARTUP_SENSOR_CHECKS_HPP
#define VCF_STARTUP_SENSOR_CHECKS_HPP

#include <stdint.h>

namespace VCFStartupSensorChecks
{
    // @brief Checks the accelerator channel one reading.
    // @param accelerator_reading Converted sensor reading.
    // @param accelerator_minimum Startup lower limit in the same units.
    // @param accelerator_maximum Startup upper limit in the same units.
    // @return True if the reading is within the limits, including both endpoints, false otherwise.
    bool check_accelerator_channel_one(
        uint32_t accelerator_reading,
        uint32_t accelerator_minimum,
        uint32_t accelerator_maximum);
}

#endif