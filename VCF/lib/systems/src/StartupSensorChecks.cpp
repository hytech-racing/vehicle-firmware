#include "StartupSensorChecks.hpp"
#include "SensorRangeCheck.hpp"

namespace VCFStartupSensorChecks
{
    bool check_accelerator_channel_one(
        uint32_t accelerator_reading,
        uint32_t accelerator_minimum,
        uint32_t accelerator_maximum)
    {
        return sensor_range_check(
            accelerator_reading,
            accelerator_minimum,
            accelerator_maximum);
    }
}