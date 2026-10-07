#ifndef PDB_CONSTANTS
#define PDB_CONSTANTS

/* External Includes */
#include "SharedFirmwareTypes.h"

using pin = uint8_t;
using time_us = uint32_t;


namespace PDBInterfaces
{
    /* General Interface Constants */
    const uint8_t ANALOG_READ_RESOLUTION = 16; // dbl check this
    const uint32_t SERIAL_BAUDRATE = 115200;

}

namespace ACUSystems
{

}
namespace ACUConstants
{

    /* Task Times */
    constexpr uint8_t IDLE_SAMPLE_PRIORITY = 0;
    constexpr time_us IDLE_SAMPLE_PERIOD_US = 1000UL; // 1 000 us = 1000 Hz


    /* CAN Constants */
    const uint32_t VEH_CAN_BAUDRATE = 1000000;
    const uint32_t EM_CAN_BAUDRATE = 500000;
}

namespace PDBConstants
{
    const uint32_t FAULT_HANDLING_PRIORITY = 2;
    const uint32_t FAULT_HANDLING_TELEMETRY_US = 500;
    
    const uint32_t TELEMETRY_PRIORITY = 1;
    const uint32_t TELEMETRY_PERIOD_US = 1000;
}

#endif
