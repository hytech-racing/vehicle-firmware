#ifndef RDC_CONSTANTS
#define RDC_CONSTANTS

/* External Includes */
#include "SharedFirmwareTypes.h"

using time_us = uint32_t;
namespace RDCConstants
{
    /* ---------- Task Priorities & Periods ---------- */
    const int SERIAL_BAUDRATE = 115200;

    constexpr uint8_t SEND_CAN_PRIORITY = 5;
    constexpr time_us SEND_CAN_PERIOD_US = 10000; // 10000 us = 100 HZ

    constexpr uint8_t ENQUEUE_FDC_CONTROL_PRIORITY = 10;
    constexpr time_us ENQUEUE_FDC_CONTROL_PERIOD_US = 50000; // 50000 us = 20 HZ

    constexpr uint8_t ENQUEUE_RSS_OPERATING_MODE_PRIORITY = 10;
    constexpr time_us ENQUEUE_RSS_OPERATING_MODE_PERIOD_US = 50000; // 50000 us = 20 HZ

    constexpr uint8_t KICK_WATCHDOG_PRIORITY = 0;
    constexpr time_us KICK_WATCHDOG_PERIOD_US = 10000; // 10000 us = 100 HZ

    constexpr uint8_t TICK_STATE_MACHINE_PRIORITY = 1;
    constexpr time_us TICK_STATE_MACHINE_PERIOD_US = 5000; // 5000 us = 200 HZ

    constexpr uint8_t COLLECT_ASYNC_DATA_PRIORITY = 4;
    constexpr time_us COLLECT_ASYNC_DATA_PERIOD_US = 5000; // 5000 us = 200 HZ

    constexpr uint8_t ENQUEUE_STRAIN_GAUGE_DATA_PRIORITY = 11;
    constexpr time_us ENQUEUE_STRAIN_GAUGE_DATA_PERIOD_US = 10000; // 10 000 us = 100 Hz

    constexpr uint8_t DEBUG_PRINT_PRIORITY = 100;
    constexpr time_us DEBUG_PRINT_PERIOD_US = 100000; // 100000 us = 10 HZ
    /* --------- CAN Constants ---------- */
    constexpr uint32_t DRIVERLESS_CAN_BAUDRATE = 1000000;

    /* ---------- EBS Constants ---------- */
    constexpr uint16_t EBS_HEARTBEAT_TIMEOUT_MS = 1000;        // 1000ms = 1 second
    constexpr uint16_t STARTUP_WATCHDOG_CHECK_DELAY_MS = 3000; // 3000ms = 3 seconds

    constexpr uint8_t EBS_AIR_PRESSURE_THRESHOLD_PSI = 60;
    constexpr uint8_t BRAKE_FLUID_PRESSURE_THRESHOLD_PSI = 100;

    constexpr uint16_t SINGLE_SOLENOID_VALIDATION_DELAY_MS = 3000; // 3000ms = 3 seconds

    /* --------- RSS Constants ---------- */
    constexpr uint8_t RSS_NODE_ID = 0x11;

    /* --------- ADC Constants ---------- */

    // (3.3 / 4095) * (1000 / 0.0127) = 63.4536067
    // based on 12.7mV change in value per 1kN of compression -- for lower A arm
    // (3.3 / 4095) * (1000 / 0.016) = 50.3663003
    // based on 16mV change in value per 1kN of compression -- for upper A arm and toe rod
    constexpr float STRAIN_GAUGE_1_SCALE = 63.4536067;
    constexpr float STRAIN_GAUGE_2_SCALE = 63.4536067;
    constexpr float STRAIN_GAUGE_3_SCALE = 50.3663003;
    constexpr float STRAIN_GAUGE_4_SCALE = 50.3663003;
    constexpr float STRAIN_GAUGE_5_SCALE = 50.3663003;

    // 0 - (2.62 * 1000 / 0.0127) = -206299.213
    // based on 2.62V = zero load, and calculated newtons / V slope -- for lower A arm
    // 0 - (2.62 * 1000 / 0.016) = -163750.0
    // based on 2.62V = zero load, and calculated newtons / V slope -- for upper A arm and toe
    constexpr float STRAIN_GAUGE_1_OFFSET = -206299.213;
    constexpr float STRAIN_GAUGE_2_OFFSET = -206299.213;
    constexpr float STRAIN_GAUGE_3_OFFSET = -163750.0;
    constexpr float STRAIN_GAUGE_4_OFFSET = -163750.0;
    constexpr float STRAIN_GAUGE_5_OFFSET = -163750.0;

    /* ---------- Watchdog Constants ---------- */
    constexpr uint16_t WATCHDOG_TIMEOUT_MS = 10; // 10ms = 10 HZ
} // namespace RDCConstants

namespace PinAssignments
{
    constexpr uint8_t WATCHDOG_PIN = 20;
    constexpr uint8_t DS_SENSORS_OK_PIN = 12;
} // namespace PinAssignments
#endif
