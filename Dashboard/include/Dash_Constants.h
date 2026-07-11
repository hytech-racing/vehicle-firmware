#ifndef DASH_CONSTANTS_H
#define DASH_CONSTANTS_H

/**
 * While for most of our boards we have interface, systems, and general constants, since Dash is so simplistic we can just
 * define some constants without encapsulating with a struct
 */

/* External Includes */
#include "SharedFirmwareTypes.h"
#include <variant_generic.h>

using time_us = uint32_t;

constexpr int NEOPIXEL_CONTROL_PIN = PC14;
constexpr int NEOPIXEL_COUNT = 16; // 12 neopixeles on dashboard

namespace DashConstants
{
    constexpr uint8_t NEOPIXEL_UPDATE_PRIORITY = 90;
    constexpr time_us NEOPIXEL_UPDATE_PERIOD_US = 100000; // 100 000 us = 10 Hz

    constexpr uint8_t SCREEN_REFRESH_PRIORITY = 100;
    constexpr time_us SCREEN_REFRESH_PERIOD_US = 33333; // 33 333 us = 30 Hz
};

#endif /* DASH_CONSTANTS_H */