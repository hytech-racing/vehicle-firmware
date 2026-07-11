#ifndef WATCHDOG_INTERFACE_H
#define WATCHDOG_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>

/* Local Interface Includes */
#include "SystemTimeInterface.h"

using pin = uint8_t;

namespace watchdog_default_parameters
{
    constexpr unsigned long WATCHDOG_KICK_INTERVAL_MS = 10UL; // 10 ms = 100 Hz
}

struct WatchdogPinout_s
{
    pin teensy_watchdog_pin;
    pin teensy_software_ok_pin;
};

struct WatchdogInterfaceParams_s
{
    WatchdogPinout_s pinout;
    unsigned long watchdog_kick_interval_ms;
};

/**
 * This class controls the boolean _watchdog_state, but does not directly control the watchdog.
 * WatchdogSystem provides functionality to initialize, monitor, and "kick" the watchdog to prevent system resets.
 *
 * NOTE:  To ensure system responsiveness, WatchdogSystem requires periodic updates by calling the `get_watchdog_state()` method.
 * This toggles the _watchdog_state (if the interval has passed) and returns the new state, which must then be sent to the watchdog.
 */
class WatchdogInterface
{
public:

    WatchdogInterface(WatchdogPinout_s pinout,
                    uint32_t watchdog_kick_interval_ms = watchdog_default_parameters::WATCHDOG_KICK_INTERVAL_MS
    ): _watchdog_parameters {
            pinout,
            watchdog_kick_interval_ms
        }
    {};

    void init();

    /**
     * Get/update watchdog state
     * @param curr_millis time of VCF time
     * @post IF reach interval, _watchdog_time updated and state switched
    */
    bool update_watchdog_state(unsigned long curr_millis);

private:

    const WatchdogInterfaceParams_s _watchdog_parameters = {};

    /**
     * @brief timestamp of the last watchdog kick
     */
    uint32_t _watchdog_time = 0;

    /**
     * @brief current output level driven on the watchdog kick pin, true = HIGH
     */
    bool _watchdog_state = false;

};

using WatchdogInterfaceInstance = etl::singleton<WatchdogInterface>;

#endif /* WATCHDOG_SYSTEM_H */
