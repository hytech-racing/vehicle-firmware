#ifndef WATCHDOG_SYSTEM_H
#define WATCHDOG_SYSTEM_H

#include <etl/singleton.h>

/**
 * This class controls the boolean _watchdog_state, but does not directly control the watchdog.
 * WatchdogSystem provides functionality to initialize, monitor, and "kick" the watchdog to prevent system resets.
 *
 * NOTE:  To ensure system responsiveness, WatchdogSystem requires periodic updates by calling the `get_watchdog_state()` method.
 * This toggles the _watchdog_state (if the interval has passed) and returns the new state, which must then be sent to the watchdog.
 */

class WatchdogSystem
{
    public:
    WatchdogSystem(const unsigned long kick_interval_ms = 10UL)
        : _watchdog_time(0),
          _watchdog_state(false),
          _watchdog_kick_interval(kick_interval_ms) {};

    private:
    /*Private constructor*/

    /* Watchdog last kicked time */
    unsigned long _watchdog_time;
    bool _watchdog_state;
    unsigned long _watchdog_kick_interval;
    /* Watchdog output state */

    public:
    /* Get/update watchdog state */
    bool getRDCWatchdogState(unsigned long curr_millis);
};

using WatchdogInstance = etl::singleton<WatchdogSystem>;

#endif /* WATCHDOG_SYSTEM_H */
