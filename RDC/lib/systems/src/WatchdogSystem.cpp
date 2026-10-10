#include "WatchdogSystem.hpp"
#include "SharedFirmwareTypes.h"

/* Returns intended watchdog state */
bool WatchdogSystem::getRDCWatchdogState(unsigned long curr_millis)
{

    if ((curr_millis - _watchdog_time) > _watchdog_kick_interval)
    {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
    }

    return _watchdog_state;
}
