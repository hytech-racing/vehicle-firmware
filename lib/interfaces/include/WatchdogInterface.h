#ifndef WATCHDOG_INTERFACE_H
#define WATCHDOG_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>

using pin = uint8_t;


namespace watchdog_default_parameters
{
    constexpr unsigned long WATCHDOG_KICK_INTERVAL_MS = 12UL;
}

struct WatchdogPinout_s
{
    pin teensy_watchdog_pin; //teensy input to watchdog
    pin teensy_sw_shdn_pin ;
};

struct WatchdogInterfaceParams_s
{
    WatchdogPinout_s pinout;
    unsigned long watchdog_kick_interval_ms;
};

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
     * @param curr_millis time of CCU time
     * @post IF reach interval, _watchdog_time updated and state switched
    */
    bool update_watchdog_state(uint32_t curr_millis);

    /**
     * Set SW_SHDN pin low
     */
    void set_sw_shdn_pin_low();

    /**
     * Set SW_SHDN pin high
     */
    void set_sw_shdn_pin_high();

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