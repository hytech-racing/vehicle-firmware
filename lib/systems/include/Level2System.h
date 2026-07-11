#ifndef LEVEL2SYSTEM_H
#define LEVEL2SYSTEM_H
/**
 * NOTE: This is the system file for 240V charging. It is called level 2 because that is what SAE defines it as.
 *       However you will see 240V naming convention used interchangably
 */

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>
#include <cstddef>
#include <cstdint>

/* Local Interface Includes */
#include "ADCInterface.h"
#include "Level2Interface.h"
#include "WatchdogInterface.h"


struct Level2SystemThresholds_s
{
    const float state_B2_control_voltage_min      = 6.36F;
    const float state_B2_control_voltage_max      = 9.59F;
    const float state_B2_proximity_voltage_min    = 1.23F;
    const float state_B2_proximity_voltage_max    = 1.82F;
    const float state_C2_control_voltage_min      = 4.00F;
    const float state_C2_control_voltage_max      = 6.53F;
    const float state_C2_proximity_voltage_min    = 1.23F;
    const float state_C2_proximity_voltage_max    = 1.82F;
};

class Level2System
{
public:

    Level2System(Level2Interface& level2_interface,
                ADCInterface& adc_interface,
                WatchdogInterface& watchdog_interface,
                Level2SystemThresholds_s thresholds = {}
    ) :
        _level2_interface(level2_interface),
        _adc_interface(adc_interface),
        _watchdog_interface(watchdog_interface),
        _thresholds(thresholds)
    {};

    /**
     * Function check for the expected startup/120V Charging Conditions (4):
     * PP = 5  +  CP = 0  +  240_En = HIGH  +  240_OK = LOW
     * @return if the 4 conditions above are met
     */
    bool check_120_conditions(ADCInterface& adc_interface);

    /**
     * Function check for the expected 240V Charging Conditions (2):
     * 240_OK = HIGH  +  JP_OUT_READ = HIGH
     * @return if the 2 conditions above are met
     */
    bool check_240_conditions(ADCInterface& adc_interface);

    /**
     * Function checks if the switch is engaged for 120V charging
     * 240_OK = LOW  +  JP_OUT_READ = HIGH
     * @return if the 2 conditions above are met
     */
    bool is_120_switched(ADCInterface& adc_interface);

    /**
     * Function checks if the switch is engaged for 120V charging
     * 240_OK = LOW  +  jumper_read = LOW
     * @return if the 2 conditions above are met
     */
    bool is_240_switched(ADCInterface& adc_interface);

    /**
     * @return true if all state B2 conditions present, else false
     */
    bool check_state_B2_conditions(ADCInterface& adc_interface, Level2Interface& level2_interface);

    /**
     * @return true if all state C2 conditions present, else false
     */
    bool check_state_C2_conditions(ADCInterface& adc_interface, Level2Interface& level2_interface);

private:

    Level2Interface& _level2_interface;
    ADCInterface& _adc_interface;
    WatchdogInterface& _watchdog_interface;
    Level2SystemThresholds_s _thresholds;

};

using Level2SystemInstance = etl::singleton<Level2System>;

#endif