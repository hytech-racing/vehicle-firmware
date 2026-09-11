#ifndef ODOMETER_INTERFACE
#define ODOMETER_INTERFACE

#include <etl/singleton.h>
#include <Arduino.h>
#include <cmath>
#include "SharedFirmwareTypes.h"
#include <EEPROM.h>


namespace odometer_interface_default_parameters
{
    const float WHEEL_RADIUS_M = 0.406;
    const float WHEEL_CIRCUMFRENCE_M = 2 * PI * WHEEL_RADIUS_M;
}

struct OdometerInterfaceParams_s
{
    const float wheel_radius_m;
    const float wheel_circumfrence_m;
};

class OdometerInterface
{
public:

    OdometerInterface() = delete;

    OdometerInterface(OdometerInterfaceParams_s params = {
                        .wheel_radius_m = odometer_interface_default_parameters::WHEEL_RADIUS_M,
                        .wheel_circumfrence_m = odometer_interface_default_parameters::WHEEL_CIRCUMFRENCE_M
                    }
    ) : _params(params)
    {};

    /**
     * @brief Method takes in the current wheel rpms (x4), averages them, and then calculate the distance we have traveled
     * @note Only updates the distance (km) we have traveled if in RTD, only write to EEPROM every 5 minutes
    */
    void updateOdometer(veh_vec<float> wheel_rpms, VehicleState_e state);

private:

    float _distance_traveled_km;
    unsigned long _last_updated_millis = 0;
    unsigned long _last_eeprom_write_millis = 0;
    OdometerInterfaceParams_s _params;


    /**
     * @brief Method writes the _distance_traveled_km to EEPROM
     * @note EEPROM size is 4284 bytes
    */
    float _writeEEPROM(float distance_traveled_km);

};

using OdometerInterfaceInstance = etl::singleton<OdometerInterface>;


#endif /* */