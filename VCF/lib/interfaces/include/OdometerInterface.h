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

/**
 * @brief EEPROM record format for ONE ring-buffer slot
 * @note This is purely a storage format used internally by _writeEEPROM()/_loadFromEEPROM()
*/
struct OdometerRecord_s
{
    float distance_traveled_km;
    uint32_t sequence;
};

struct OdometerInterfaceParams_s
{
    const float wheel_radius_m;
    const float wheel_circumfrence_m;
};

class OdometerInterface
{
public:

    OdometerInterface(OdometerInterfaceParams_s params = {
                        .wheel_radius_m = odometer_interface_default_parameters::WHEEL_RADIUS_M,
                        .wheel_circumfrence_m = odometer_interface_default_parameters::WHEEL_CIRCUMFRENCE_M
                    }
    ) : _params(params)
    {
        _distance_traveled_km = _loadFromEEPROM();
    };

    /**
     * @brief Method takes in the current wheel rpms (x4), averages them, and then calculate the distance we have traveled
     * @note Only updates the distance (km) we have traveled if in RTD, only write to EEPROM every 5 minutes
     *       (plus an immediate flush the moment RTD ends, to minimize data loss)
    */
    void updateOdometer(veh_vec<float> wheel_rpms, VehicleState_e state);

private:

    /**
     * @note Instead of always writing the same EEPROM address which degrades the cell, we cycle
     *       through NUM_EEPROM_SLOTS addresses round-robin
    */
    static constexpr int NUM_EEPROM_SLOTS = 16;
    static constexpr int SLOT_SIZE = sizeof(OdometerRecord_s);

    float _distance_traveled_km;
    unsigned long _last_updated_millis = 0;
    unsigned long _last_eeprom_write_millis = 0;
    bool _was_ready_to_drive = false;

    /**
     * @brief Used for Ring buffer bookkeeping
     * @param _current_slot is which of the NUM_EEPROM_SLOTS addresses we last wrote to, so the next write knows where to advance to
     * @param _write_sequence is  counter that only ever increases, once per write. It is written alongside the distance in every record,
     *                        so that on boot we can scan all slots  and identify the most recently written one as whichever slot
     *                        has the highest sequence number
    */
    int _current_slot = 0;
    uint32_t _write_sequence = 0;

    OdometerInterfaceParams_s _params;

    /**
     * @brief Method writes distance_traveled_km to the next slot in the EEPROM ring buffer
     * @note EEPROM size is 4284 bytes; this ring buffer uses NUM_EEPROM_SLOTS * SLOT_SIZE bytes,
     *       cycling addresses to spread write wear instead of a single address/cell
    */
    void _writeEEPROM(float distance_traveled_km);

    /**
     * @brief Scans all ring buffer slots on boot and returns the distance from
     *        whichever slot has the highest sequence number (i.e. most recently written)
    */
    float _loadFromEEPROM();

};

using OdometerInterfaceInstance = etl::singleton<OdometerInterface>;


#endif /* */