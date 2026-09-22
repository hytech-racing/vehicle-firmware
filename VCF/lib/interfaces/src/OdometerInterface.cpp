#include "OdometerInterface.h"

void OdometerInterface::updateOdometer(veh_vec<speed_rpm> wheel_rpms, VehicleState_e state)
{
    unsigned long curr_millis = millis();

    if (state == VehicleState_e::READY_TO_DRIVE)
    {
        if (!_was_ready_to_drive)
        {
            // Just entered RTD —> reset the timer baseline so the very first calc below doesn't include however long we were sitting idle
            _last_updated_millis = curr_millis;
            _was_ready_to_drive = true;
        }

        auto wheel_rpms_array = wheel_rpms.as_array();
        float avg_wheel_rpm = 0.0f;

        for (const auto &wheel_rpm : wheel_rpms_array)
        {
            avg_wheel_rpm += wheel_rpm;
        }
        avg_wheel_rpm /= wheel_rpms_array.size();

        float dt_minutes = (curr_millis - _last_updated_millis) / 60000.0f;
        _last_updated_millis = curr_millis;

        float distance_m = avg_wheel_rpm * _params.wheel_circumfrence_m * dt_minutes;
        _distance_traveled_km += distance_m / 1000.0f;

        if (curr_millis - _last_eeprom_write_millis >= 300000UL) // 5 minutes; could be too conservative
        {
            _writeEEPROM(_distance_traveled_km);
            _last_eeprom_write_millis = curr_millis;
        }
    }
    else
    {
        if (_was_ready_to_drive)
        {
            /**
             * @note Just left RTD —> flush immediately so we don't lose up to 5 minutes of unrecorded driving if power
             *       is cut before the next periodic write would have fired
             * @warning If we lose power during RTD then there is nothig we can do; unlikely to happen
             */
            _writeEEPROM(_distance_traveled_km);
            _last_eeprom_write_millis = curr_millis;
            _was_ready_to_drive = false;
        }
    }
}

void OdometerInterface::_writeEEPROM(float distance_traveled_km)
{
    _write_sequence++;
    _current_slot = (_current_slot + 1) % NUM_EEPROM_SLOTS;

    OdometerRecord_s record;
    record.distance_traveled_km = distance_traveled_km;
    record.sequence = _write_sequence;

    int addr = _current_slot * SLOT_SIZE;
    EEPROM.put(addr, record);
}

float OdometerInterface::_loadFromEEPROM()
{
    /**
     * @note On boot we don't know which slot was written to most recently: Read every slot, and trust whichever one reports
     *       the highest sequence number, since sequence only ever increases and gets stamped on every write.
    */
    OdometerRecord_s best_record{0.0f, 0};
    bool is_latest_found = false;

    for (int slot = 0; slot < NUM_EEPROM_SLOTS; slot++)
    {
        OdometerRecord_s record;
        EEPROM.get(slot * SLOT_SIZE, record);

        // Sanity check to reject slots that have never been written
        if (!isnan(record.distance_traveled_km) &&
            record.distance_traveled_km >= 0.0f &&
            record.distance_traveled_km < 1000000.0f)
        {
            if (!is_latest_found || record.sequence > best_record.sequence)
            {
                best_record = record;
                is_latest_found = true;
                _current_slot = slot;
            }
        }
    }

    _write_sequence = best_record.sequence;
    return best_record.distance_traveled_km;
}