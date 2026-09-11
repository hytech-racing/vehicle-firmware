#include "OdometerInterface.h"

void OdometerInterface::updateOdometer(veh_vec<float> wheel_rpms, VehicleState_e state)
{
    if (state == VehicleState_e::READY_TO_DRIVE)
    {
        auto wheel_rpms_array = wheel_rpms.as_array();
        float avg_wheel_rpm;

        for(const auto & wheel_rpm : wheel_rpms_array)
        {
            avg_wheel_rpm += wheel_rpm;
        }

        avg_wheel_rpm = avg_wheel_rpm /4;


        unsigned long curr_millis = millis();
        float dt_minutes = (curr_millis - _last_updated_millis) / 60000.0;
        _last_updated_millis = curr_millis;

        float distance_m = avg_wheel_rpm * _params.wheel_circumfrence_m * dt_minutes;
        _distance_traveled_km += distance_m / 1000.0;

        if (curr_millis - _last_eeprom_write_millis >= 300000UL)
        {
            _writeEEPROM(_distance_traveled_km);
            _last_eeprom_write_millis = curr_millis;
        }
    }
    else
    {
        _distance_traveled_km += 0;
    }
}

float OdometerInterface::_writeEEPROM(float distance_traveled_km)
{

    if (_distance_traveled_km == 0)
    {
        EEPROM.write(0, distance_traveled_km);
    }

    EEPROM.update(0, distance_traveled_km);
}

