#ifndef _TEMPSENSORINTERFACE_H_
#define _TEMPSENSORINTERFACE_H_

#include <stm32h7xx_hal.h>
#include <stm32h750xx.h>
#include "hytech.h"

struct TempSensorRegisters_s {
    static constexpr uint8_t TEMP_VALUE = 0x00;
    static constexpr uint8_t CONFIG = 0x01;
    static constexpr uint8_t T_HYST_SETPOINT = 0x02;
    static constexpr uint8_t T_OVER_SETPOINT = 0x03;
    static constexpr uint8_t ONE_SHOT = 0x04;
}

struct TempSensorData_s {
    float temp_value;
    uint16_t config_bits; // use comparator mode, 
    uint16_t t_hyst_sp; // default 75 deg. C
    uint16_t t_os_sp; // default 80 deg. C
    uint16_t one_shot;
}

class TempSensorInterface {
    public:
    TempSensorInterface(uint16_t addr, uint16_t t_hyst_sp, uint16_t t_os_sp) {
         _addr = addr << 1; 
         overtemp_reached = false;
         _sensor_data.t_hyst_sp = t_hyst_sp;
         _sensor_data.t_os_sp = t_os_sp;

    };
    void initSensor();
    void readTempValue();

    private:
    uint16_t _addr;
    TempSensorData_s _sensor_data;
    bool overtemp_reached;

   
}

#endif // _TEMPSENSORINTERFACE_H_