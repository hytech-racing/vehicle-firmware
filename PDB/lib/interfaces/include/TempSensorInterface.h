#ifndef _TEMPSENSORINTERFACE_H_
#define _TEMPSENSORINTERFACE_H_

#include <stm32h7xx_hal.h>
#include "HT_I2C.h"
#include "hytech.h"

//uint16_t pins[] = [3,4,5,6,7,8,9,10] //all GPIO pins on port D

struct TempSensorRegisters_s {
    static constexpr uint8_t TEMP_VALUE = 0x00;
    static constexpr uint8_t CONFIG = 0x01;
    static constexpr uint8_t T_HYST_SETPOINT = 0x02;
    static constexpr uint8_t T_OVER_SETPOINT = 0x03;
    static constexpr uint8_t ONE_SHOT = 0x04;
};

struct TempSensorData_s {
    float temp_value = 0.0f;
    //uint16_t config_bits = 0; // use comparator mode, 
    float t_hyst_sp = 75.0f; // default 75 deg. C
    float t_os_sp = 80.0f; // default 80 deg. C
    //uint16_t one_shot = 0;
};

class TempSensorInterface {
    public:
    GPIO_TypeDef* alert_port;
    uint16_t alert_pin;
    volatile bool _alert_pending = false; //ensures caching optimization does not prevent interrupt logic from running
    //public to allow ease of use if the interrupt function is put in a different file

    TempSensorInterface(uint16_t addr, float t_hyst_sp, float t_os_sp, GPIO_TypeDef* port, uint16_t pin) {
         _addr = addr << 1; 
         overtemp_reached = false;
         _sensor_data.t_hyst_sp = t_hyst_sp;
         _sensor_data.t_os_sp = t_os_sp;
         alert_port = port;
         alert_pin = pin;
    };

    void onAlertIrq();

    void encodeSetPoint(float temp, uint8_t out[2]);
    bool initSensor();
    bool readTempValue();
    bool getOvertemp() const { return overtemp_reached; };
    //const ensures no one changes the object at hand
    float getTemp() const { return _sensor_data.temp_value; };
    bool isOvertemp() const;
    void handleAlert();

    private:
    uint16_t _addr;
    TempSensorData_s _sensor_data;
    bool overtemp_reached;
};

extern TempSensorInterface temps[8]; //Top to bottom

#endif // _TEMPSENSORINTERFACE_H_