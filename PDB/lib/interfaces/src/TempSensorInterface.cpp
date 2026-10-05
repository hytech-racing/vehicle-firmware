#include "TempSensorInterface.h"
#include "HT_I2C.h"
#include "LDSWInterface.h"
// Not One shot and normal mode of operation

// 16 bits Data Register that are 12 bit left justified, one LSB is 0.0625C
// Consider ignoring first read (will be 0s initialized)
// Convert given temperature into properly formatted code

static const uint32_t TIMEOUT = 10;

TempSensorInterface temps[8] = {
    TempSensorInterface(0x48, 75.0f, 80.0f, GPIOD, GPIO_PIN_3),
    TempSensorInterface(0x49, 75.0f, 80.0f, GPIOD, GPIO_PIN_4),
    TempSensorInterface(0x4A, 75.0f, 80.0f, GPIOD, GPIO_PIN_5),
    TempSensorInterface(0x4B, 75.0f, 80.0f, GPIOD, GPIO_PIN_6),
    TempSensorInterface(0x4C, 75.0f, 80.0f, GPIOD, GPIO_PIN_7),
    TempSensorInterface(0x4D, 75.0f, 80.0f, GPIOD, GPIO_PIN_8),
    TempSensorInterface(0x4E, 75.0f, 80.0f, GPIOD, GPIO_PIN_9),
    TempSensorInterface(0x4F, 75.0f, 80.0f, GPIOD, GPIO_PIN_10)
};

void TempSensorInterface::encodeSetPoint(float temp, uint8_t out[2]) {
    uint16_t raw = (uint16_t) ((int16_t)(temp * 16.0f)) << 4; //old compilers tweak out with shifting on signed ints so precaution
    out[0] = (uint8_t) (raw >> 8); // (First 8 bytes, cast last always)
    out[1] = (uint8_t) (raw & 0xFF); // (Last 8 bytes)
}

bool TempSensorInterface::initSensor() {
    uint8_t config_bits = 0x00; // Redundancy, each bit of the byte configures something on the register refer to page 15
    
    uint8_t hyst_bytes[2];
    encodeSetPoint(_sensor_data.t_hyst_sp, hyst_bytes);
    uint8_t os_bytes[2];
    encodeSetPoint(_sensor_data.t_os_sp, os_bytes);

    // HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    // Initialize configure bits in configure register
    HAL_StatusTypeDef config_status = HAL_I2C_Mem_Write(&hi2c1, _addr, TempSensorRegisters_s::CONFIG, I2C_MEMADD_SIZE_8BIT, 
        &config_bits, sizeof(config_bits), TIMEOUT); 
    if (config_status != HAL_OK) return 0;

    // Hysteris point
    HAL_StatusTypeDef hyst_sp_status = HAL_I2C_Mem_Write(&hi2c1, _addr, TempSensorRegisters_s::T_HYST_SETPOINT, I2C_MEMADD_SIZE_8BIT,
        hyst_bytes, 2, TIMEOUT);
    if (hyst_sp_status != HAL_OK) return 0;
    
    // OverTemperature shutdown point
    HAL_StatusTypeDef t_os_sp_status = HAL_I2C_Mem_Write(&hi2c1, _addr, TempSensorRegisters_s::T_OVER_SETPOINT, I2C_MEMADD_SIZE_8BIT,
        os_bytes, 2, TIMEOUT);
    if (t_os_sp_status != HAL_OK) return 0;
    
    _alert_pending = isOvertemp(); //checks when initialized if the temperature is already hot
    return 1;
}

bool TempSensorInterface::readTempValue() {
    uint8_t rx_data[2];
    // Reads the temperature from the temperature data register (address defined in the interface instantiation)
    HAL_StatusTypeDef temp_status = HAL_I2C_Mem_Read(&hi2c1, _addr, TempSensorRegisters_s::TEMP_VALUE, I2C_MEMADD_SIZE_8BIT, 
        rx_data, 2, TIMEOUT);
    if (temp_status != HAL_OK) return 0;

    int16_t raw_temp = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    raw_temp = raw_temp >> 4;
    _sensor_data.temp_value = raw_temp * 0.0625f;
    return 1;
}

bool TempSensorInterface::isOvertemp() const {
    return HAL_GPIO_ReadPin(alert_port, alert_pin) == GPIO_PIN_RESET; //checks if the pin is low for temperature fault
}

void TempSensorInterface::onAlertIrq() {
    _alert_pending = true; //only sets the flag
}

//Personal note: the temp would not swwing wildly to trigger multiple interrupts
bool TempSensorInterface::handleAlert() {
    if(!_alert_pending) return 0;
    _alert_pending = false;
    bool now = isOvertemp();
    if (now != overtemp_reached) { //react to a new transition (for not sending multiple can lines and other things)
        overtemp_reached = now;
        if (overtemp_reached) {
            //Handle the issue, perhaps shutdown the loadswitch (check LDSWInterface)
            uint8_t tempIndex = alert_pin - 3 //indexes the temperature sensors according to addr / pin
            if (tempIndex == 0) {
                //disable LIDAR
                LDSWs[4].disable();
            }
            elif (tempindex == 1) {
                //disable Orin
                LDSWs[1].disable();
            }
            elif (tempindex == 3) {
                // disable DTI Inverter
                LDSWs[5].disable();
            }
            else {
                // disable hotswap since buck is heating up on the main line
                HS5066.shutOff(); // be very careful with this - DOUBLE CHECK WITH VANSH
            }
        }
        else {
            //overtemp solved (below hystersis), perhaps do nothing
        }
    }
    return 1;
    //in theory if simply shutting off the loads is an idempotent operation, but can messages are not
}