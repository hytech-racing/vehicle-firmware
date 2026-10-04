#include "TempSensorInterface.h"
#include "HT_I2C.h"
// D5 one-shot mode? Y/N, currently no

void TempSensorInterface::encodeSetPoint(uint16_t data, uint8_t out[2]) {
    int16_t raw = (int16_t) (data << 16);
    out[0] = (uint8_t) raw >> 8; // (First 8 bytes)
    out[1] = (uint8_t) raw & 0xFF; // (Last 8 bytes)
}

void TempSensorInterface::initSensor() {
    uint8_t config_bits = 0x40; // interrupt mode, no one-shot mode, no SMBA
    
    uint8_t hyst_bytes[2];
    encodeSetPoint(_sensor_data.t_hyst_sp, hyst_bytes);
    uint8_t os_bytes[2];
    encodeSetPoint(_sensor_data.t_os_sp, os_bytes);

    HAL_StatusTypeDef config_status = HAL_I2C_Mem_Write(&hi2c1, 
                                                        _addr,
                                                        TempSensorRegisters_s::CONFIG,
                                                        I2C_MEMADD_SIZE_8BIT,
                                                        &config_bits,
                                                        sizeof(config_bits),
                                                        HAL_MAX_DELAY); 
    HAL_StatusTypeDef hyst_sp_status = HAL_I2C_Mem_Write(&hi2c1,
                                                        _addr,
                                                        TempSensorRegisters_s::T_HYST_SETPOINT,
                                                        I2C_MEMADD_SIZE_8BIT,
                                                        hyst_bytes,
                                                        2,
                                                        HAL_MAX_DELAY);
    HAL_StatusTypeDef t_os_sp_status = HAL_I2C_Mem_Write(&hi2c1,
                                                        _addr,
                                                        TempSensorRegisters_s::T_OVER_SETPOINT,
                                                        I2C_MEMADD_SIZE_8BIT,
                                                        os_bytes,
                                                        2,
                                                        HAL_MAX_DELAY);
}

void TempSensorInterface::readTempValue() {
    uint8_t rx_data[2];
    HAL_StatusTypeDef temp_status = HAL_I2C_Mem_Read(&hi2c1,
                                                    _addr,
                                                    TempSensorRegisters_s::TEMP_VALUE,
                                                    I2C_MEMADD_SIZE_16BIT,
                                                    rx_data,
                                                    2,
                                                    HAL_MAX_DELAY);
    if (temp_status == HAL_OK) {
        int16_t raw_temp = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        raw_temp = raw_temp >> 4;
        _sensor_data.temp_value = raw_temp * 0.0625f;
    }
}