#include "TempSensorInterface.h"
// D5 one-shot mode? Y/N, currently no
void TempSensorInterface::initSensor() {
    uint8_t config_bits = 0x40; // interrupt mode, no one-shot mode, no SMBA
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
                                                        I2c_MEMADD_SIZE_16BIT,
                                                        &(_sensor_data.t_hyst_sp),
                                                        sizeof(_sensor_data.t_hyst_sp),
                                                        HAL_MAX_DELAY);
    HAL_StatusTypeDef t_os_sp_status = HAL_I2C_Mem_Write(&hi2c1,
                                                        _addr,
                                                        TempSensorRegisters_s::T_OVER_SETPOINT,
                                                        I2C_MEMADD_SIZE_16BIT,
                                                        &(_sensor_data.t_os_sp),
                                                        sizeof(_sensor_data.t_os_sp),
                                                        HAL_MAX_DELAY);
    
}

void TempSensorInterface::readTempValue() {
    uint8_t pData[2];
    HAL_StatusTypeDef temp_status = HAL_I2C_Mem_Read(&hi2c1,
                                                    _addr,
                                                    TempSensorRegisters_s::TEMP_VALUE,
                                                    I2C_MEMADD_SIZE_16BIT,
                                                    pData,
                                                    2,
                                                    HAL_MAX_DELAY);
    if (temp_status == HAL_OK) {
        int16_t raw_temp = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        raw_temp = raw_temp >> 4;
        _sensor_data.temp_value = raw_temp * 0.625f;
    }
}