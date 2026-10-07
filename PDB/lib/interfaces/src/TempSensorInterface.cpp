#include "TempSensorInterface.hpp"
#include <math.h>


HAL_StatusTypeDef TempSensorInterface::initAllSensors()
{
    // Try every sensor, even if an earlier one fails
    HAL_StatusTypeDef result = HAL_OK;
    for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
    {
        if (initSensor(i) != HAL_OK)
        {
            result = HAL_ERROR;
        }
    }
    return result;
}

HAL_StatusTypeDef TempSensorInterface::initSensor(uint8_t index)
{
    if (!_isValidIndex(index) || _hi2c == nullptr)
    {
        return HAL_ERROR;
    }
    TempSensor_s &sensor = _all_temp_sensors[index];
    sensor.status.is_initialized = false;

    // The A2:A0 address is latched after the device has seen its address twice (datasheet p. 16)
    // 1st transaction. Force normal continuous conversion
    uint8_t config = temp_sensor_default_params::CONFIG_NORMAL;
    if (writeRegister(sensor, TempSensorRegisterAddresses_s::CONFIG, &config, 1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 2nd transaction. Set the address pointer register on the temperature register so every
    // read after this doesn't need to rewrite the address pointer register
    if (setAddressPointerRegister(sensor, TempSensorRegisterAddresses_s::TEMP_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    sensor.status.is_initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef TempSensorInterface::readAllSensorsTemp()
{
    // Read every sensor, even if an earlier one fails
    HAL_StatusTypeDef result = HAL_OK;
    for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
    {
        if (readSensorTemp(i) == HAL_ERROR)
        {
            result = HAL_ERROR;
        }
    }
    return result;
}

HAL_StatusTypeDef TempSensorInterface::readSensorTemp(uint8_t index)
{
    if (!_isValidIndex(index))
    {
        return HAL_ERROR;
    }
    TempSensor_s &sensor = _all_temp_sensors[index];

    // Every read/write restarts the conversion, and reading before it finishes aborts it and returns the previous result
    if ((HAL_GetTick() - sensor.last_bus_tick) < temp_sensor_default_params::MIN_READ_INTERVAL_MS)
    {
        return HAL_BUSY;
    }

    // Missing somehow (or never initialized): retry init. Its first reading becomes available on a later call.
    if (!sensor.status.is_initialized)
    {
        return (initSensor(index) == HAL_OK) ? HAL_BUSY : HAL_ERROR;
    }

    uint8_t buffer[2];
    if (readRegister(sensor, TempSensorRegisterAddresses_s::TEMP_VALUE, buffer, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    sensor.data.raw_temp = static_cast<uint16_t>((buffer[0] << 8) | (buffer[1] & 0xF0));
    sensor.data.temp_degC = decodeTemp(buffer[0], buffer[1]);
    sensor.data.last_update_tick = HAL_GetTick();

    sensor.status.is_out_of_range = (sensor.data.temp_degC < temp_sensor_default_params::TEMP_MIN_C) ||
                                    (sensor.data.temp_degC > temp_sensor_default_params::TEMP_MAX_C);

    updateOvertemp(sensor);
    return HAL_OK;
}

const TempSensorData_s *TempSensorInterface::getData(uint8_t index) const
{
    return _isValidIndex(index) ? &_all_temp_sensors[index].data : nullptr;
}

const TempSensorStatus_s *TempSensorInterface::getStatus(uint8_t index) const
{
    return _isValidIndex(index) ? &_all_temp_sensors[index].status : nullptr;
}

float TempSensorInterface::getTemp(uint8_t index) const
{
    return _isValidIndex(index) ? _all_temp_sensors[index].data.temp_degC : NAN;
}

bool TempSensorInterface::isOvertemp(uint8_t index) const
{
    return _isValidIndex(index) && _all_temp_sensors[index].status.is_overtemp;
}

bool TempSensorInterface::isOnline(uint8_t index) const
{
    return _isValidIndex(index) && _all_temp_sensors[index].status.is_online;
}

bool TempSensorInterface::isFresh(uint8_t index, uint32_t max_age_ms) const
{
    if (!_isValidIndex(index) || isnan(_all_temp_sensors[index].data.temp_degC))
    {
        return false;
    }
    return (HAL_GetTick() - _all_temp_sensors[index].data.last_update_tick) <= max_age_ms;
}

bool TempSensorInterface::anyOvertemp() const
{
    for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
    {
        if (_all_temp_sensors[i].status.is_overtemp)
        {
            return true;
        }
    }
    return false;
}

bool TempSensorInterface::allOnline() const
{
    for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
    {
        if (!_all_temp_sensors[i].status.is_online)
        {
            return false;
        }
    }
    return true;
}

float TempSensorInterface::getMaxTemp() const
{
    float max_t = NAN;
    for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
    {
        float t = _all_temp_sensors[i].data.temp_degC;
        if (!isnan(t) && (isnan(max_t) || t > max_t))
        {
            max_t = t;
        }
    }
    return max_t;
}

float TempSensorInterface::decodeTemp(uint8_t msb, uint8_t lsb)
{
    int32_t raw = (static_cast<int32_t>(msb) << 8) | (lsb & 0xF0);
    if (raw & 0x8000)
    {
        raw -= 0x10000; // sign-extend 16-bit two's complement
    }
    return static_cast<float>(raw) / 256.0f; // (raw >> 4) / 16
}

bool TempSensorInterface::_isValidIndex(uint8_t index) const
{
    return index < temp_sensor_default_params::NUM_SENSORS;
}

HAL_StatusTypeDef TempSensorInterface::setAddressPointerRegister(TempSensor_s &sensor, uint8_t desired_register)
{
    if (sensor.address_pointer == desired_register)
    {
        return HAL_OK; // If desired address pointer == current address pointer register, then move on
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c,
                                                    sensor.hal_address,
                                                    &desired_register,
                                                    1,
                                                    temp_sensor_default_params::I2C_TIMEOUT_MS
    );
    sensor.address_pointer = (status == HAL_OK) ? desired_register : temp_sensor_default_params::POINTER_UNKNOWN;
    return _updateSensorStatus(sensor, status);
}



HAL_StatusTypeDef TempSensorInterface::readRegister(TempSensor_s &sensor,
                                                    uint8_t desired_register,
                                                    uint8_t *buffer,
                                                    uint16_t length
)
{
    if (setAddressPointerRegister(sensor, desired_register) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(_hi2c,
                                                    sensor.hal_address,
                                                    buffer,
                                                    length,
                                                    temp_sensor_default_params::I2C_TIMEOUT_MS
    );
    if (status != HAL_OK)
    {
        sensor.address_pointer = temp_sensor_default_params::POINTER_UNKNOWN; // re-send it next time
    }
    return _updateSensorStatus(sensor, status);
}

HAL_StatusTypeDef TempSensorInterface::writeRegister(TempSensor_s &sensor,
                                                    uint8_t desired_register,
                                                    uint8_t *buffer,
                                                    uint16_t length
)
{
    // Pointer byte followed by data in one transaction (datasheet Fig. 15)
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(_hi2c,
                                                sensor.hal_address,
                                                desired_register,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buffer,
                                                length,
                                                temp_sensor_default_params::I2C_TIMEOUT_MS
    );
    sensor.address_pointer = (status == HAL_OK) ? desired_register : temp_sensor_default_params::POINTER_UNKNOWN;
    return _updateSensorStatus(sensor, status);
}

HAL_StatusTypeDef TempSensorInterface::_updateSensorStatus(TempSensor_s &sensor, HAL_StatusTypeDef status)
{
    sensor.last_bus_tick = HAL_GetTick();

    if (status == HAL_OK)
    {
        sensor.status.consecutive_errors = 0;
        sensor.status.is_online = true;
    }
    else
    {
        sensor.status.error_count++;
        if (sensor.status.consecutive_errors < temp_sensor_default_params::MAX_CONSEC_ERRORS)
        {
            sensor.status.consecutive_errors++;
        }
        if (sensor.status.consecutive_errors >= temp_sensor_default_params::MAX_CONSEC_ERRORS)
        {
            sensor.status.is_online = false;
        }
    }
    return status;
}

void TempSensorInterface::updateOvertemp(TempSensor_s &sensor)
{
    // Software comparator with hysteresis: set at/above t_os_sp, clear below t_hyst_sp, otherwise hold
    if (sensor.data.temp_degC >= temp_sensor_default_params::OVERTEMP_SHUTDOWN_SETPOINT)
    {
        sensor.status.is_overtemp = true;
    }
    else if (sensor.data.temp_degC < temp_sensor_default_params::HYSTERESIS_SETPOINT)
    {
        sensor.status.is_overtemp = false;
    }
}