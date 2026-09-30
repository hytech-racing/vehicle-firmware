#include "Hotswap.h"

Hotswap::Hotswap(const Config_s &config)
    : _config(config)
{
}

void Hotswap::Init()
{
    _status = HAL_I2C_IsDeviceReady(
        &hi2c1,
        address
        3,
        I2C_TIMEOUT_MS
    );
}

void Hotswap::_ReadWord(
    std::uint8_t command,
    std::uint16_t &stored_data
)
{
    std::uint8_t buffer[2];

    HAL_I2C_Mem_Read(
        &hi2c1,
        address
        command,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        2,
        I2C_TIMEOUT_MS
    );

    data =
        static_cast<std::uint16_t>(buffer[0]) |
        (static_cast<std::uint16_t>(buffer[1]) << 8);
}

void Hotswap::ReadVoltage()
{
    std::uint16_t raw = 0;
    _ReadWord(CMD_READ_VIN, raw);

    _voltage_V =
        (static_cast<float>(raw) * 100.0f - 255.0f)
        / 4596.0f;
}

/*

PMbus Conversion: X = (Y * 10^(-R) - b)/m

Page 81 -> Read_VIN
m = 4596.0
b = 255.0
R = -2

*/

void Hotswap::ReadCurrent()
{
    std::uint16_t raw = 0;
    _ReadWord(CMD_READ_IIN, raw);

    _current_A =
        (static_cast<float>(raw) * 100.0f - 237.03f)
        / 15166.6f;
}

/*

Page 6 -> CL to ground -> overcurrent threshold = 50mV

Page 82 -> Read_IN
m = 7583.3 x RSNS_mOhm
b = 237.03
R = -2

*/

void Hotswap::ReadFault()
{
    std::uint16_t raw = 0;
    _ReadWord(CMD_DIAGNOSTIC_WORD, _fault_word); 
}

void _WriteByte(
    std::uint8_t command,
    std::uint8_t send_data
)
{
    return HAL_I2C_Mem_Write(
        hi2c1,
        address
        command,
        I2C_MEMADD_SIZE_8BIT,
        &send_data,
        1,
        I2C_TIMEOUT_MS
    );
}

void Hotswap::ShutOff()
{
    _last_status = _WriteByte(
        CMD_OPERATION,
        OPERATION_OFF
    );
}

void Hotswap::TurnOn()
{
    _last_status = _WriteByte(
        CMD_OPERATION,
        OPERATION_ON
    );
}

void Hotswap::ClearFaults()
{
    HAL_I2C_Master_Transmit(
        hi2c1,
        address
        CMD_CLEAR_FAULTS,
        1,
        I2C_TIMEOUT_MS
    );
}