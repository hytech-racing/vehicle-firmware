#include "Hotswap.h"

Hotswap::Hotswap(const Config_s &config)
    : _config(config)
{
}

void Hotswap::Init()
{
    _last_status = HAL_I2C_IsDeviceReady(
        _config.hi2c,
        static_cast<std::uint16_t>(_config.address),
        3,
        I2C_TIMEOUT_MS
    );
}

void Hotswap::_ReadWord(
    std::uint8_t command,
    std::uint16_t &data
)
{
    std::uint8_t buffer[2];

    HAL_I2C_Mem_Read(
        &hi2c1,
        static_cast<std::uint16_t>(_config.address),
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