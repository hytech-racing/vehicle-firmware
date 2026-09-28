#ifndef HOTSWAP_H
#define HOTSWAP_H

#include <cstdint>
#include <stm32h7xx_hal.h>

class Hotswap
{
public:
    struct Config_s
    {
        std::uint8_t address = 0x15 << 1
        std::uint32_t rsense_uOhm;
        std::uint16_t current_limit_mV_x10;
    };
    explicit Hotswap(const Config_s &config);

    void Init();

    void ReadCurrent();
    void ReadVoltage();
    void ReadFault();

    void ShutOff();
    void TurnOn();
    void ClearFaults();

    float voltage_V() const
    {
        return _voltage_V;
    }

    float current_A() const
    {
        return _current_A;
    }

    std::uint16_t fault_word() const
    {
        return _fault_word;
    }

private:
    constexpr std::uint8_t CMD_OPERATION       = 0x01;
    constexpr std::uint8_t CMD_CLEAR_FAULTS    = 0x03;

    constexpr std::uint8_t CMD_READ_VIN        = 0x88;
    constexpr std::uint8_t CMD_READ_IIN        = 0x89;

    constexpr std::uint8_t CMD_DIAGNOSTIC_WORD = 0xE1;

    constexpr std::uint8_t OPERATION_OFF       = 0x00;
    constexpr std::uint8_t OPERATION_ON        = 0x80;

    constexpr std::uint32_t I2C_TIMEOUT_MS      = 10;

    Config_s _config;

    float _voltage_V = 0.0f;
    float _current_A = 0.0f;
    std::uint16_t _fault_word = 0;

    void _ReadWord(
        std::uint8_t command,
        std::uint16_t &data
    );

    void _WriteByte(
        std::uint8_t command,
        std::uint8_t data
    );
};

#endif