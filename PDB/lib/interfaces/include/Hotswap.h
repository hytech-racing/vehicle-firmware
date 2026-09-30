#ifndef HOTSWAP_H
#define HOTSWAP_H

#include <cstdint>
#include <stm32h7xx_hal.h>

class Hotswap
{
public:
    void Init();

    void ReadCurrent();
    void ReadVoltage();
    void ReadFault();

    void ShutOff();
    void TurnOn();
    void ClearFaults();

private:
    static constexpr std::uint8_t CMD_OPERATION       = 0x01;
    static constexpr std::uint8_t CMD_CLEAR_FAULTS    = 0x03;

    static constexpr std::uint8_t CMD_READ_VIN        = 0x88;
    static constexpr std::uint8_t CMD_READ_IIN        = 0x89;

    static constexpr std::uint8_t CMD_DIAGNOSTIC_WORD = 0xE1;

    static constexpr std::uint8_t OPERATION_OFF       = 0x00;
    static constexpr std::uint8_t OPERATION_ON        = 0x80;

    static constexpr std::uint32_t I2C_TIMEOUT_MS      = 10;

    static constexpr std::uint8_t address = 0x15 << 1;

    Config_s _config;

    float _voltage_V = 0.0f;
    float _current_A = 0.0f;
    std::uint16_t _fault_word = 0;

    void _ReadWord(
        std::uint8_t command,
        std::uint16_t &stored_data
    );

    void _WriteByte(
        std::uint8_t command,
        std::uint8_t send_data
    );
};

#endif