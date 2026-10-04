#ifndef HOTSWAP_H
#define HOTSWAP_H

#include <cstdint>
#include <stm32h7xx_hal.h>

class Hotswap {
public:
    struct BlackboxRecord {
        uint8_t  ram[7];
        uint8_t  timer;
        uint16_t statusWord;
        uint8_t  statusMfr;
        uint16_t statusMfr2;
        uint8_t  statusInput;
        uint16_t vinPeakRaw, iinPeakRaw, pinPeakRaw, tempPeakRaw;
    };
    struct Config_s {
        GPIO_TypeDef*  SMBA_PORT;
        uint16_t  SMBA_PIN;
        GPIO_TypeDef*  PGD_GPIO_PORT;
        uint16_t  PGD_PIN;
    };

    struct InterruptResponse{
        float Vin, Iin, Vout, Pin, Temp;
        uint16_t faults;
    };

    explicit Hotswap(const Config_s& config) : _config(config) {}
    
    InterruptResponse int_data;

    bool init();
    void set4Retry();
    void unmaskFaults();

    void readInputCurrent();
    void readOutputVoltage();
    void readInputVoltage();
    void readInputPower();
    void readTemp();
    void readFault();

    void smbaIrqHandler();
    void pgdIrqHandler();
    bool handleAlert();
    void readTelemetry();
    bool readBlackBoxEEPROM(BlackboxRecord& r);
    float _decoder(uint16_t raw, float R, float b, float m);

    void shutOff();
    void clearFaults();

    Config_s _config;

private:
    volatile bool _alert_pending = false;
    static constexpr uint8_t CMD_OPERATION       = 0x01;
    static constexpr uint8_t CMD_CLEAR_FAULTS    = 0x03;

    static constexpr uint8_t DEVICE_SETUP1       = 0xCC;
    static constexpr uint8_t RETRY_MASK          = 0xE0;
    static constexpr uint8_t RETRY_4             = 0x80;

    static constexpr uint8_t CMD_READ_VIN        = 0x88;
    static constexpr uint8_t CMD_READ_IIN        = 0x89;
    static constexpr uint8_t CMD_READ_VOUT       = 0x8B;
    static constexpr uint8_t CMD_READ_TEMP       = 0x8D;
    static constexpr uint8_t CMD_READ_POWER      = 0x97;
    static constexpr uint8_t CMD_DIAGNOSTIC_WORD = 0xE1;

    static constexpr uint8_t OPERATION_OFF       = 0x00;

    static constexpr uint8_t CMD_FETCH_BB_EEPROM = 0xEB; //copies the eeprom into shadow registers
    static constexpr uint8_t CMD_READ_BB_EEPROM  = 0xF4; //reads the eeprom

    static constexpr uint32_t I2C_TIMEOUT_MS      = 10;

    static constexpr uint8_t address = 0x15 << 1; //shifted to accomodate R/W! bit that hal automatically changes

    float _Vin = 0.0f;
    float _Iin = 0.0f;
    float _Vout = 0.0f;
    float _Pin = 0.0f;
    float _Temp = 0.0f;
    uint16_t _fault_word = 0;
    volatile bool powerGood;
    HAL_StatusTypeDef _status;

    uint16_t _readWord(uint8_t command);
};

extern Hotswap HS5066;

#endif