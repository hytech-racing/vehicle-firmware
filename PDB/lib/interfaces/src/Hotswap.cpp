#include "Hotswap.h"

Hotswap::Hotswap(const Config_s &config)
    : _config(config)
{
}

void Hotswap::Init()
{
    _status = HAL_I2C_IsDeviceReady(
        &hi2c1,
        address,
        3,
        I2C_TIMEOUT_ms 
    )

    set4retry();
    clearFaults();
    unmaskFaults();
}

void Hotswap::unmaskFaults()
{
    uint8_t data[2] = {0x00, 0x00};

    return HAL_I2C_Mem_Write(
        hi2c,
        address << 1,
        0xD9,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        HAL_MAX_DELAY
    );
}

/*
There are 16 total faults labeled 0-15 correpsonding to bits 0-15
the data we send contains the 16 bits the correspond to the total faults, with 1 meaning the fault is masked and 0 meaning the fault is unmasked.
For example: Data = 0000 0000 0100 0100
mask bit 2 and 6
Bit 2 - overtemp masked
Bit 6 - Fet Fail asked

Note that low bytes are sent first, meaning to mask bit 0, data = {0x01, 0x00}

View full table for all faults in datasheet page 59
*/

void Hotswap::Set4retry(
    std::I2C_HandleTypeDef *hi2c
)
{
    std::uint8_t value;

    HAL_I2C_Mem_Read(
        hi2c,
        Address,
        DEVICE_SETUP1,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        i2c_timeout_ms
    );

    value = (value & ~RETRY_MASK) | RETRY_4;

    HAL_I2C_Mem_Write(
        hi2c,
        M5066_ADDR,
        DEVICE_SETUP1,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        HAL_MAX_DELAY
    );
}

/*
By default, Setting !Retry pin to ground will cause the M5066 to retry infinite times when fault occurs, while setting it to VCC will cause the M5066 to not retry at all. 
To overide this and set the number of retries to 4, we need to change DEVICE_SETUP1 register
A mask is needed because retry settings only corresponds to bits 5-7 of the register, so we need to change the rigister bits 5-7 without changing the other bits
Essentially, we need to read the register, mask out bits 5-7, and then set bits 5-7 to the value corresponding to 4 retries.
check datasheet page 74 to change to either 0, 1, 2, 4, 8, 16, or infinite retries
*/

void Hotswap::_ReadWord( //send command + read bytes
    std::uint8_t command,
    std::uint16_t &stored_data
)
{
    std::uint8_t buffer[2];

    HAL_I2C_Mem_Read(
        &hi2c1,
        address,
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

//low bytes are sent first, so we need to shift the high byte left by 8 bits and then OR it with the low byte to get the full 16 bit value

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
    _ReadWord(CMD_DIAGNOSTIC_WORD, _fault_word); 
}

/*
Similar situation as unmaskFaults(); but 1 means fault present and 0 means no fault present.
For example: fault word = 0000 0000 0100 0100
Fault for bit 2 and 6
Bit 2 - overtemp fault
Bit 6 - Fet Fail

View full table for all faults in datasheet page 59
*/

void Hotswap::_WriteByte( // send command only
    std::uint8_t command,
    std::uint8_t send_data
)
{
    HAL_I2C_Mem_Write(
        hi2c1,
        address,
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

void Hotswap::ClearFaults()
{
    HAL_I2C_Master_Transmit(
        hi2c1,
        address,
        CMD_CLEAR_FAULTS,
        1,
        I2C_TIMEOUT_MS
    );
}

void Hotswap::smbaIrqHandler()
{
    lastFault = readFault();
    clearFaults();
}

//clear faults after every read fault so we know if it ever happens again

void Hotswap::pgdIrqHandler()
{
    powerGood = HAL_GPIO_ReadPin(PGD_GPIO_Port, PGD_Pin) == GPIO_PIN_SET;
}

//can decide later what to do if power not good