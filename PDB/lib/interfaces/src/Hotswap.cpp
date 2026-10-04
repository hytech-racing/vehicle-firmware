#include "Hotswap.h"
#include "HT_I2C.h"
#include <cstring>

Hotswap::Config_s config = {GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_11};
Hotswap HS5066(config); //must split the hotswap extern and constructor call

// --------------------------- Initialization Functions ---------------------------------
bool Hotswap::init() {
    // HAL_I2C_IsDeviceReady( instance, device address, trials, timeout )
    _status = HAL_I2C_IsDeviceReady( &hi2c1, address, 3, I2C_TIMEOUT_MS );
    if (_status == 1) return 0;

    BlackboxRecord r; //only check the blackbox eeprom at the beginning (otherwise just exploit current state)
    if(readBlackBoxEEPROM(r)) {
        //Send through CAN the previous state for the fault
        //Or potentially decode the eepromstate first using the ram
    };

    readFault();

    set4Retry();
    clearFaults();
    unmaskFaults();
}

/*
By default, Setting !Retry pin to ground will cause the M5066 to retry infinite times when fault occurs, while setting it to VCC will cause the M5066 to not retry at all. 
To overide this and set the number of retries to 4, we need to change DEVICE_SETUP1 register
A mask is needed because retry settings only corresponds to bits 5-7 of the register, so we need to change the rigister bits 5-7 without changing the other bits
Essentially, we need to read the register, mask out bits 5-7, and then set bits 5-7 to the value corresponding to 4 retries.
check datasheet page 74 to change to either 0, 1, 2, 4, 8, 16, or infinite retries
*/
void Hotswap::set4Retry() {
    std::uint8_t value;

    //HAL_I2C_Mem_Read( instance, deviceAddr, memAddr, memAddrSize, *data, dataSize, timeout)
    HAL_I2C_Mem_Read( &hi2c1, address, DEVICE_SETUP1, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);
    value = (value & ~RETRY_MASK) | RETRY_4;

    HAL_I2C_Mem_Write( &hi2c1, address, DEVICE_SETUP1, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY );
}

void Hotswap::clearFaults() {
    uint8_t command = CMD_CLEAR_FAULTS;
    //HAL_I2C_Mem_Transmit(instance, deviceAddr, memAddr, memAddrSize, timeout) - only sends a command (not a data to the mem address)
    HAL_I2C_Master_Transmit( &hi2c1, address, &command, 1, I2C_TIMEOUT_MS);
}

/*
There are 16 total faults mapped to bits 0-15, we unmask all faults.
1 is fault masked (SMDA unchanged) and 0 is unmasked (SMDA pulled low when fault occurs).
Note: low bytes sent first, meaning to mask bit 0, data = {0x01, 0x00}
View full table for all faults in datasheet page 59
*/
void Hotswap::unmaskFaults() {
    uint8_t data[2] = {0x00, 0x00};
    HAL_I2C_Mem_Write( &hi2c1, address << 1, 0xD9, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );
}


// ------------------------- Reading/Writing Functions -----------------------
//send command (which represents the memory address) + read bytes
uint16_t Hotswap::_readWord( std::uint8_t command) {
    uint8_t buffer[2];
    HAL_I2C_Mem_Read( &hi2c1, address, command, I2C_MEMADD_SIZE_8BIT, buffer, 2, I2C_TIMEOUT_MS );

    return buffer[1] << 8 | buffer[0]; //low bytes are sent first
}

float Hotswap::_decoder( uint16_t raw, float R, float b, float m) {
    return (raw * pow(10.0f, -R) - b) / m;  // PMbus Conversion: X = (Y * 10^(-R) - b)/m
}

/* Page 81 -> Read_VIN -> m = 4596.0, b = 255.0, R = -2 */

void Hotswap::readInputVoltage() {
    uint16_t raw =_readWord(CMD_READ_VIN);
    _Vin = _decoder(raw, -2, 255.0f, 4596.0f);
}

/*
Page 6 -> CL to ground -> overcurrent threshold = 50mV
Page 82 -> Read_IIN -> m = 7583.3 x RSNS_mOhm = 7583.3 x 2, b = 237.03, R = -2
*/

void Hotswap::readInputCurrent() {
    uint16_t raw = _readWord(CMD_READ_IIN);
    _Iin = _decoder(raw, -2, 237.03, 7583.3 * 2);
}

/* Page 81 -> Read_VIN -> m = 4596.0, b = 455.0, R = -2 */
void Hotswap::readOutputVoltage() {
    uint16_t raw = _readWord(CMD_READ_VOUT);
    _Vout = _decoder(raw, -2, 455.0f, 4596.0f);
}

/*
Page 82 -> Read_PIN -> m = 8511 x RSNS_mOhm, b = 6868, R = -4
*/
void Hotswap::readInputPower() {
    uint16_t raw = _readWord(CMD_READ_POWER);
    _Pin = _decoder(raw, -4, 6868, 2511 * 2);
}

/*
Page 81 -> Read_Temp -> m = 100, b = 26437, R = -2
*/
void Hotswap::readTemp() {
    uint16_t raw = _readWord(CMD_READ_TEMP);
    _Temp = _decoder(raw, -2, 262437, 100);
}

void Hotswap::shutOff() {
    uint8_t data = OPERATION_OFF;
    HAL_I2C_Mem_Write(&hi2c1, address, CMD_OPERATION, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
}

// ------------------------ Interrupt handlers --------------------------
/* View full table for all faults in datasheet page 59 */

void Hotswap::smbaIrqHandler() {
    _alert_pending = true;
}

void Hotswap::pgdIrqHandler() {
    powerGood = HAL_GPIO_ReadPin(_config.PGD_GPIO_PORT, _config.PGD_PIN) == GPIO_PIN_SET; //not used currently more for debugging
}

bool Hotswap::handleAlert() {
    _fault_word = _readWord(CMD_DIAGNOSTIC_WORD); // bit to fault type mapping like unmask, 1 represents fault
    
    readTelemetry(); //read current data for the handler
    clearFaults();  //clear faults manually once handled (faults read)
    if(_fault_word == 0) return 0; //no faults detected

    //packed response with all the data prior to the fault
    int_data = InterruptResponse({_Vin, _Iin, _Vout, _Pin,_Temp, _fault_word});
    return 1;
}

//Whenever a warning happens, can't do much but collect telemetry data
void Hotswap::readTelemetry() {
    readOutputVoltage();
    readInputVoltage();
    readInputCurrent();
    readInputPower();
    readTemp();
}

bool Hotswap::readBlackBoxEEPROM(BlackboxRecord& r) {
    // 1. copy EEPROM -> shadow registers
    uint8_t cmd = CMD_FETCH_BB_EEPROM;
    _status = HAL_I2C_Master_Transmit(&hi2c1, address, &cmd, 1, I2C_TIMEOUT_MS);
    if (_status != HAL_OK) return false;

    // 2. block read: 1 count byte + 22 data bytes
    uint8_t buf[23];
    _status = HAL_I2C_Mem_Read(&hi2c1, address, CMD_READ_BB_EEPROM, I2C_MEMADD_SIZE_8BIT,
            buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (_status != HAL_OK || buf[0] != 22) return false; //checks also if the count is right

    // 3. split into fields
    const uint8_t* d = &buf[1]; //skips the count byte
    memcpy(r.ram, d, 7);
    r.timer       = d[7];
    r.statusWord  = d[8]  | (d[9]  << 8);
    r.statusMfr   = d[10];
    r.statusMfr2  = d[11] | (d[12] << 8);
    r.statusInput = d[13];
    r.vinPeakRaw  = d[14] | (d[15] << 8);
    r.iinPeakRaw  = d[16] | (d[17] << 8);
    r.pinPeakRaw  = d[18] | (d[19] << 8);
    r.tempPeakRaw = d[20] | (d[21] << 8);
    return true;
}