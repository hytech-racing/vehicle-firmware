/* */
#include "Hotswap.h"
#include "HT_I2C.h"
#include <cstring>

Hotswap::Config_s config = {GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_11};
Hotswap HS5066(config); //must split the hotswap extern and constructor call

// --------------------------- Initialization Functions ---------------------------------
bool Hotswap::init() {
    /* Initialize Hotswap by checking device status, reading previous state (eeprom), checking for faults before clearing
    * Unmask the faults (so all pull SMBA line) and retries to 4
    * We only check the blackbox eeprom at start (otherwise just exploit current state for interrupts)
    * LEAD-ASK: Must decide what to do with the eeprom/fault data (decode? reflect changes in PDB or simply send through CAN)
    * TODO: Check if retries are reset to 0 or stay at 4 everytime set4Retry function called */
    
    // HAL_I2C_IsDeviceReady( instance, device address, trials, timeout )
    _status = HAL_I2C_IsDeviceReady( &hi2c1, address, 3, I2C_TIMEOUT_MS );
    if (_status == 1) return 0;

    BlackboxRecord r;
    if(readBlackBoxEEPROM(r)) {
        //Send through CAN the previous state for the fault? or potentially decode the eepromstate first using the ram
    };

    _fault_word = _readWord(CMD_DIAGNOSTIC_WORD);

    set4Retry(); //does this reset retries counter?
    clearFaults();
    unmaskFaults();
}

void Hotswap::set4Retry() {
    /* By default, setting !Retry pin to GND results in infinite retries when fault occurs, setting it to VCC result in no retries. 
    Need to change DEVICE_SETUP1 register to override to 4 retries, bit 5-7 of register map to # of retries
    Check datasheet page 74 to change to either 0, 1, 2, 4, 8, 16, or infinite retries, 4 retries -> register[5:7] = 100b */

    uint8_t value;

    // HAL_I2C_Mem_Read( instance, deviceAddr, memAddr, memAddrSize, *data, dataSize, timeout)
    HAL_I2C_Mem_Read( &hi2c1, address, DEVICE_SETUP1, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);
    value = (value & ~RETRY_MASK) | RETRY_4; //RETRYMASK = 1110_0000, RETRY_4 = 1000_0000

    HAL_I2C_Mem_Write( &hi2c1, address, DEVICE_SETUP1, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY );
}

void Hotswap::clearFaults() {
    /* HAL_I2C_Mem_Transmit(instance, deviceAddr, memAddr, memAddrSize, timeout)
    * Only sends a command (not data to the mem address) to zero out the internal fault register */

    uint8_t command = CMD_CLEAR_FAULTS;
    HAL_I2C_Master_Transmit( &hi2c1, address, &command, 1, I2C_TIMEOUT_MS);
}

void Hotswap::unmaskFaults() {
    /* There are 16 total faults mapped to bits 0-15, we unmask all faults. (Faults mapping given in datasheet Page 59)
    * 1 is fault masked (SMDA unchanged) and 0 is unmasked (SMDA pulled low when fault occurs).
    * Note: low bytes sent first, meaning to mask bit 0, data = {0x01, 0x00} */

    uint8_t data[2] = {0x00, 0x00};
    HAL_I2C_Mem_Write( &hi2c1, address << 1, 0xD9, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );
}


// ------------------------- Reading/Writing Functions -----------------------
uint16_t Hotswap::_readWord( uint8_t command ) {
    // Send command (which represents the memory address) + read bytes into buffer
    
    uint8_t buffer[2];
    HAL_I2C_Mem_Read( &hi2c1, address, command, I2C_MEMADD_SIZE_8BIT, buffer, 2, I2C_TIMEOUT_MS );

    return buffer[1] << 8 | buffer[0]; //low bytes are sent first
}

uint16_t Hotswap::readDecodedTelemetry(uint8_t command, float R, float b, float m){
    /* PMbus Conversion: X = (Y * 10^(-R) - b)/m */

    uint16_t raw = _readWord(command);
    return (raw * pow(10.0f, -R) - b) / m;
}

//Whenever a warning happens, collect telemetry data (at minimum for the warning)
void Hotswap::readTelemetry() {
    /* Page 81 - 83 for R, b, m values
    * RSNS_mOhm = 2 (mOhms) used for m calculation
    * Page 6 -> CL to ground -> overcurrent threshold = 50mV for Iin and others */

    _Vout = readDecodedTelemetry(CMD_READ_VOUT, -2, 455.0f, 4596.0f);
    _Vin = readDecodedTelemetry(CMD_READ_VIN, -2, 255.0f, 4596.0f);
    _Iin = readDecodedTelemetry(CMD_READ_IIN, -2, 237.03, 7583.3 * 2);
    _Pin = readDecodedTelemetry(CMD_READ_PIN, -2, 455.0f, 4596.0f);
    _Temp = readDecodedTelemetry(CMD_READ_TEMP,  -2, 262437, 100);
}

void Hotswap::shutOff() {
    //Only when an temp sense goes out of control on the main line (read datahseet)
    uint8_t data = OPERATION_OFF;
    HAL_I2C_Mem_Write(&hi2c1, address, CMD_OPERATION, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
}

// ------------------------ Interrupt handlers --------------------------
/* View full table for all faults in datasheet page 59 */

void Hotswap::smbaIrqHandler() {
    _alert_pending = true;
}

void Hotswap::pgdIrqHandler() {
    //Need to rewrite the PGOOD stuff to properly handler alert
    powerGood = HAL_GPIO_ReadPin(_config.PGD_GPIO_PORT, _config.PGD_PIN) == GPIO_PIN_SET;
}

bool Hotswap::handleAlert() {
    /* Reads + saves fault word, reads current telemetry data, and only then manually clears faults register. 
    * Saves all interrupt data into a packed interrupt response object if fault occured for bookeeping.
    * Returns 1 if fault occured, 0 if all faults were resolved when handling */

    // bit to fault type mapping like unmask(), 1 represents fault
    _fault_word = _readWord(CMD_DIAGNOSTIC_WORD);

    readTelemetry();
    clearFaults();
    if(_fault_word == 0) return 0; //no faults detected (when faults cleared already)

    int_data = InterruptResponse({_Vin, _Iin, _Vout, _Pin, _Temp, _fault_word});
    return 1;
}

bool Hotswap::readBlackBoxEEPROM(BlackboxRecord& r) {
    /* Hotswap EEPROM data is read from initialization
    * EEPROM data also captures interrupt instance data, but rereading telemetry is simpler
    * (Can be updated in the future) */

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