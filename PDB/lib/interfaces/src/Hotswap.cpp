/* Anthony, this implementation follows your temperature sensor one
There are a few differences, the read function is rolled up together (no address pointer stuff, feels more efficient), and instead we sometimes 
send a command when data isn't being read (ie zeroing out the fault register)
Hardware fields, statuses and other stuff still needs to be added

General To-Do for HOTSWAP:
// Add accessor methods
// Compute hardware fields from the scchematics provided + datasheet
// Figure out exactly what you want to do with the EEPROM blackbox when initialized (aka faults in previous run)
// In initialization, add the slew rate requirements and other hardware consideration (Rachel)

For the .h header files
Also it came to my realization that alert mask bits do not map one on one with diagnostic words, will check that in shop
Also, I tried writing telemetry commands in a way you can add more in the future in the .h file without hardcoding into the main cpp, 
if you feel like it is really forced and there is a better way do look at that.
I also stuffed everything into a single hotswap_info struct and built it top down from that with the status and data registers
*/
#include "Hotswap.h"
#include "HT_I2C.h"
#include <cstring>

Config_s config = {PB5, PB11};
Hotswap HS5066(config); //must split the hotswap extern and constructor call

// --------------------------- Initialization Functions ---------------------------------

HAL_StatusTypeDef Hotswap::initHotswap() {
    /* Initialize Hotswap by checking device status, reading previous state (eeprom), checking for faults before clearing
    * Unmask faults (so all pull SMBA line) and retries to 4
    * Only check the blackbox eeprom at start (otherwise just exploit current state for interrupts)
    * LEAD-ASK: Must decide what to do with the eeprom/fault data (decode? reflect changes in PDB or simply send through CAN)
    * TODO: Check if retries are reset to 0 or stay at 4 everytime set4Retry function called */
    
    // HAL_I2C_IsDeviceReady( instance, device address, trials, timeout )
    HAL_StatusTypeDef result = HAL_OK;

    if (HAL_I2C_IsDeviceReady( &hi2c1, hotswap_info.hal_address, 3, hotswap_default_params::I2C_TIMEOUT_MS ) != HAL_OK)
    {
        result = HAL_ERROR;
    };

    checkPreviousFaults();

    if (set4Retry() != HAL_OK) 
    {
        result = HAL_ERROR;
    }; //does this reset retries counter?

    if (clearFaults() != HAL_OK) 
    {
        result = HAL_ERROR;
    };

    if (unmaskFaults() != HAL_OK) 
    {
        result = HAL_ERROR;
    };

    pinMode(hotswap_info._config.PGD_PIN, INPUT);

    return result;
}

void Hotswap::checkPreviousFaults() 
{
    BlackboxRecord r;
    if(readBlackBoxEEPROM(r) == HAL_OK) 
    {
        //Send through CAN the previous state for the fault? or potentially decode the eepromstate first using the ram
    };
    hotswap_info._fault_word = sendCommand(hotswap_commands::CMD_DIAGNOSTIC_WORD);
}

HAL_StatusTypeDef Hotswap::set4Retry() 
{
    /* By default, setting !Retry pin to GND results in infinite retries when fault occurs
                   setting it to VCC result in no retries
    Write to DEVICE_SETUP1 register to override to 4 retries, bit 5-7 of register map to # of retries
    Check datasheet page 74 to change to either 0, 1, 2, 4, 8, 16, or infinite retries, 4 retries -> register[5:7] = 100b */
    
    HAL_StatusTypeDef result = HAL_OK;
    uint8_t* value;

    if (readRegister(hotswap_commands::DEVICE_SETUP1, value, 1) != HAL_OK) 
    {
        result = HAL_ERROR;
    }
    
    //RETRYMASK = 1110_0000, RETRY_4 = 1000_0000
    *value = (*value & ~hotswap_default_params::RETRY_MASK) | hotswap_default_params::RETRY_4;
    
    if (writeRegister(hotswap_commands::DEVICE_SETUP1, value, 1) != HAL_OK)
    {
        result = HAL_ERROR;
    }
    
    return result;
}

HAL_StatusTypeDef Hotswap::clearFaults() 
{
    return sendCommand(hotswap_commands::CMD_CLEAR_FAULTS);
}

HAL_StatusTypeDef Hotswap::unmaskFaults() 
{
    /* There are 16 total faults mapped to bits 0-15, we unmask all faults. (Faults mapping given in datasheet Page 59)
    * 1 is fault masked (SMDA unchanged) and 0 is unmasked (SMDA pulled low when fault occurs).
    * Note: low bytes sent first, meaning to mask bit 0, data = {0x01, 0x00} */

    uint8_t data[2] = {0x00, 0x00};
    return writeRegister(hotswap_commands::CMD_UNMASK_FAULTS, data, 2);
}


// ------------------------- Reading/Writing Functions -----------------------
uint16_t Hotswap::readWord( uint8_t command ) 
{
    // Read word and combine according to lowest byte being sent first
    uint8_t buffer[2];
    decode_status = readRegister(command, buffer, 2);

    return buffer[1] << 8 | buffer[0]; //low bytes are sent first
}

float Hotswap::readDecodedTelemetry(uint8_t command, Telem telem_class)
{
    /* PMBus conversion: X = (Y * 10^(-R) - b) / m */
    const PmbusCoeff_s &coeffs = COEFFS[static_cast<size_t>(telem_class)]; //Finds correct coefficients

    uint16_t raw = readWord(command) & 0x0FFF; //Lowest 12 bit matter
    return (raw * powf(10.0f, -coeffs.R) - coeffs.b) / coeffs.m;
}

//Whenever a warning happens, collect telemetry data (at minimum for the warning)
HAL_StatusTypeDef Hotswap::readTelemetry() 
{
    /* Page 81 - 83 for R, b, m values
    * RSNS_mOhm = 2 (mOhms) used for m calculation
    * Page 6 -> CL to ground -> overcurrent threshold = 50mV for Iin and others */

    HAL_StatusTypeDef result = HAL_OK;

    
    hotswap_info.data._Vout = readDecodedTelemetry(hotswap_telemetry_requests::CMD_READ_VOUT, Telem::VOUT);
    if (decode_status != HAL_OK) 
    {
        result = HAL_ERROR;
    }

    hotswap_info.data._Vin = readDecodedTelemetry(hotswap_telemetry_requests::CMD_READ_VIN, Telem::VIN);
    if (decode_status != HAL_OK) 
    {
        result = HAL_ERROR;
    }

    hotswap_info.data._Iin = readDecodedTelemetry(hotswap_telemetry_requests::CMD_READ_IIN, Telem::IIN);
    if (decode_status != HAL_OK) 
    {
        result = HAL_ERROR;
    }

    hotswap_info.data._Pin = readDecodedTelemetry(hotswap_telemetry_requests::CMD_READ_PIN, Telem::PIN);
    
    if (decode_status != HAL_OK) 
    {
        result = HAL_ERROR;
    }
    
    hotswap_info.data._Temp = readDecodedTelemetry(hotswap_telemetry_requests::CMD_READ_TEMP,  Telem::TEMP);
    if (decode_status != HAL_OK) 
    {
        result = HAL_ERROR;
    }

    hotswap_info._powerGood = digitalRead(hotswap_info._config.PGD_PIN);

    return result;
}

HAL_StatusTypeDef Hotswap::shutOff() 
{
    //Only when an temp sense goes out of control on the main line (read datahseet)
    return sendCommand(hotswap_commands::CMD_OPERATION_OFF);
}

// ------------------------ Interrupt handlers --------------------------
/* View full table for all faults in datasheet page 59 */

void Hotswap::smbaIrqHandler() 
{
    hotswap_info._alert_pending = true;
}

/*void Hotswap::pgdIrqHandler() 
{
    //TO-DO: Need to rewrite the PGOOD stuff to properly handle alert
    powerGood = HAL_GPIO_ReadPin(_config.PGD_GPIO_PORT, _config.PGD_PIN) == GPIO_PIN_SET;
}*/

HAL_StatusTypeDef Hotswap::handleAlert() 
{
    /* Reads + saves fault word, reads current telemetry data, and only then manually clears faults register. */

    // bit to fault type mapping like unmask(), 1 represents fault
    hotswap_info._fault_word = sendCommand(hotswap_commands::CMD_DIAGNOSTIC_WORD); //can be moved to telemetry perhaps

    HAL_StatusTypeDef result = HAL_OK;
    if (readTelemetry() != HAL_OK) {
        result = HAL_ERROR;
    };

    if (clearFaults() != HAL_OK) {
        result = HAL_ERROR;
    };

    return result;
}

HAL_StatusTypeDef Hotswap::readBlackBoxEEPROM(BlackboxRecord& r) 
{
    /* Hotswap EEPROM data is read during initialization of hotswap
    * EEPROM data also captures interrupt instance data, but rereading telemetry is simpler
    * (Can be updated in the future) */

    // 1. copy EEPROM -> shadow registers
    HAL_StatusTypeDef result = HAL_OK;

    if (sendCommand(hotswap_commands::CMD_FETCH_BB_EEPROM) != HAL_OK)
    {
        result = HAL_ERROR;
    }

    // 2. block read: 1 count byte + 22 data bytes
    uint8_t buf[23];
    if (readRegister(hotswap_commands::CMD_READ_BB_EEPROM, buf, 23) != HAL_OK || buf[0] != 22) 
    {
        result = HAL_ERROR;
    };

    // 3. split into fields
    const uint8_t* d = &buf[1]; //skips the count byte

    memcpy(r.ram, d, 7); //copies the RAM bytes over (TO-DO: Decode them perhaps)

    r.timer       = d[7];
    r.statusWord  = d[8]  | (d[9]  << 8);
    r.statusMfr   = d[10];
    r.statusMfr2  = d[11] | (d[12] << 8);
    r.statusInput = d[13];
    r.vinPeakRaw  = d[14] | (d[15] << 8);
    r.iinPeakRaw  = d[16] | (d[17] << 8);
    r.pinPeakRaw  = d[18] | (d[19] << 8);
    r.tempPeakRaw = d[20] | (d[21] << 8);

    return result;
}

// ---------------------- Read/Write Functions (Inspired by Anthony's Temp Sensors) --------------------------
HAL_StatusTypeDef Hotswap::sendCommand(uint8_t desired_command)
{
    /* HAL_I2C_Mem_Transmit(instance, deviceAddr, memAddr, memAddrSize, timeout)
    * Uses the same function as redirecting the internal address pointer, but chip realizes it is a command */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c,
                                                hotswap_info.hal_address,
                                                &desired_command,
                                                I2C_MEMADD_SIZE_8BIT,
                                                hotswap_default_params::I2C_TIMEOUT_MS
    );
    return _updateSensorStatus(status);
}

HAL_StatusTypeDef Hotswap::writeRegister(uint8_t desired_register,
                                                    uint8_t *buffer,
                                                    uint16_t length
)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(_hi2c,
                                                hotswap_info.hal_address,
                                                desired_register,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buffer,
                                                length,
                                                hotswap_default_params::I2C_TIMEOUT_MS
    );
    return _updateSensorStatus(status);
}

HAL_StatusTypeDef Hotswap::readRegister(uint8_t desired_register,
                                                    uint8_t *buffer,
                                                    uint16_t length
)
{
    // HAL_I2C_Mem_Read( instance, deviceAddr, memAddr, memAddrSize, *data, dataSize, timeout)
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c,
                                                hotswap_info.hal_address,
                                                desired_register,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buffer,
                                                length,
                                                hotswap_default_params::I2C_TIMEOUT_MS
    );
    return _updateSensorStatus(status);
}

HAL_StatusTypeDef Hotswap::_updateSensorStatus(HAL_StatusTypeDef status)
{
    hotswap_info.last_bus_tick = HAL_GetTick();

    if (status == HAL_OK)
    {
        hotswap_info.status.consecutive_errors = 0;
        hotswap_info.status.is_online = true;
    }
    else
    {
        hotswap_info.status.error_count++;
        if (hotswap_info.status.consecutive_errors < hotswap_default_params::MAX_CONSEC_ERRORS)
        {
            hotswap_info.status.consecutive_errors++;
        }
        if (hotswap_info.status.consecutive_errors >= hotswap_default_params::MAX_CONSEC_ERRORS)
        {
            hotswap_info.status.is_online = false;
        }
    }
    return status;
}