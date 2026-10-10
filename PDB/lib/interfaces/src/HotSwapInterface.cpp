#include "HotSwapInterface.hpp"
#include <cstring>


HAL_StatusTypeDef HotSwapInterface::initHotswap(I2C_HandleTypeDef *hi2c)
{
    /**
     * @note Hotswap will already be on when MCU recieves power, thus when we "initialize" the Hotswap,
     *       by checking device status, reading previous state (eeprom), checking for faults before clearing
    */

    _hi2c = hi2c;
    if (_hi2c == nullptr)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef result = HAL_OK;

    if (HAL_I2C_IsDeviceReady( _hi2c, hotswap_info.hal_address, 3, hotswap_default_params::I2C_TIMEOUT_MS ) != HAL_OK)
    {
        return HAL_ERROR;   // Chip not answering: don't send it any configuration
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
    pinMode(hotswap_info._config.SMBA_PIN, INPUT);   // Open-drain, active low; polled in update()

    hotswap_info.status.is_initialized = (result == HAL_OK);
    return result;
}

void HotSwapInterface::checkPreviousFaults()
{
    // Keep the blackbox for the rest of the run; getBlackbox() / isBlackboxValid() expose it
    hotswap_info.is_blackbox_valid = (readBlackBoxEEPROM(hotswap_info.blackbox) == HAL_OK);
    //TODO: Send through CAN the previous state for the fault? or potentially decode the eepromstate first using the ram
    hotswap_info.diagnostic_word = readWord(HotSwapTelemetryRegisters_s::DIAGNOSTIC_WORD_READ_REGISTER);  // readWord returns the register; sendCommand only returns HAL status
}

HAL_StatusTypeDef HotSwapInterface::set4Retry()
{
    /* By default (DEVICE_SETUP1 bits 7:5 = 000) the RETRY pin decides: GND or floating = retry forever,
    *  VDD = latch off on the first fault. Writing 100 to bits 7:5 overrides the pin with 4 retries
    *  (see RETRY_MASK / RETRY_NUM in HotSwapInterface.hpp, datasheet Table 7-64). */

    uint8_t value = 0;

    // Read-modify-write so only the retry bits change; don't write anything if the read failed.
    // Bits 4:0 hold the current limit settings and write protect, which must not be overwritten.
    if (_readRegister(HotSwapConfigurationRegisters_s::DEVICE_SETUP1_REGISTER, &value, 1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Example, read value = 0001 0100 (retry 000, bit 4 and bit 2 set):
    //   value & ~RETRY_MASK  = 0001 0100 & 0001 1111 = 0001 0100   clear bits 7:5, keep bits 4:0
    //   ...   | RETRY_NUM      = 0001 0100 | 1000 0000 = 1001 0100   bits 7:5 = 100 (retry 4 times)
    value = (value & ~hotswap_default_params::RETRY_MASK) | hotswap_default_params::RETRY_NUM;

    return _writeRegister(HotSwapConfigurationRegisters_s::DEVICE_SETUP1_REGISTER, &value, 1);
}

HAL_StatusTypeDef HotSwapInterface::clearFaults()
{
    return _sendCommand(HotSwapControlRegisters_s::CLEAR_FAULTS_REGISTER);
}

HAL_StatusTypeDef HotSwapInterface::unmaskFaults()
{
    /* There are 16 total faults mapped to bits 0-15, we unmask all faults. (Faults mapping given in datasheet Page 59)
    * 1 is fault masked (SMDA unchanged) and 0 is unmasked (SMDA pulled low when fault occurs).
    * Note: low bytes sent first, meaning to mask bit 0, data = {0x01, 0x00} */

    uint8_t data[2] = {0x00, 0x00};
    return _writeRegister(HotSwapConfigurationRegisters_s::ALERT_MASK_REGISTER, data, 2);
}


// ------------------------- Reading/Writing Functions -----------------------
uint16_t HotSwapInterface::readWord( uint8_t command )
{
    // Read word and combine according to lowest byte being sent first
    uint8_t buffer[2] = {0, 0};
    decode_status = _readRegister(command, buffer, 2);

    return buffer[1] << 8 | buffer[0]; //low bytes are sent first
}

float HotSwapInterface::readDecodedTelemetry(uint8_t command, TelemOptions_e telem_class)
{
    /* PMBus conversion: X = (Y * 10^(-R) - b) / m */
    const PMBusCoefficients_s &coeffs = COEFFICIENTS[static_cast<size_t>(telem_class)]; //Finds correct coefficients

    uint16_t raw = readWord(command) & 0x0FFF; //Lowest 12 bit matter
    return (raw * powf(10.0f, -coeffs.R) - coeffs.b) / coeffs.m;
}

//Whenever a warning happens, collect telemetry data (at minimum for the warning)
HAL_StatusTypeDef HotSwapInterface::readTelemetry()
{
    /* Page 81 - 83 for R, b, m values
    * RSNS_mOhm = 2 (mOhms) used for m calculation
    * Page 6 -> CL to ground -> overcurrent threshold = 50mV for Iin and others */

    HAL_StatusTypeDef result = HAL_OK;


    hotswap_info.data.Vout = readDecodedTelemetry(HotSwapTelemetryRegisters_s::READ_VOUT_REGISTER, TelemOptions_e::VOUT);
    if (decode_status != HAL_OK)
    {
        result = HAL_ERROR;
    }

    hotswap_info.data.Vin = readDecodedTelemetry(HotSwapTelemetryRegisters_s::READ_VIN_REGISTER, TelemOptions_e::VIN);
    if (decode_status != HAL_OK)
    {
        result = HAL_ERROR;
    }

    hotswap_info.data.Iin = readDecodedTelemetry(HotSwapTelemetryRegisters_s::READ_IIN_REGISTER, TelemOptions_e::IIN);
    if (decode_status != HAL_OK)
    {
        result = HAL_ERROR;
    }

    hotswap_info.data.Pin = readDecodedTelemetry(HotSwapTelemetryRegisters_s::READ_PIN_REGISTER, TelemOptions_e::PIN);

    if (decode_status != HAL_OK)
    {
        result = HAL_ERROR;
    }

    hotswap_info.data.temp = readDecodedTelemetry(HotSwapTelemetryRegisters_s::READ_TEMPERATURE_1_REGISTER,  TelemOptions_e::TEMP);
    if (decode_status != HAL_OK)
    {
        result = HAL_ERROR;
    }

    hotswap_info.is_power_good = digitalRead(hotswap_info._config.PGD_PIN);

    return result;
}

HAL_StatusTypeDef HotSwapInterface::shutOff()
{
    //Only when an temp sense goes out of control on the main line (read datahseet)
    // OPERATION (01h) is a read/write byte: 80h = FET on, 00h = FET off
    uint8_t operation_off = HotSwapControlCommands_s::OPERATION_OFF_CMD;
    return _writeRegister(HotSwapControlRegisters_s::OPERATION_REGISTER, &operation_off, 1);
}

// ------------------------ Interrupt handlers --------------------------
/* View full table for all faults in datasheet page 59 */


HAL_StatusTypeDef HotSwapInterface::handleAlert()
{
    /* Reads + saves fault word, reads current telemetry data, and only then manually clears faults register. */

    // bit to fault type mapping like unmask(), 1 represents fault
    hotswap_info.diagnostic_word = readWord(HotSwapTelemetryRegisters_s::DIAGNOSTIC_WORD_READ_REGISTER);  // readWord returns the register; sendCommand only returns HAL status //can be moved to telemetry perhaps

    HAL_StatusTypeDef result = HAL_OK;
    if (readTelemetry() != HAL_OK) {
        result = HAL_ERROR;
    };

    if (clearFaults() != HAL_OK) {
        result = HAL_ERROR;
    };

    return result;
}

HAL_StatusTypeDef HotSwapInterface::update()
{
    if (!hotswap_info.status.is_initialized)
    {
        return HAL_ERROR;
    }

    // SMBA is open-drain, active low: low = an unmasked fault/warning is active
    const bool is_smba_asserted = (digitalRead(hotswap_info._config.SMBA_PIN) == LOW);
    if (is_smba_asserted)
    {
        return handleAlert();   // Also refreshes telemetry
    }
    return readTelemetry();
}

HAL_StatusTypeDef HotSwapInterface::readBlackBoxEEPROM(HotSwapBlackboxEEPROM_s &blackbox_reg_copy)
{
    /*
    * Hotswap EEPROM data is read during initialization of hotswap
    * EEPROM data also captures interrupt instance data, but rereading telemetry is simpler
    * (Can be updated in the future)
    */

    // 1. copy EEPROM -> shadow registers
    if (_sendCommand(HotSwapControlRegisters_s::FETCH_BB_EEPROM_REGISTER) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 2. block read: 1 count byte + 22 data bytes
    uint8_t buf[23] = {};
    if (_readRegister(HotSwapTelemetryRegisters_s::READ_BB_EEPROM_REGISTER, buf, 23) != HAL_OK || buf[0] != 22)
    {
        return HAL_ERROR;
    };

    // 3. split into fields
    const uint8_t* src_register = &buf[1]; // skip the count byte

    memcpy(blackbox_reg_copy.ram, src_register, 7); // copies the RAM bytes over (TO-DO: Decode them perhaps)

    blackbox_reg_copy.timer               = src_register[7];
    blackbox_reg_copy.status_word         = src_register[8]  | (src_register[9]  << 8);
    blackbox_reg_copy.status_1            = src_register[10];                               // STATUS_MFR_SPECIFIC
    blackbox_reg_copy.status_2            = src_register[11] | (src_register[12] << 8);     // STATUS_MFR_SPECIFIC_2
    blackbox_reg_copy.status_input        = src_register[13];
    blackbox_reg_copy.voltage_in_peak_raw = src_register[14] | (src_register[15] << 8);
    blackbox_reg_copy.current_in_peak_raw = src_register[16] | (src_register[17] << 8);
    blackbox_reg_copy.power_in_peak_raw   = src_register[18] | (src_register[19] << 8);
    blackbox_reg_copy.temp_peak_raw       = src_register[20] | (src_register[21] << 8);

    return HAL_OK;
}

HAL_StatusTypeDef HotSwapInterface::_sendCommand(uint8_t desired_command)
{
    /*
    * PMBus send byte: START | addr+W | command | STOP
    * HAL_I2C_Master_Transmit(instance, deviceAddr, *data, size, timeout)
    */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c,
                                                    hotswap_info.hal_address,
                                                    &desired_command,
                                                    1,
                                                    hotswap_default_params::I2C_TIMEOUT_MS
    );
    return _updateSensorStatus(status);
}

HAL_StatusTypeDef HotSwapInterface::_writeRegister(uint8_t desired_register,
                                                uint8_t *buffer,
                                                uint16_t length
)
{
    /*
    * PMBus write: START | addr+W | register | data[0] ... data[length-1] | STOP
    * HAL_I2C_Mem_Write(instance, deviceAddr, memAddr, memAddrSize, *data, size, timeout)
    */
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

HAL_StatusTypeDef HotSwapInterface::_readRegister(uint8_t desired_register,
                                                uint8_t *buffer,
                                                uint16_t length
)
{
    /*
    * PMBus read: START | addr+W | register | RESTART | addr+R | data[0] ... data[length-1] | STOP
    * The received bytes are written into buffer (the caller's memory); status only says whether it worked.
    */
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

HAL_StatusTypeDef HotSwapInterface::_updateSensorStatus(HAL_StatusTypeDef status)
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
        else
        {
            hotswap_info.status.is_online = false;
        }
    }
    return status;
}