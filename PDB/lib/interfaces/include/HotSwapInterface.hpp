#ifndef HOT_SWAP_INTERFACE_HPP
#define HOT_SWAP_INTERFACE_HPP

/**
 * @file HotSwapInterface.hpp
 * @brief Driver for the LM5066H hotswap controller on the PDB's main input.
 *
 * Signals to the MCU:
 *  - PGD  : open-drain power good, high when the output is up.
 *  - SMBA : open-drain alert, pulled low when an unmasked fault/warning occurs (ALERT_MASK, D8h).
 *  - PMBus over I2C1 at address 0x15 (ADR2/ADR1/ADR0 all tied to GND on PDB).
 *
 * Telemetry: 12-bit ADC values in PMBus "direct" format, decoded as X = (Y * 10^-R - b) / m with the
 *            coefficients in COEFFS (datasheet Table 7-72). Current and power coefficients depend on the current
 *            limit setting, the ADC range (1x/2x VCL) and the sense resistor (RSNS_MOHM).
*/

#include <cstdint>
#include <stm32h7xx_hal.h>
#include <Arduino.h>
#include <etl/singleton.h>


namespace hotswap_default_params
{
    static constexpr uint8_t ADDRESS = 0x15 << 1;           // shifted to accomodate R/W! bit that hal automatically changes
    static constexpr uint32_t I2C_TIMEOUT_MS    = 10;
    static constexpr uint8_t MAX_CONSEC_ERRORS  = 10;       // failed transactions before hotswap is offline

    static constexpr uint8_t RETRY_MASK  = 0xE0;            // 1110 0000: bits 7:5
    static constexpr uint8_t RETRY_NUM   = 0x80;            // 1000 0000: 100 in bits 7:5 = retry 4 times

    static constexpr float RSNS_MOHM = 2.0f;                // R8 on schematic

}

/**
 * @struct HotSwapControlRegisters_s
 * @note Command codes signaling which register we're talking to
*/
struct HotSwapControlRegisters_s
{
    static constexpr uint8_t OPERATION_REGISTER         = 0x01;   // R/W 1: write OPERATION_ON_CMD / OPERATION_OFF_CMD
    static constexpr uint8_t CLEAR_FAULTS_REGISTER      = 0x03;   // send: clears STATUS registers, re-arms blackbox
    static constexpr uint8_t FETCH_BB_EEPROM_REGISTER   = 0xEB;   // send: copy blackbox EEPROM into shadow registers
};

/**
 * @struct HotSwapControlCommands_s
 * @note Data values for OPERATION_REGISTER
*/
struct HotSwapControlCommands_s
{
    static constexpr uint8_t OPERATION_ON_CMD           = 0x80;   // OPERATION_REGISTER data: FET on
    static constexpr uint8_t OPERATION_OFF_CMD          = 0x00;   // OPERATION_REGISTER data: FET off
};

/**
 * @struct HotSwapConfigurationRegisters_s
 * @note Command codes signaling which register we're talking to
*/
struct HotSwapConfigurationRegisters_s
{
    static constexpr uint8_t DEVICE_SETUP1_REGISTER = 0xCC;     // R/W 1: retries (7:5), current limit setting/source
    static constexpr uint8_t ALERT_MASK_REGISTER    = 0xD8;     // R/W 2: which faults/warnings pull SMBA low (1 = masked)
};

/**
 * @struct HotSwapTelemetryRegisters_s
 * @note Command codes signaling which register we're talking to
*/
struct HotSwapTelemetryRegisters_s
{
    static constexpr uint8_t DIAGNOSTIC_WORD_READ_REGISTER  = 0xE1;     // Read; 2 byte  Every fault/warning in one read

    // Instantaneous measurements (12-bit, decode with COEFFS)
    static constexpr uint8_t READ_VIN_REGISTER              = 0x88;     // Read; 2 byte  TelemOptions_e::VIN
    static constexpr uint8_t READ_IIN_REGISTER              = 0x89;     // Read; 2 byte  TelemOptions_e::IIN
    static constexpr uint8_t READ_VOUT_REGISTER             = 0x8B;     // Read; 2 byte  TelemOptions_e::VOUT
    static constexpr uint8_t READ_TEMPERATURE_1_REGISTER    = 0x8D;     // Read; 2 byte  TelemOptions_e::TEMP (external MMBT3904 on DIODE)
    static constexpr uint8_t READ_PIN_REGISTER              = 0x97;     // Read; 2 byte  TelemOptions_e::PIN
    static constexpr uint8_t READ_BB_EEPROM_REGISTER        = 0xF4;     // blk 22 (section 7.5.2.79; Table 7-2 says 16). Send FETCH_BB_EEPROM first
};

/**
 * @note Different telemetry classes available for the hotswap. There are more! (Table 7-1)
 * @warning Indexes COEFFS below, so the order here must match COEFFS row for row
*/
enum class TelemOptions_e : uint8_t
{
    VIN,
    VIN_AVG,
    VOUT,
    VOUT_AVG,
    TEMP,
    TEMP_AVG,
    IIN,
    IIN_AVG,
    IOUT,
    POUT,
    PIN,
    PIN_AVG,
    NUM_TELEM_OPTIONS
};

/**
 * @note
 * - Every telemetry register returns a raw 12-bit count, Y (0–4095).
 *
 * - PMBus "direct format" turns it into a real value X with: X = (Y × 10^(−R) − b) / m
*/
struct PMBusCoefficients_s
{
    float R;
    float b;
    float m;
};

// Add more accordingly if needed (most are already pulled from pages 81-83)
constexpr PMBusCoefficients_s COEFFICIENTS[] =
{
    /* VIN      */ { -2.0f, 255.0f, 4596.0f },
    /* VIN_AVG  */ { -2.0f, 233.0f, 4596.0f },
    /* VOUT     */ { -2.0f, 455.0f, 4596.0f },
    /* VOUT_AVG */ { -2.0f, 417.0f, 4596.0f },
    /* TEMP     */ { -2.0f, 26437.0f, 100.0f },
    /* TEMP_AVG */ { -2.0f, 26437.0f, 100.0f },
    /* IIN      */ { -2.0f, 237.03f, 7583.3f * hotswap_default_params::RSNS_MOHM },
    /* IIN_AVG  */ { -2.0f, 220.65f, 7583.3f * hotswap_default_params::RSNS_MOHM },
    /* IOUT     */ { -2.0f, 220.65f, 7583.3f * hotswap_default_params::RSNS_MOHM },
    /* POUT     */ { -4.0f, 6868.0f, 8511.0f * hotswap_default_params::RSNS_MOHM },
    /* PIN      */ { -4.0f, 6868.0f, 8511.0f * hotswap_default_params::RSNS_MOHM },
    /* PIN_AVG  */ { -4.0f, 6672.0f, 8511.0f * hotswap_default_params::RSNS_MOHM },
};
static_assert(sizeof(COEFFICIENTS) / sizeof(COEFFICIENTS[0]) == static_cast<size_t>(TelemOptions_e::NUM_TELEM_OPTIONS),
              "COEFFS must have exactly one row per Telem entry, in the same order");

/**
 * @struct HotSwapBlackboxEEPROM_s
 * @brief Struct defines what is read from a READ_BB_EEPROM command (datasheet section 7.5.2.79, page 78)
 * @note *_raw fields are undecoded 12-bit ADC counts; decode with COEFFS (VIN, IIN, PIN, TEMP rows)
 * @note
 *      - READ_BB_EEPROM is a manufacturer-specific command used to read contents
 *        stored in the Blackbox shadow registers internal to the device.
 *
 *      - Before issuing this command, the FETCH_BB_EEPROM command needs to be sent
 *        to load the shadow registers with Blackbox contents from the internal EEPROM.
 *
 *      - READ_BB_EEPROM retrieves twenty-two (22) bytes of Blackbox information
 *        stored in the EEPROM.
*/
struct HotSwapBlackboxEEPROM_s
{
    uint8_t ram[7];
    uint8_t timer;
    uint16_t status_word;
    uint8_t status_1;
    uint16_t status_2;
    uint8_t status_input;
    uint16_t voltage_in_peak_raw;
    uint16_t current_in_peak_raw;
    uint16_t power_in_peak_raw;
    uint16_t temp_peak_raw;
};

/**
 * @struct HotSwapConfig_s
 * @param SMBA_PIN Pin of the SMBA alert line (open-drain, active low)
 * @param PGD_PIN Pin of the PGD power-good line (open-drain, high = output up)
*/
struct HotSwapConfig_s
{
    uint32_t SMBA_PIN;
    uint32_t PGD_PIN;

    /// TODO: Not used yet, could be in the future. probably cannot be floats though (Aazam)
    // float slew_rate;
    // float I_limit;
    // float circuit_breaker_threshold;
    // float power_limit;
    // float fault_timer;
    // float UVLO_threshold;
    // float powerGood_threshold;
    // float fault_respose;
};

/**
 * @struct HotSwapData_s
 * @note Defines the data that we are reading from the Hotswap
*/
struct HotSwapData_s
{
    float Vin = 0.0f;
    float Iin = 0.0f;
    float Vout = 0.0f;
    float Pin = 0.0f;
    float temp = 0.0f;
};

struct HotSwapStatus_s
{
    bool is_initialized = false;    ///< Init sequence completed
    bool is_online = false;         ///< False after MAX_CONSEC_ERRORS failed transactions
    uint32_t error_count = 0;       ///< Total failed I2C transactions
    uint8_t consecutive_errors = 0; ///< Failed transactions since the last success
};

/**
 * @struct DiagnosticWordBits_s
 * @note
 *  - Defines the bit structure the DIAGNOSTIC_WORD register.
 *
 *  - Command is useful to get all status data at once instead of doing individual status commands
 *    (STATUS_INPUT, STATUS_MFR_SPECIFIC, etc.)
 *
*/
struct DiagnosticWordBits_s
{
    static constexpr uint8_t VOUT_UV_WARN           = 15;
    static constexpr uint8_t IIN_OP_WARN            = 14; // OP -> Overpower
    static constexpr uint8_t VIN_UV_WARN            = 13;
    static constexpr uint8_t VIN_OV_WARN            = 12;
    static constexpr uint8_t POWER_GOOD_ACTIVELOW   = 11;
    static constexpr uint8_t OVERTEMP_WARN          = 10;
    static constexpr uint8_t TIMER_LATCHED_OFF      = 9;
    static constexpr uint8_t FET_FAIL_WARN          = 8;
    static constexpr uint8_t CONFIG_PRESET          = 7;
    static constexpr uint8_t DEVICE_OFF             = 6;
    static constexpr uint8_t VIN_UV_FAULT           = 5;
    static constexpr uint8_t VIN_OV_FAULT           = 4;
    static constexpr uint8_t IIN_OC_FAULT           = 3; // IIN_OC/PFET_OP_FAULT
    static constexpr uint8_t OVERTEMP_FAULT         = 2;
    static constexpr uint8_t COMMS_FAULT            = 1;
    static constexpr uint8_t CIRCUIT_BREAKER_FAULT  = 0;
};

/**
 * @struct AlertMaskBits_s
 * @note Defines the structure of the ALERT_MASK. Each of these bits could be a reason the
 *       SMBA Alert line is pulled low.
*/
struct AlertMaskBits_s
{
    static constexpr uint8_t VOUT_UV_WARN           = 15;
    static constexpr uint8_t IIN_LIMIT_WARN         = 14;
    static constexpr uint8_t VIN_UV_WARN            = 13;
    static constexpr uint8_t VIN_OV_WARN            = 12;
    static constexpr uint8_t POWER_GOOD_ACIVE_LOW   = 11; // POWER GOOD, active-low (overbar in the datasheet)
    static constexpr uint8_t OVERTEMP_WARN          = 10;
    static constexpr uint8_t WATCHDOG_FAULT         = 9;
    static constexpr uint8_t OVERPOWER_WARN         = 8;  // Overpower limit warn
    static constexpr uint8_t SCP_FAULT              = 7;  // Short circuit protection fault
    static constexpr uint8_t FET_FAIL_FAULT         = 6;  //
    static constexpr uint8_t VIN_UV_FAULT           = 5;
    static constexpr uint8_t VIN_OV_FAULT           = 4;
    static constexpr uint8_t IIN_FAULT              = 3; // Iin / PFet fault
    static constexpr uint8_t OVERTEMP_FAULT         = 2;
    static constexpr uint8_t COMMS_FAULT            = 1;
    static constexpr uint8_t CIRCUIT_BREAKER_FAULT  = 0;

    // Power-on default is 0xFD20: bits 15-10, 8 and 5 masked
    static constexpr uint16_t DEFAULT_MASK = 0xFD20;
};

struct Hotswap_s
{
    uint16_t hal_address = 0;
    uint32_t last_bus_tick = 0;
    HotSwapData_s data;
    HotSwapStatus_s status;
    uint16_t diagnostic_word = 0;
    bool is_power_good = false;
    HotSwapBlackboxEEPROM_s blackbox = {};      // Faults recorded by the chip before this boot; read once in checkPreviousFaults()
    bool is_blackbox_valid = false;             // True if the blackbox read at boot succeeded
    HotSwapConfig_s _config = {};
};

class HotSwapInterface
{
public:

    explicit HotSwapInterface(const HotSwapConfig_s& config)
    {
        hotswap_info.hal_address = hotswap_default_params::ADDRESS;
        hotswap_info._config = config;
    }

    /**
     * @param hi2c HAL I2C handle of the bus the LM5066 is on (e.g. STM32I2CInterface::getHandle())
    */
    HAL_StatusTypeDef initHotswap(I2C_HandleTypeDef *hi2c);


    void checkPreviousFaults();

    HAL_StatusTypeDef set4Retry();

    HAL_StatusTypeDef clearFaults();

    HAL_StatusTypeDef unmaskFaults();

    uint16_t readWord( uint8_t command );

    float readDecodedTelemetry(uint8_t command, TelemOptions_e telem_class);

    HAL_StatusTypeDef readTelemetry(); //Pin, Vin, Vout, Temp, Iin

    HAL_StatusTypeDef shutOff();

    HAL_StatusTypeDef handleAlert();

    /**
     * @brief Periodic work: handleAlert() if the SMBA pin is low, else readTelemetry()
    */
    HAL_StatusTypeDef update();

    /**
     * @brief Reads the blackbox EEPROM using the internal shadow registers
     * @post Data is stored in a HotSwapBlackboxEEPROM_s struct/instance
    */
    HAL_StatusTypeDef readBlackBoxEEPROM(HotSwapBlackboxEEPROM_s &blackbox_register);

    // Accessors
    const HotSwapData_s &getData() const { return hotswap_info.data; }
    const HotSwapStatus_s &getStatus() const { return hotswap_info.status; }
    uint16_t getDiagnosticWord() const { return hotswap_info.diagnostic_word; }
    bool isPowerGood() const { return hotswap_info.is_power_good; }

    /**
     * @return Blackbox contents read at boot (faults the chip recorded before this power-up).
     * @note Only meaningful if isBlackboxValid() is true.
    */
    const HotSwapBlackboxEEPROM_s &getBlackbox() const { return hotswap_info.blackbox; }
    bool isBlackboxValid() const { return hotswap_info.is_blackbox_valid; }

private:

    HAL_StatusTypeDef decode_status = HAL_OK; // Result of the last readWord(), checked after each readDecodedTelemetry()
    Hotswap_s hotswap_info;
    I2C_HandleTypeDef *_hi2c = nullptr;   // Set in initHotswap()

    /**
     * @brief PMBus send byte: sends a command code with no data (e.g. CLEAR_FAULTS, FETCH_BB_EEPROM).
     *        Records the result with _updateSensorStatus().
     * @param desired_command Command code to send
     * @return HAL status of the transfer (HAL_OK, HAL_ERROR, HAL_BUSY or HAL_TIMEOUT)
    */
    HAL_StatusTypeDef _sendCommand(uint8_t desired_command);

    /**
     * @brief PMBus write byte/word: sends a command code followed by data, to set a register.
     *        Records the result with _updateSensorStatus().
     * @param desired_register Command code of the register to write
     * @param buffer Data to write; word registers are sent low byte first
     * @param length Number of data bytes (the "No. of Data Bytes" column of datasheet Table 7-2)
     * @return HAL status of the transfer (HAL_OK, HAL_ERROR, HAL_BUSY or HAL_TIMEOUT)
    */
    HAL_StatusTypeDef _writeRegister(uint8_t desired_register, uint8_t *buffer, uint16_t length);

    /**
     * @brief PMBus read byte/word/block: sends a command code, then reads the register's data back.
     *        Records the result with _updateSensorStatus().
     * @param desired_register Command code of the register to read
     * @param buffer [out] Caller-provided memory the received bytes are written into, low byte first.
     *               This is where the data comes back; the return value only says whether it worked.
     *               Block reads (e.g. READ_BB_EEPROM) start with a count byte.
     * @param length Number of bytes to read; buffer must hold at least this many
     * @return HAL status of the transfer (HAL_OK, HAL_ERROR, HAL_BUSY or HAL_TIMEOUT).
     *         If it isn't HAL_OK, the contents of buffer are not valid.
    */
    HAL_StatusTypeDef _readRegister(uint8_t desired_register, uint8_t *buffer, uint16_t length);

    /**
     * @note If the sensor stats is HAL_OK then reset all errors. Otherwise, we will increment error counter.
     *       If we have too many consectuive errors, then we will turn the hotswap off
    */
    HAL_StatusTypeDef _updateSensorStatus(HAL_StatusTypeDef status);

};
using HotSwapInterfaceInstance = etl::singleton<HotSwapInterface>;

#endif