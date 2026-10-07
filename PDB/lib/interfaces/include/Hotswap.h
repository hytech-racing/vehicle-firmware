// Aazam To-Do for HOTSWAP:
// Add accessor methods
// Compute hardware fields from the scchematics provided + datasheet
// Figure out exactly what you want to do with the EEPROM blackbox when initialized (aka faults in previous run)
// In initialization, add the slew rate requirements and other hardware consideration (Rachel)

#ifndef HOTSWAP_H
#define HOTSWAP_H

#include <cstdint>
#include <stm32h7xx_hal.h>
#include <Arduino.h>

namespace hotswap_default_params
{
    static constexpr uint8_t ADDR = 0x15 << 1;                    // shifted to accomodate R/W! bit that hal automatically changes
    static constexpr uint32_t MIN_READ_INTERVAL_MS = 100;         // conversion ~60 ms; any bus access restarts it
    static constexpr uint32_t I2C_TIMEOUT_MS       = 10;
    static constexpr uint8_t MAX_CONSEC_ERRORS    = 100;          // failed transactions before hotswap is offline (check with Anthony)
    
    static constexpr uint8_t NUM_RETRIES         = 4;
    static constexpr uint8_t RETRY_MASK          = 0xE0;
    static constexpr uint8_t RETRY_4             = 0x80;
}

namespace hotswap_commands {
    static constexpr uint8_t CMD_OPERATION       = 0x01;
    static constexpr uint8_t CMD_CLEAR_FAULTS    = 0x03;

    static constexpr uint8_t DEVICE_SETUP1       = 0xCC;

    static constexpr uint8_t CMD_DIAGNOSTIC_WORD = 0xE1;

    static constexpr uint8_t CMD_UNMASK_FAULTS   = 0xD9;
    static constexpr uint8_t CMD_OPERATION_OFF   = 0x00;

    static constexpr uint8_t CMD_FETCH_BB_EEPROM = 0xEB; //copies the eeprom into shadow registers
    static constexpr uint8_t CMD_READ_BB_EEPROM  = 0xF4; //reads the eeprom
}

//Different telemetry classes available for the hotswap
//Note: VIN has VIN_min, VIN_peak, VIN_UV, more specifics in datasheet Table 7-1 (page 81)
enum class Telem : uint8_t 
{ 
    VIN, 
    VIN_AVG,
    VOUT, 
    VOUT_AVG, 
    TEMP, 
    IIN,
    IIN_AVG,
    IOUT,
    POUT,
    PIN,
    PIN_AVG
};

constexpr float RSNS_MOHM = 2.0f;

struct PmbusCoeff_s 
{ 
    float R; 
    float b; 
    float m; 
};

//Add more accordingly if needed (most are already pulled from P81-83)
constexpr PmbusCoeff_s COEFFS[] = 
{
    /* VIN      */ { -2.0f,   255.0f,  4596.0f },
    /* VIN_AVG  */ { -2.0f,   233.0f,  4596.0f },
    /* VOUT     */ { -2.0f,   455.0f,  4596.0f },
    /* VOUT_AVG */ { -2.0f,   417.0f,  4596.0f },
    /* TEMP     */ { -2.0f, 26437.0f,   100.0f },
    /* TEMP_AVG */ { -2.0f, 26437.0f,   100.0f },
    /* IIN      */ { -2.0f,   237.03f, 7583.3f * RSNS_MOHM },
    /* IIN_AVG  */ { -2.0f,   220.65f, 7583.3f * RSNS_MOHM },
    /* IOUT     */ { -2.0f,   220.65f, 7583.3f * RSNS_MOHM },
    /* POUT     */ { -4.0f,  6868.0f,  8511.0f * RSNS_MOHM },
    /* PIN      */ { -4.0f,  6868.0f,  8511.0f * RSNS_MOHM },
    /* PIN_AVG  */ { -4.0f,  6672.0f,  8511.0f * RSNS_MOHM },
};
    
namespace hotswap_telemetry_requests {
    // Instantaneous readings (read word, 12-bit)
    static constexpr uint8_t CMD_READ_VIN        = 0x88; // Telem::VIN
    static constexpr uint8_t CMD_READ_IIN        = 0x89; // Telem::IIN 
    static constexpr uint8_t CMD_READ_VOUT       = 0x8B; // Telem::VOUT
    static constexpr uint8_t CMD_READ_IOUT       = 0x8C; // Telem::IOUT
    static constexpr uint8_t CMD_READ_TEMP       = 0x8D; // Telem::TEMP
    static constexpr uint8_t CMD_READ_POUT       = 0x96; // Telem::POUT
    static constexpr uint8_t CMD_READ_PIN        = 0x97; // Telem::PIN

    // Min / peak since last clear (read word)
    static constexpr uint8_t CMD_READ_VIN_MIN    = 0xA0; // Telem::VIN
    static constexpr uint8_t CMD_READ_VIN_PEAK   = 0xA1; // Telem::VIN
    static constexpr uint8_t CMD_READ_IIN_PEAK   = 0xA2; // Telem::IIN
    static constexpr uint8_t CMD_READ_PIN_PEAK   = 0xA3; // Telem::PIN
    static constexpr uint8_t CMD_READ_VOUT_MIN   = 0xA4; // Telem::VOUT
    static constexpr uint8_t CMD_READ_TEMP_PEAK  = 0xC8; // Telem::TEMP

    // Averaged readings (read word; averaging length set by SAMPLES_FOR_AVG, DBh)
    static constexpr uint8_t CMD_READ_VIN_AVG    = 0xDC; // Telem::VIN_AVG
    static constexpr uint8_t CMD_READ_VOUT_AVG   = 0xDD; // Telem::VOUT_AVG
    static constexpr uint8_t CMD_READ_IIN_AVG    = 0xDE; // Telem::IIN_AVG
    static constexpr uint8_t CMD_READ_PIN_AVG    = 0xDF; // Telem::PIN_AVG
    static constexpr uint8_t CMD_READ_TEMP_AVG   = 0xC7; // Telem::TEMP_AVG
}

struct BlackboxRecord {
    uint8_t  ram[7];
    uint8_t  timer;
    uint16_t statusWord;
    uint8_t  statusMfr;
    uint16_t statusMfr2;
    uint8_t  statusInput;
    uint16_t vinPeakRaw, iinPeakRaw, pinPeakRaw, tempPeakRaw;
};

//Main hardware fields
struct Config_s {
    uint32_t  SMBA_PIN;
    uint32_t  PGD_PIN;
    //These need proper formulas, calculation and software checks
    float slew_rate;
    float I_limit;
    float Circuit_breaker_threshold;
    float Power_limit;
    float Fault_timer;
    float UVLO_threshold;
    float PowerGood_threshold;
    float FaultResponse;
};

struct HotswapData_s {
    float _Vin = 0.0f;
    float _Iin = 0.0f;
    float _Vout = 0.0f;
    float _Pin = 0.0f;
    float _Temp = 0.0f;
};

struct HotswapStatus_s
{
    bool is_initialized;     ///< Init sequence completed
    bool is_online;          ///< False after MAX_CONSEC_ERRORS failed transactions
    uint16_t _diagnostic_word; ///< reads when interrupt occurs to diagnose hotswap
    uint32_t error_count;        ///< Total failed I2C transactions
    uint8_t consecutive_errors; ///< Failed transactions since the last success
};

struct DiagnosticWordBits
{
    //Note: Not a one to one mapping unfortunately to Alerts (Warns/Faults)
    static constexpr uint8_t VOUT_UV_WARN         = 15;
    static constexpr uint8_t IIN_OP_WARN          = 14;
    static constexpr uint8_t VIN_UV_WARN          = 13;
    static constexpr uint8_t VIN_OV_WARN          = 12;
    static constexpr uint8_t POWER_GOOD           = 11;
    static constexpr uint8_t OVERTEMP_WARN        = 10;
    static constexpr uint8_t TIMER_LATCHED_OFF    = 9;
    static constexpr uint8_t FET_FAIL_WARN        = 8;
    static constexpr uint8_t CONFIG_PRESET        = 7;
    static constexpr uint8_t DEVICE_OFF           = 6;
    static constexpr uint8_t VIN_UV_FAULT         = 5;
    static constexpr uint8_t VIN_OV_FAULT         = 4;
    static constexpr uint8_t IIN_OC_PFET_OP_FAULT = 3;
    static constexpr uint8_t OVERTEMP_FAULT       = 2;
    static constexpr uint8_t CML_FAULT            = 1;
    static constexpr uint8_t CIRCUIT_BREAKER_FAULT = 0;
};

struct AlertMaskBits
{
    //All are unmasked, may unmask some if line is super noisy (LEADs Note)
    static constexpr uint8_t VOUT_UV_WARN      = 15;
    static constexpr uint8_t IIN_LIMIT_WARN    = 14;
    static constexpr uint8_t VIN_UV_WARN       = 13;
    static constexpr uint8_t VIN_OV_WARN       = 12;
    static constexpr uint8_t NPG               = 11; // POWER GOOD, active-low (overbar in the datasheet)
    static constexpr uint8_t OVERTEMP_WARN     = 10;
    static constexpr uint8_t WATCHDOG_FAULT    = 9;
    static constexpr uint8_t OVERPOWER_WARN    = 8;  // overpower limit warn
    static constexpr uint8_t SCP_FAULT         = 7;
    static constexpr uint8_t FET_FAIL_FAULT    = 6;
    static constexpr uint8_t VIN_UV_FAULT      = 5;
    static constexpr uint8_t VIN_OV_FAULT      = 4;
    static constexpr uint8_t IIN_PFET_FAULT    = 3;
    static constexpr uint8_t OVERTEMP_FAULT    = 2;
    static constexpr uint8_t CML_FAULT         = 1;  // communications fault
    static constexpr uint8_t CIRCUIT_BREAKER_FAULT = 0;

    // Power-on default is 0xFD20: bits 15-10, 8 and 5 masked
    static constexpr uint16_t DEFAULT_MASK = 0xFD20;
};



class Hotswap {
public:
    explicit Hotswap(const Config_s& config) : _config(config) {}
    

    HAL_StatusTypeDef initHotswap();
    void checkPreviousFaults();
    HAL_StatusTypeDef set4Retry();
    HAL_StatusTypeDef clearFaults();
    HAL_StatusTypeDef unmaskFaults();

    uint16_t readWord( uint8_t command );
    float readDecodedTelemetry(uint8_t command, Telem telem_class);
    HAL_StatusTypeDef readTelemetry(); //Pin, Vin, Vout, Temp, Iin
    
    HAL_StatusTypeDef shutOff();

    void smbaIrqHandler();
    // void pgdIrqHandler();
    HAL_StatusTypeDef handleAlert();

    HAL_StatusTypeDef readBlackBoxEEPROM(BlackboxRecord &r);
    
    Config_s _config;

private:
    struct Hotswap_s
    {
        uint16_t hal_address;
        //uint8_t address_pointer;
        uint32_t last_bus_tick;
        HotswapData_s data;
        HotswapStatus_s status;
        uint16_t _fault_word = 0;
        volatile bool _powerGood;
        Config_s _config;
        volatile bool _alert_pending;
    };

    I2C_HandleTypeDef *_hi2c;

    HAL_StatusTypeDef sendCommand(uint8_t desired_command);
    HAL_StatusTypeDef writeRegister(uint8_t desired_register, uint8_t *buffer, uint16_t length);
    HAL_StatusTypeDef readRegister(uint8_t desired_register, uint8_t *buffer, uint16_t length);
    HAL_StatusTypeDef _updateSensorStatus(HAL_StatusTypeDef status);

    HAL_StatusTypeDef decode_status; //refreshed in every decode instruction, placeholder more than anything else

    Hotswap_s hotswap_info;
};

extern Hotswap HS5066;

#endif