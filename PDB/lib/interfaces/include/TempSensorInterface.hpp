#ifndef TEMPSENSOR_INTERFACE_H
#define TEMPSENSOR_INTERFACE_H

#include <stdint.h>
#include <math.h>
#include <stm32h7xx_hal.h>
#include <stm32h750xx.h>
#include <etl/singleton.h>
#include "HT_I2C.h"
#include "hytech.h"


namespace temp_sensor_default_params
{
    constexpr uint8_t NUM_SENSORS = 8;
    constexpr uint8_t MIN_ADDR = 0x48;                      // 7-bit 1001 000: A2 A1 A0 = 000
    constexpr uint8_t MAX_ADDR = 0x4F;                      // 7-bit 1001 111: A2 A1 A0 = 111
    constexpr float HYSTERESIS_SETPOINT = 75.0f;
    constexpr float OVERTEMP_SHUTDOWN_SETPOINT = 80.0f;
    constexpr float TEMP_MIN_C = -55.0f;                    // guaranteed measurement range (datasheet p. 11)
    constexpr float TEMP_MAX_C = 125.0f;
    constexpr uint32_t MIN_READ_INTERVAL_MS = 100;          // conversion ~60 ms; any bus access restarts it
    constexpr uint32_t I2C_TIMEOUT_MS       = 10;
    constexpr uint8_t  MAX_CONSEC_ERRORS    = 3;            // failed transactions before a sensor is offline
    constexpr uint8_t  CONFIG_NORMAL = 0x00;                // continuous conversion (chip power-on default)
    constexpr uint8_t POINTER_UNKNOWN = 0xFF;

    static_assert(NUM_SENSORS <= (MAX_ADDR - MIN_ADDR + 1), "ADT75 has only 8 addresses per bus");
    static_assert(HYSTERESIS_SETPOINT <= OVERTEMP_SHUTDOWN_SETPOINT,
                  "Hysteresis setpoint must not be above the overtemp setpoint");
    static_assert(HYSTERESIS_SETPOINT >= TEMP_MIN_C && OVERTEMP_SHUTDOWN_SETPOINT <= TEMP_MAX_C,
                  "Setpoints must be within the guaranteed -55..125 C range");
}

enum TempSensorIDs_e : uint8_t
{
    BUCK_LIDAR  = 0,  // 0x48, A2:A0 = 000
    BUCK_ORIN   = 1,  // 0x49, A2:A0 = 001
    BUCK_12V    = 2,  // 0x4A, A2:A0 = 010
    BUCK_DTI    = 3,  // 0x4B, A2:A0 = 011
    BUCK_5V     = 4,  // 0x4C, A2:A0 = 100
    BUCK_3V3    = 5,  // 0x4D, A2:A0 = 101
    MCU         = 6,  // 0x4E, A2:A0 = 110
    HOTSWAP     = 7,  // 0x4F, A2:A0 = 111
    NUM_TEMP_SENSOR_IDS
};

/**
 * @brief Register addresses written to the address pointer register (datasheet Table 7, p. 13)
*/
struct TempSensorRegisterAddresses_s
{
    static constexpr uint8_t TEMP_VALUE      = 0x00; // R,   16-bit: measured temperature
    static constexpr uint8_t CONFIG          = 0x01; // R/W,  8-bit: operating modes
    static constexpr uint8_t T_HYST_SETPOINT = 0x02; // R/W, 16-bit: OS pin release point (unused when polling)
    static constexpr uint8_t T_OVER_SETPOINT = 0x03; // R/W, 16-bit: OS pin trip point (unused when polling)
    static constexpr uint8_t ONE_SHOT        = 0x04; // R/W, 16-bit: one-shot trigger (unused)
};

/**
 * @brief Measurement for one sensor
 * @param hal_address is the 7-bit I2C address: 1001 + A2 A1 A0 pins
 * @param temp_degC is the most recent successful read, NAN until the first one
 * @param raw_temp is the raw register contents, D15:D4 valid, D3:D0 always 0
 * @param last_update_ms is the return value of HAL_GetTick(), which returns
 *                       number of milliseconds elapsed since the program started
*/
struct TempSensorData_s
{
    uint8_t hal_address;
    float temp_degC;
    uint16_t raw_temp;
    uint32_t last_update_tick;
};

/**
 * @brief Health flags for one sensor
 * @param is_initialized true when init sequenece completes, false otherwise
 * @param is_online false after MAX_CONSEC_ERRORS failed transactions, true otherwise
 * @param error_count is the number of failed I2C transactions
 *                    Failed Transaction: HAL I2C calls that did not return HAL_OK (NACK, timeout, bus error, etc.)
 * @param consecutive_errors
*/
struct TempSensorStatus_s
{
    bool is_initialized;     ///< Init sequence completed
    bool is_online;          ///< False after MAX_CONSEC_ERRORS failed transactions
    bool is_overtemp;        ///< Software comparator: set at t_os_sp, cleared below t_hyst_sp
    bool is_out_of_range;    ///< Last reading outside the guaranteed -55..125 C
    uint32_t error_count;        ///< Total failed I2C transactions
    uint8_t consecutive_errors; ///< Failed transactions since the last success
};

class TempSensorInterface
{
public:

    /**
     * @brief Registers all temp sensors. Does not touch the bus.
     * @param hi2c HAL I2C handle for the bus the sensors are on
    */
    TempSensorInterface(I2C_HandleTypeDef *hi2c) : _hi2c(hi2c), _all_temp_sensors{}
    {
        for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
        {
            TempSensor_s &sensor = _all_temp_sensors[i];
            sensor.hal_address = static_cast<uint16_t>(temp_sensor_default_params::MIN_ADDR + i) << 1;
            sensor.address_pointer = temp_sensor_default_params::POINTER_UNKNOWN;
            sensor.data.hal_address = static_cast<uint8_t>(temp_sensor_default_params::MIN_ADDR + i);
            sensor.data.temp_degC = NAN; // no reading yet
        }
    }

    TempSensorInterface(const TempSensorInterface &)            = delete;
    TempSensorInterface &operator=(const TempSensorInterface &) = delete;

    /**
     * @brief Method loops through and intializes all 8 sensors
     * @note Tries every sensor even if an earlier one fails
     * @return HAL_OK if all sensors initalize, HAL_ERROR if any fails
    */
    HAL_StatusTypeDef initAllSensors();

    /**
     * @brief Initializes one sensor
     * @note Sends two transactions:
     *    1. CONFIG = CONFIG_NORMAL
     *    2. Address register pointer -> TEMP_VALUE
     * @note The sensor responds to its A2:A0 address from the first transaction; the address is
     *       latched (pin changes ignored from then on) after the second (datasheet p. 16).
     * @param index sensor index
     * @return HAL_OK: Both transactions succeeded.
     *         HAL_ERROR: Index is invalid, I2C handle is null, or the sensor did not respond.
    */
    HAL_StatusTypeDef initSensor(uint8_t index);

    /**
     * @brief Method loops through and reads all 8 sensors temp register
     * @return HAL_OK if all sensors can be read, HAL_ERROR if any fails
    */
    HAL_StatusTypeDef readAllSensorsTemp();


    /// HAL_OK = new reading stored. HAL_BUSY = no new reading this call (too soon,
    /// or sensor was just (re)initialized); not an error. HAL_ERROR = bus failure.
    HAL_StatusTypeDef readSensorTemp(uint8_t index);

    // Accessors. Pointer getters return nullptr for an invalid index.
    const TempSensorData_s *getData(uint8_t index) const;
    const TempSensorStatus_s *getStatus(uint8_t index) const;
    float getTemp(uint8_t index) const;                     ///< NAN if invalid index or no reading yet
    bool  isOvertemp(uint8_t index) const;
    bool  isOnline(uint8_t index) const;
    bool  isFresh(uint8_t index, uint32_t max_age_ms) const; ///< Has a reading newer than max_age_ms
    bool  anyOvertemp() const;
    bool  allOnline() const;
    float getMaxTemp() const;

    float getHysteresisSetpoint() const { return temp_sensor_default_params::HYSTERESIS_SETPOINT; }
    float getOvertempSetpoint() const { return temp_sensor_default_params::OVERTEMP_SHUTDOWN_SETPOINT; }

    /// Register bytes -> deg C (12-bit two's complement, 0.0625 C/LSB)
    static float decodeTemp(uint8_t msb, uint8_t lsb);

private:

    /**
     * @brief Represents one sensor
     * @param hal_address is the 7-bit I2C address shifted left by 1, as the HAL expects (e.g. 0x48 -> 0x90)
     * @param address_pointer is a copy of the sensor's address pointer register
     * @param last_bus_tick is the HAL_GetTick() value of the last transaction with this sensor, successful or not.
     * @param data is the most recent measurement, see TempSensorData_s
     * @param status is the health flags and error counters, see TempSensorStatus_s
    */
    struct TempSensor_s
    {
        uint16_t hal_address;
        uint8_t address_pointer;
        uint32_t last_bus_tick;
        TempSensorData_s data;
        TempSensorStatus_s status;
    };

    I2C_HandleTypeDef *_hi2c;
    TempSensor_s _all_temp_sensors[temp_sensor_default_params::NUM_SENSORS];

    bool _isValidIndex(uint8_t index) const;

    /**
     * @brief Metthod selects which register the sensor's next read will return
     * @note The ADT75's address pointer register decides which register a read returns.
     *       It holds its value until it is written again, so this method only sends the pointer byte
     *       when sensor.address_pointer differs from desired_register.
     * @param sensor is a reference to a sensor
     * @param desired_register is the register address 0x00-0x04, see TempSensorRegisterAddresses_s
     * @return HAL_OK if the pointer is set (or already was), otherwise the failed HAL status.
     *         On failure the sensors pointer is set to temp_sensor_default_params::POINTER_UNKNOWN so the next call resends it.
     */
    HAL_StatusTypeDef setAddressPointerRegister(TempSensor_s &sensor, uint8_t desired_register);

    /**
     *
    */
    HAL_StatusTypeDef readRegister(TempSensor_s &sensor, uint8_t desired_register, uint8_t *buffer, uint16_t length);

    /**
     * @brief Writes data to one of the sensor's registers
     * @note Sends the register address (into the address pointer register) followed by the data, in a single transaction via HAL_I2C_Mem_Write
     * @param sensor Sensor to write to
     * @param desired_register CONFIG, T_HYST_SETPOINT or T_OVER_SETPOINT (TEMP_VALUE is read-only)
     * @param buffer Data to write, MSB first for 16-bit registers
     * @param length 1 for CONFIG (8-bit), 2 for T_HYST/T_OS (16-bit)
     * @return HAL status of the write, also recorded by _updateSensorStatus()
    */
    HAL_StatusTypeDef writeRegister(TempSensor_s &sensor, uint8_t desired_register, uint8_t *buffer, uint16_t length);

    /**
     * @brief Records the result of one HAL I2C call for a sensor and updates its status
     * @param sensor Sensor the transaction was sent to
     * @param status Return value of the HAL I2C call
     * @return status unchanged
    */
    HAL_StatusTypeDef _updateSensorStatus(TempSensor_s &sensor, HAL_StatusTypeDef status);

    /**
     * @brief Check if we have an overtemp based on the overtemp shutdown setpoint
    */
    void updateOvertemp(TempSensor_s &sensor);
};

using TempSensorInterfaceInstance = etl::singleton<TempSensorInterface>;

#endif // TEMPSENSOR_INTERFACE_H