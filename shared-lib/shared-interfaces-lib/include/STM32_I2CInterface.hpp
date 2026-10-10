#ifndef STM32_I2C_INTERFACE_H
#define STM32_I2C_INTERFACE_H

/**
 * @file STM32_I2CInterface.h
 * @brief Generic, per-board-configurable I2C master interface for STM32H7 (HAL-based).
 *
 *  Design goals:
 *    - One reusable driver; each board passes its own config (which I2C peripheral, pins, kernel clock,
 *      timing register).
 *    - Device drivers (temp sensors, hotswap, ...) take getHandle() and call the HAL I2C functions on it.
 *    - Supports several buses at once (one object per bus).
 *
 * @note Clocks and pins are set up inside init(), not in HAL_I2C_MspInit(), so this driver defines no
 *       global HAL hooks and can coexist with any board code.
*/

#if defined(ARDUINO_ARCH_STM32)

/* Standard Library */
#include <stdint.h>

/* External Includes */
#include <Arduino.h>
#include <stm32h7xx_hal.h>
#include <etl/singleton.h>


/**
 * @brief Everything a board specifies to configure one I2C bus
 * @note timing is the raw TIMINGR value. Generate it with STM32CubeMX (or ST's I2C timing tool) for the
 *       chosen kernel clock and bus speed; it is only valid for that kernel clock.
*/
struct STM32I2CConfig_s
{
    // Which peripheral
    I2C_TypeDef* instance = nullptr;            // I2C1, I2C2, I2C3, I2C4

    // Pins (Arduino pin names, e.g. PB6 / PB7); alternate function comes from the core's PinMap_I2C tables
    uint32_t scl_pin = NC;
    uint32_t sda_pin = NC;

    // Kernel clock feeding the peripheral
    // I2C1-3: RCC_I2C123CLKSOURCE_D2PCLK1 / _PLL3 / _HSI / _CSI
    // I2C4:   RCC_I2C4CLKSOURCE_D3PCLK1 / _PLL3 / _HSI / _CSI
    uint32_t kernel_clock_source = RCC_I2C123CLKSOURCE_D2PCLK1;

    uint32_t timing = 0;                        // TIMINGR, from CubeMX for this kernel clock + bus speed

    bool analog_filter = true;                  // Suppresses spikes < 50 ns on SCL/SDA
    uint8_t digital_filter = 0;                 // 0 = off, 1-15 = filter spikes up to N kernel clock periods
};


class STM32I2CInterface
{
public:

    STM32I2CInterface() = default;

    STM32I2CInterface(const STM32I2CInterface &)            = delete;
    STM32I2CInterface &operator=(const STM32I2CInterface &) = delete;

    /**
     * @brief Configures clocks, pins, timing and filters, then initializes the bus as a 7-bit master
     * @return True if configured, false on a bad config or a HAL error
    */
    bool init(const STM32I2CConfig_s &config);

    /**
     * @return HAL handle for this bus, for HAL_I2C_* calls in device drivers
    */
    I2C_HandleTypeDef *getHandle() { return &_hi2c; }

    /**
     * @return True once init() has succeeded
    */
    bool isInitialized() const { return _is_initialized; }

private:

    /**
     * @brief Selects the kernel clock and enables the peripheral clock for this instance
    */
    bool _enableClocks();

    STM32I2CConfig_s _config;
    I2C_HandleTypeDef _hi2c = {};
    bool _is_initialized = false;
};

using STM32I2CInterfaceInstance = etl::singleton<STM32I2CInterface>;

#endif // ARDUINO_ARCH_STM32

#endif // STM32_I2C_INTERFACE_H
