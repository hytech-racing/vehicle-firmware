#include "STM32_I2CInterface.hpp"

#if defined(ARDUINO_ARCH_STM32)


bool STM32I2CInterface::init(const STM32I2CConfig_s &config)
{
    _config = config;
    _is_initialized = false;

    if (_config.instance == nullptr || _config.scl_pin == NC || _config.sda_pin == NC)
    {
        return false;
    }

    if (!_enableClocks())
    {
        return false;
    }

    // Pins: the core's PinMap_I2C tables supply the open-drain alternate function, so no AF numbers per board
    pinmap_pinout(digitalPinToPinName(_config.scl_pin), PinMap_I2C_SCL);
    pinmap_pinout(digitalPinToPinName(_config.sda_pin), PinMap_I2C_SDA);

    _hi2c.Instance = _config.instance;
    _hi2c.Init.Timing = _config.timing;
    _hi2c.Init.OwnAddress1 = 0;
    _hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _hi2c.Init.OwnAddress2 = 0;
    _hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    _hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&_hi2c) != HAL_OK)
    {
        return false;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&_hi2c, _config.analog_filter ? I2C_ANALOGFILTER_ENABLE : I2C_ANALOGFILTER_DISABLE) != HAL_OK)
    {
        return false;
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&_hi2c, _config.digital_filter) != HAL_OK)
    {
        return false;
    }

    _is_initialized = true;
    return true;
}

bool STM32I2CInterface::_enableClocks()
{
    RCC_PeriphCLKInitTypeDef clock_init = {};

    if (_config.instance == I2C4)
    {
        clock_init.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
        clock_init.I2c4ClockSelection = _config.kernel_clock_source;
    }
    else
    {
        clock_init.PeriphClockSelection = RCC_PERIPHCLK_I2C123;   // I2C1-3 share one kernel clock mux
        clock_init.I2c123ClockSelection = _config.kernel_clock_source;
    }
    if (HAL_RCCEx_PeriphCLKConfig(&clock_init) != HAL_OK)
    {
        return false;
    }

    if (_config.instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_ENABLE();
    }
    else if (_config.instance == I2C2)
    {
        __HAL_RCC_I2C2_CLK_ENABLE();
    }
    else if (_config.instance == I2C3)
    {
        __HAL_RCC_I2C3_CLK_ENABLE();
    }
    else if (_config.instance == I2C4)
    {
        __HAL_RCC_I2C4_CLK_ENABLE();
    }
    else
    {
        return false;
    }
    return true;
}

#endif // ARDUINO_ARCH_STM32
