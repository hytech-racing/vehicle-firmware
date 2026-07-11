#ifndef MAIN_H
#define MAIN_H

/**
 * @brief The STM32H750 has one main clock input (a crystal oscillator on the board, called HSE — High Speed External)
 *        and uses PLLs (Phase Locked Loops) to multiply that frequency up to the speeds the chip needs.
 *        The PLL essentially takes in a slow frequency and ouputs a faster one.
 */
extern "C" void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};

    /*
     * CONFIGURATION
     *
     * We use Scale 0 to achieve the maximum 480MHz CPU speed.
     * PWR_LDO_SUPPLY = use the internal LDO voltage regulator.
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0); // Scale 0 = 480MHz

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /**
     * CONFIGURE OSCILLATOR SOURCES
     *
     * HSE  (High Speed External) = 25MHz crystal on the PCB.
     * This is the primary clock source for PLL1, PLL2, PLL3.
     *
     * HSI48 (High Speed Internal 48MHz) = internal RC oscillator.
     * This is used exclusively for USB which requires exactly 48MHz.
     *
     */

    /* Macro to configure the PLL clock source */
    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

    /**
     * CONFIGURE PLL1 (main system clock → 480MHz CPU)
     *
     * VCO (Voltage-Controlled Oscillator) is the core module of the PLL.
     * It multiplies the base clock input into a much higher frequency for the CPU and peripherals
     *
     * PLL math using 25MHz HSE input:
     * Input  = HSE / PLLM    = 25MHz / 5   = 5MHz    (VCO input)
     * VCO    = Input × PLLN  = 5MHz × 192  = 960MHz  (VCO output)
     * SYSCLK = VCO / PLLP    = 960MHz / 2  = 480MHz  → CPU core
     * PLL1Q  = VCO / PLLQ    = 960MHz / 5  = 192MHz  → peripheral clock
     * PLL1R  = VCO / PLLR    = 960MHz / 2  = 480MHz  → other peripherals
     *
     * PLLRGE = VCO/PLL1 clock Input range
     * PLLVCOSEL = VCO/PLL1 clock Output range
     */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 5;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2; // Clock range frequency between 4 and 8 MHz
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE; // VCO output 192-836MHz range
    RCC_OscInitStruct.PLL.PLLFRACN = 0; // no fractional divider

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**
     * CONFIGURE CPU AND BUS CLOCK DIVIDERS
     *
     * The STM32H750 has a hierarchy of internal buses running at progressively slower speeds.
     * Peripherals hang off the slower APB buses
     *
     * SYSCLK  = 480MHz  (from PLL1P above)
     * HCLK    = 240MHz  (SYSCLK / AHBCLKDivider = 2)  -> AHB bus, memory, DMA
     * APB1    = 120MHz  (HCLK / APB1CLKDivider = 2)   -> timers, I2C, UART, SPI
     * APB2    = 120MHz  (HCLK / APB2CLKDivider = 2)   -> SPI1, USART1, ADC
     * APB3    = 120MHz  (HCLK / APB3CLKDivider = 2)   -> LTDC/LCD display controller
     * APB4    = 120MHz  (HCLK / APB4CLKDivider = 2)   -> SPI6, I2C4, LPTIM
     *
     * Flash latency = 4 wait states required at 240MHz AHB with VOS0.
     * (The CPU is faster than flash so wait states let flash catch up.)
     *
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  |
                                RCC_CLOCKTYPE_SYSCLK  |
                                RCC_CLOCKTYPE_PCLK1   |
                                RCC_CLOCKTYPE_PCLK2   |
                                RCC_CLOCKTYPE_D3PCLK1 |
                                RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // PLL1 selection as system clock
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();
    }

    /**
     * CONFIGURE PERIPHERAL CLOCKS (PLL2, PLL3, and others)
     *
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART16  |
                                            RCC_PERIPHCLK_RNG         |
                                            RCC_PERIPHCLK_SPI123      |
                                            RCC_PERIPHCLK_SAI2        |
                                            RCC_PERIPHCLK_SAI1        |
                                            RCC_PERIPHCLK_SDMMC       |
                                            RCC_PERIPHCLK_USART234578 |
                                            RCC_PERIPHCLK_ADC         |
                                            RCC_PERIPHCLK_I2C123      |
                                            RCC_PERIPHCLK_USB         |
                                            RCC_PERIPHCLK_QSPI        |
                                            RCC_PERIPHCLK_FMC         |
                                            RCC_PERIPHCLK_SPI45       |
                                            RCC_PERIPHCLK_SPI6        |
                                            RCC_PERIPHCLK_FDCAN;


    /**
     * PLL2. (FMC (flexible memory controller), FDCAN, SPI)
     *
     * PLL2 math using 25MHz HSE input:
     * Input  = HSE / PLL2M   = 25MHz / 2      = 12.5MHz
     * VCO    = Input × PLL2N = 12.5MHz × 12   = 150MHz
     * PLL2P  = VCO / PLL2P   = 150MHz / 2     = 75MHz → SPI
     * PLL2Q  = VCO / PLL2Q   = 150MHz / 3     = 50MHz → FDCAN
     * PLL2R  = VCO / PLL2R   = 150MHz / 2     = 75MHz → FMC
     *
     */
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    //  PeriphClkInitStruct.PLL2.PLL2N = 115; // Max Freq @ 3v3 (overclocked SDRAM)
    PeriphClkInitStruct.PLL2.PLL2N = 12;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 3;  // FDCAN
    PeriphClkInitStruct.PLL2.PLL2R = 2;  // FMC
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2; // Clock range frequency between 4 and 8 MHz
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;

    /**
     * PLL3. (Clocks ADC, I2C, SAI (audio), UART)
     *
     * PLL3 math using 25MHz HSE input:
     * Input  = HSE / PLL3M   = 25MHz / 6        = 4.1667MHz
     * VCO    = Input × PLL3N = 4.1667MHz × 295  = 1229.1667MHz
     * PLL3P  = VCO / PLL3P   = 1229.1667MHz / 64   = 19.2057Hz → SAI
     * PLL3Q  = VCO / PLL3Q   = 1229.1667MHz / 4    = 307.2917MHz  → I2C, SPI
     * PLL3R  = VCO / PLL3R   = 1229.1667MHz / 32   = 38.4115MHz  → ADC
     *
     */
    PeriphClkInitStruct.PLL3.PLL3M = 6;
    PeriphClkInitStruct.PLL3.PLL3N = 295;
    PeriphClkInitStruct.PLL3.PLL3P = 64;
    PeriphClkInitStruct.PLL3.PLL3Q = 4;
    PeriphClkInitStruct.PLL3.PLL3R = 32;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;

    /**
     * INDIVIDUAL PERIPHERAL CLOCK SOURCE ASSIGNMENTS
     * Each peripheral can independently select its clock source.
     * Most use PLL2 or PLL3 outputs configured above.
     *
     */
    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_PLL2;
    PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
    PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
    PeriphClkInitStruct.Sai23ClockSelection = RCC_SAI23CLKSOURCE_PLL3;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PLL3;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /* Enable USB Voltage detector */
    HAL_PWREx_EnableUSBVoltageDetector();
}

#endif