#include <Arduino.h>

extern "C" {
#include "clock_config.h"
#include "peripherals.h"
}

void setup()
{
    /* NOTE: STM32duino runs its own SystemClock_Config() before setup().
       If this board needs the custom clock tree instead, that override has to
       be wired in at the framework level (see clock_config.h — which is NOT
       yet valid; clocks need to be regenerated for the 16 MHz HSE first).
       For now the peripheral inits below assume a valid clock is already up. */

    // TODO(clocks): confirm clock strategy — either rely on STM32duino's default
    // for genericSTM32H750VB, or override with PDB_SystemClock_Config() once fixed.

    PDB_GPIO_Init();
    PDB_ADC1_Init();
    PDB_FDCAN1_Init();
    PDB_I2C1_SMBUS_Init();
    PDB_USB_OTG_FS_PCD_Init();

    // TODO: HAL_FDCAN_ConfigGlobalFilter() — accept-all into FIFO0 until the
    // real CAN ID list exists, then HAL_FDCAN_Start().
}

void loop()
{
    
}