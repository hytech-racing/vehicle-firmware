#ifndef PDB_CONFIG_H
#define PDB_CONFIG_H

/**
 * @file PDB_Config.hpp
 * @brief MCU-level configuration copied from CubeMX's generated Core/Src/main.c.
 *        Pin definitions (CubeMX's main.h) live in Pins.h.
 * @note After regenerating in CubeMX, copy SystemClock_Config() and MPU_Config() from Core/Src/main.c
 *       into PDB_SystemConfig.cpp, and the pin defines from Core/Inc/main.h into Pins.h.
*/

#include "stm32_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief System clock configuration. Called by the Arduino core before setup().
*/
void SystemClock_Config(void);

/**
 * @brief Memory protection unit configuration. Call at the start of setup().
*/
void MPU_Config(void);

#ifdef __cplusplus
}
#endif

#endif // PDB_SYSTEM_CONFIG_H