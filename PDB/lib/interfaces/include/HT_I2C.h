#ifndef HT_I2C_H
#define HT_I2C_H

#include <stm32h7xx_hal.h>

// Hardware I2C
extern I2C_HandleTypeDef hi2c1;

#ifdef __cplusplus
extern "C" {
#endif

void HT_I2C_Init(void);
// void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);
// void HAL_I2C_Mspnit(I2C_HandleTypeDef* i2cHandle);

#ifdef __cplusplus
}
#endif

#endif // HT_I2C_H