#ifndef HT_I2C_H
#define HT_I2C_H

// Hardware I2C
extern I2C_HandleTypeDef hi2c1;

void HT_I2C_Init(void);

extern "C" {
    void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);
}

#endif // HT_I2C_H