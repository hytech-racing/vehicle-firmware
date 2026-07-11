#ifndef STM32_CAN_H
#define STM32_CAN_H

#define FDCAN_TypeDef FDCAN_GlobalTypeDef

/* Standard Library */
#include <stdint.h>

/* External Includes */
#include <stm32h7xx_hal.h>
#include <stm32h750xx.h>
#include <stm32h7xx_hal_fdcan.h>


typedef struct
{
    uint32_t id;
    uint8_t extended;
    uint8_t dlc;
    uint8_t data[64];
    uint8_t* buf;
    uint8_t len;
} CAN_message_t;

#endif
