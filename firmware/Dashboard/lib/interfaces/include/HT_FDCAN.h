#ifndef HT_FDCAN_WRAPPER
#define HT_FDCAN_WRAPPER

/* External Includes */
#include <stm32h7xx_hal.h>
#include <stm32h750xx.h>
#include "hytech.h"

/* Local Interface Includes */
#include "ACUInterface.h"
#include "DashCANInterfaceImpl.h"
#include "VCFInterface.h"
#include "VCRInterface.h"


/**
 * @brief This method configures the CAN hardware. So it...
 *        - Enables the FDCAN clock
 *        - Sets up the GPIO pins PD0 (RX) and PD1 (TX) as CAN pins
 *        - Configures timing. NominalPrescaler, NominalTimeSeg1, NominalTimeSeg2 set the baud rate
 *        - Sets up a filter that accepts ALL CAN message IDs (FilterID1 = 0, FilterID2 = 0 means no filtering)
 *        - Starts the peripheral and enables the receive interrupt
 */
int FDCAN_init();

/**
 * @brief This method sends a CAN message:
 * @param id is the CAN ID
 * @param data is the data buffer
 * @param len is the length of the message
 *
 * The big switch statement converts the byte length (0-8) to the FDCAN DLC (Data Length Code) enum format the HAL expects.
 * Then the method adds the message to the TX FIFO queue
 */
int FDCAN_write(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief This method just stores pointers so that the interrupt handler can use them.
 *
 *        The interrupt handler is HAL_FDCAN_RxFifo0Callback()
 *        This is called automatically by the hardware when a CAN message arrives.
 *        The method retrieves the message from the hardware FIFO,
 *        then it converts it from the STM32 HAL format (FDCAN_RxHeaderTypeDef) to our CAN_message_t format
 *        Then it calls DashCAN::dash_read_switch() to route it to the right interface
 *
 */
void FDCAN_set_interfaces(CANInterfaces_s &interfaces);

/**
 * @brief This method configures the physical pins:
 *        - Enables Port D clock
 *        - Sets PD0 and PD1 to Alternate Function mode with AF9 (which is the FDCAN1 function on those pins)
 *        - STM32 pins are multiplexed since the same physical pin can be GPIO, SPI, CAN, UART etc.
 */
void FDCAN1_GPIO_init_PD0D1(void);

#endif