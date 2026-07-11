#include "HT_FDCAN.h"


FDCAN_HandleTypeDef hfdcan1;

// Static pointers for interrupt handler
static CANInterfaces_s* global_interfaces = nullptr;
static unsigned long global_last_millis = 0;


int FDCAN_init()
{
    __HAL_RCC_FDCAN_CLK_ENABLE();
    FDCAN1_GPIO_init_PD0D1();

    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 1;
    hfdcan1.Init.NominalSyncJumpWidth = 7;
    hfdcan1.Init.NominalTimeSeg1 = 42;
    hfdcan1.Init.NominalTimeSeg2 = 7;
    hfdcan1.Init.DataPrescaler = 2;
    hfdcan1.Init.DataSyncJumpWidth = 8;
    hfdcan1.Init.DataTimeSeg1 = 16;
    hfdcan1.Init.DataTimeSeg2 = 8;
    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 1;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr = 0;
    hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr = 16;
    hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        return 0;
    }

    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0; // Use the first available Standard filter slot
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // Set to mask mode
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Route accepted messages to FIFO0
    sFilterConfig.FilterID1 = 0x000; // Filter ID (e.g., 0x000 to match all IDs)
    sFilterConfig.FilterID2 = 0x000; // Mask ID (e.g., 0x000 mask to accept all IDs)

    // Accept ALL standard IDs for testing (ID & Mask == ID & FilterID1)
    // If FilterID1 = 0, FilterID2 = 0, then: (ID & 0) == (ID & 0) -> always true
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        // Handle Error
        return 0;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return 0;
    }

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    return 1;
}

int FDCAN_write(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    // Map len to DLC
    switch(len)
    {
        case 0:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_0;
            break;
        }
        case 1:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_1;
            break;
        }
        case 2:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_2;
            break;
        }
        case 3:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_3;
            break;
        }
        case 4:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_4;
            break;
        }
        case 5:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_5;
            break;
        }
        case 6:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_6;
            break;
        }
        case 7:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_7;
            break;
        }
        case 8:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        }
        default:
        {
            txHeader.DataLength = FDCAN_DLC_BYTES_8; break;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)data) != HAL_OK)
    {
        return 0; // send failed
    }

    return 1;
}

void FDCAN_set_interfaces(CANInterfaces_s &interfaces)
{
    // Store references for use in interrupt handler
    global_interfaces = &interfaces;
}

void FDCAN1_GPIO_init_PD0D1(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // 1. Enable GPIO Port D clock
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // 2. Configure PD0 (RX) and PD1 (TX)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // Use the correct high speed for stable CAN signaling
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    // Use the correct Alternate Function for FDCAN1 on PD0/PD1 (AF9)
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Ensure FDCAN clock is enabled, though it's also in FDCAN_Init()
    __HAL_RCC_FDCAN_CLK_ENABLE();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((hfdcan->Instance == FDCAN1) && ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0))
    {
        // Only process if pointers have been set
        if (global_interfaces == nullptr) return;
        {

            FDCAN_RxHeaderTypeDef rxHeader;
            uint8_t rxData[8];

            // Retrieve the message from FIFO 0
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
            {
                CAN_message_t msg;
                msg.id = rxHeader.Identifier;
                msg.len = rxHeader.DataLength;
                msg.buf = rxData;
                DashCAN::dash_CAN_recv_switch(*global_interfaces, msg, global_last_millis);
            }
        }
    }
}

/**
  * @brief FDCAN MSP Initialization
  * This function configures the hardware resources for FDCAN
  * Called automatically by HAL_FDCAN_Init()
  * Sets up the interrupt priority and enables it in the NVIC
  * (Nested Vectored Interrupt Controller — the STM32's interrupt manager)
  */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        // Enable FDCAN1 clock (already enabled in FDCAN_Init, but doing it here too for completeness)
        __HAL_RCC_FDCAN_CLK_ENABLE();

        // Enable FDCAN1 IT0 interrupt in NVIC
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
}