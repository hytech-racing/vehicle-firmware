#include "fdcan.h"
#include "PDBCANInterfaceImpl.h"

FDCAN_HandleTypeDef hfdcan1;
static CANInterfaces_s* global_interfaces = nullptr;

/* FDCAN1 init function */
int FDCAN_Init(void) {
  __HAL_RCC_FDCAN_CLK_ENABLE();
  FDCAN1_PD0PD1_init();
  //HAL_FDCAN_MspInit Automatically called

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1; //Must be update to allow a 1 always true rule
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8; //Space for 8 can message frames in the recieve FIFO
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8; //Space for 8 CAN message frames to be send out in the FIFO
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    return 0;
  if (FDCAN_Config_Start() != 1)
    return 0;
  
  return 1;
}

/**
 * @brief Configures the FDCAN with appropriate can message filtering
 * Also sets up the interrupts for receiving messages, and starts the config (can move to init if preferred)
 * Different CAN filtering modes exist (mask, range), used mask for simplicity, but can be switched as needed
 * @retval None
 */
int FDCAN_Config_Start(void) {
  FDCAN_FilterTypeDef filterConfig;

  filterConfig.IdType = FDCAN_STANDARD_ID;
  filterConfig.FilterIndex = 0;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //store accepted messages into Recieve FIFO 0
  filterConfig.FilterID1 = 0x000;
  filterConfig.FilterID2 = 0x000; //all 0s so any integer works

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig) != HAL_OK) 
    return 0;
  //ensures new messages entering FIFO would trigger interrupt and eventually call HAL_FDCAN_RxFIfo0Callback()
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    return 0;
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    return 0;

  return 1;
}

/* Initializes the FDCAN peripheral clock, rest of the clocks done by pinMode() functions generally */
//Check with Nazar since he did not use any HAL configuration for the CAN clock, but seems logical
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1) {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL; //sets the FDCAN clock
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
      return; // early return -> perhaps consider a different error handler
    __HAL_RCC_FDCAN_CLK_ENABLE(); // FDCAN1 clock enable, extra reundancy from Init

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0); //sets to priority 0 (should not miss CAN messages)
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
}

//PD0D1 code pulled out of the main MSPInit function for modularity
void FDCAN1_PD0PD1_init(void) {
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1; //Pin numbers
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //Switched to very high freq from low (stabilizes CAN - from dash)
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

//Function provided by CubeMx, left for completeness (not in Dashboard)
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle) {
  if(fdcanHandle->Instance==FDCAN1) {
    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
  }
}

/* ------------------------------- FDCAN R/W Functionality -------------------------------*/
//Sets the VCR interfaces for later referencing
void FDCAN_set_interfaces(CANInterfaces_s &interfaces) {
  // Store references for use in interrupt handler
  global_interfaces = &interfaces;
}

// Override the weak symbol in the HAL Code for RxFifo0Callback (exists in the HAL IRQ handler)
// The RXFifo0ITs encodes the different interrupt flags that are currently raised
// We are only interested in the new message one, thus being bitwise anded and compared with 0
// Return if interfaces for processing these can messages do not exist in the first place
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  // Start with checking if there is a CAN Message
  if ((hfdcan ->Instance == FDCAN1) && ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)) {
    if (global_interfaces == nullptr) return;
    
    FDCAN_RxHeaderTypeDef rxHeader;

    //important if multiple can message happens at once, before they are serviced
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
      CAN_message_t msg = {};
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, msg.data) != HAL_OK) 
        break; //so it doesn't keep looping on a faulty message
      msg.id = rxHeader.Identifier;
      msg.extended = (rxHeader.IdType == FDCAN_EXTENDED_ID);
      msg.buf = msg.data;
      msg.len = rxHeader.DataLength;
      PDBCAN::PDB_CAN_receive_switch(*global_interfaces, msg);
    }
  }
}

// Where do I even determine the id when writing code? from the CAN repo as Alex showed
int FDCAN_write(uint32_t id, const uint8_t *data, uint8_t len) {
  FDCAN_TxHeaderTypeDef txHeader; //define a txHeader, fill it up and then add into the queue

  const uint32_t dlc_lengths[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, 
    FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8 }; 
  //set up a small mapping for len to macro definitions

  if (len > 8 || data == nullptr) 
    return 0;
  
  txHeader.Identifier = id; //Can ID
  txHeader.IdType = FDCAN_STANDARD_ID; //11 bit standard id
  txHeader.TxFrameType = FDCAN_DATA_FRAME; // Sends data, not request for data (remote frame)
  txHeader.DataLength = dlc_lengths[len]; //length of the data
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // Dashboard had this as passive - error degradation, ask Nazar
  txHeader.BitRateSwitch = FDCAN_BRS_OFF; //Faster data sending rates
  txHeader.FDFormat = FDCAN_CLASSIC_CAN; //8 bit can (instead of 64 bits)
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //logs when fifo successfuly sends for tracking
  txHeader.MessageMarker = 0; // Set to 0 since no events

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)data) == HAL_OK; //returns 0 if failed
}