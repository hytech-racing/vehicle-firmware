#ifndef PDB_FDCAN
#define PDB_FDCAN

#include <stm32h7xx_hal.h>
#include <stm32h750xx.h>
#include "hytech.h"
#include "VCRInterface.h"

struct CANInterfaces_s; //forward declaration, type exists somewhere but defined somewhere else (since C++ compiles top to bottom)

// namespace initialize_can { //namespaces are overrated
  int FDCAN_Init(void);
  int FDCAN_Config_Start(void);
  void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle);
  void FDCAN1_PD0PD1_init(void);
  void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle);
//}
//namespace read_write_can {
  void FDCAN_set_interfaces(CANInterfaces_s &interfaces);
  void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
  int FDCAN_write(uint32_t id, const uint8_t *data, uint8_t len);
//}

#endif