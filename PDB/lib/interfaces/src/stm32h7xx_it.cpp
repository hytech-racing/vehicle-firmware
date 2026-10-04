#include "stm32h7xx_hal.h" 

extern FDCAN_HandleTypeDef hfdcan1; //promises that this exists somewhere else, so the pointer can be passed
extern "C" {
  void FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan1);
  }
}

//A lot simpler than Nazar's implementation by circumventing forward declaration, ask why