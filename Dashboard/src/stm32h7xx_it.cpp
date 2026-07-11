/**
  ******************************************************************************
  * @file    stm32h7xx_it.cpp
  * @brief   Interrupt Service Routines for STM32H7xx
  ******************************************************************************
  */

extern "C"
{
    // Forward declarations
    extern void HAL_FDCAN_IRQHandler(void *hfdcan);
    extern struct
    {
        void *Instance;
    } hfdcan1;

    /**
      * @brief  FDCAN1 Interrupt Handler
      */
    void FDCAN1_IT0_IRQHandler(void)
    {
        HAL_FDCAN_IRQHandler(&hfdcan1);
    }
}
