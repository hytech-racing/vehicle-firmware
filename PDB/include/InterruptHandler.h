#ifndef PDBIntHandler
#define PDBIntHandler

//Not as important as the stmh7xx_hal file already includes this weak callback
//externed so the hal library is able to exploit strong-weak callbacks
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif