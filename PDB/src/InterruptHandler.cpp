#include "HT_I2C.h"
#include "TempSensorInterface.hpp"
#include "Hotswap.h"

//The SMDA pin is set up as a GPIO EXTI


//externed so the hal library is able to exploit strong-weak callbacks
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    /* Checks where the interrupt from the rising/falling edge occurs and calls
    * necessary function (hotswap, temp sensor)
    */
    if (GPIO_Pin == HS5066._config.SMBA_PIN) {
        HS5066.smbaIrqHandler();
    }
    else if (GPIO_Pin == HS5066._config.PGD_PIN) {
        HS5066.smbaIrqHandler(); //not fully setup since it is only a PGOOD
    }
    else {
        for (auto &t: temps) {
            if (t.alert_pin == GPIO_Pin) {
                t.onAlertIrq();
            }
        }
    }
}