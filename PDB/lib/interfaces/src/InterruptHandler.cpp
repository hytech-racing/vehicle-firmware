#include "HT_I2C.h"
#include "TempSensorInterface.h"
#include "Hotswap.h"

//externed so the hal library is able to exploit strong-weak callbacks
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == HS5066._config.SMBA_PIN) {
        HS5066.smbaIrqHandler();
    }
    else if (GPIO_Pin == HS5066._config.PGD_PIN) {
        HS5066.smbaIrqHandler();
    }
    else {
        for (auto &t: temps) {
            if (t.alert_pin == GPIO_Pin) {
                t.onAlertIrq();
            }
        }
    }
}