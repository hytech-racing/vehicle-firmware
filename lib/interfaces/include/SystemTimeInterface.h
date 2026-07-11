#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

/* External Inclides */
#include <Arduino.h>


namespace sys_time
{
    unsigned long hal_millis();
    unsigned long hal_micros();
}

#endif // SYSTEMTIMEINTERFACE_H