#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

/* External Includes */
#include <Arduino.h>


namespace sys_time
{
    unsigned long hal_millis();
    unsigned long hal_micros();
} // namespace sys_time

#endif // SYSTEMTIMEINTERFACE_H