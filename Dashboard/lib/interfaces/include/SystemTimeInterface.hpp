#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

/* External Includes */
#include <Arduino.h>


namespace sys_time
{
    unsigned long halMillis();
    unsigned long halMicros();
} // namespace sys_time

#endif // SYSTEMTIMEINTERFACE_H