#include "SystemTimeInterface.h"

#include <Arduino.h>

namespace sys_time
{
    unsigned long halMillis()
    {
        return millis();
    }

    unsigned long halMicros()
    {
        return micros();
    }
} // namespace sys_time
