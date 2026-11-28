
#include "SystemTimeInterface.h"

// mock system time interface

unsigned long millis, micros;

namespace sys_time
{
    unsigned long halMillis()
    {
        return millis;
    }
    unsigned long halMicros()
    {
        return micros;
    }

    void setMillis(unsigned long m)
    {
        millis = m;
    }

    void setMicros(unsigned long m)
    {
        micros = m;
    }

} // namespace sys_time
