
#include "SystemTimeInterface.h"

// mock system time interface

unsigned long millis, micros;

namespace sys_time {
unsigned long hal_millis() { return millis; }
unsigned long hal_micros() { return micros; }

void set_millis(unsigned long m)
{
    millis = m;
}

void set_micros(unsigned long m)
{
    micros = m;
}

} // namespace sys_time
