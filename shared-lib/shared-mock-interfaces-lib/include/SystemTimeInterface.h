#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

namespace sys_time
{
    unsigned long halMillis();
    unsigned long halMicros();

    void setMillis(unsigned long m);
    void setMicros(unsigned long m);
} // namespace sys_time


#endif // SYSTEMTIMEINTERFACE_H