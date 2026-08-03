#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

namespace sys_time
{
    unsigned long hal_millis();
    unsigned long hal_micros();

    void set_millis(unsigned long m);
    void set_micros(unsigned long m);
}



#endif // SYSTEMTIMEINTERFACE_H