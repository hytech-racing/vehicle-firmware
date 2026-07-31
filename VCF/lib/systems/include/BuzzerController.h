#ifndef BUZZER
#define BUZZER

/* ETL Library Includes */
#include <etl/singleton.h>


class BuzzerController
{
public:

    BuzzerController()
    {
        _last_activation_time_ms = 0;
    }

    void activate(unsigned long curr_millis)
    {
        _last_activation_time_ms = curr_millis;
    }

    void deactivate()
    {
        _last_activation_time_ms = 0;
    }

    bool buzzer_is_active(unsigned long millis)
    {
        return _last_activation_time_ms != 0 && (millis - _last_activation_time_ms) < _BUZZER_PERIOD_MS;
    }

private:

    const unsigned long _BUZZER_PERIOD_MS = 2000;
    unsigned long _last_activation_time_ms;
    
};

using BuzzerControllerInstance = etl::singleton<BuzzerController>;

#endif /* BUZZER */