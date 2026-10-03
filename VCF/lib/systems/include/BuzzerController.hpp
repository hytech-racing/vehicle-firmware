#ifndef BUZZER_CONTROLLER_HPP
#define BUZZER_CONTROLLER_HPP

#include <etl/singleton.h>


class BuzzerController
{
public:

    BuzzerController()
    {
        _last_activation_time_ms = 0;
    }

    /**
     * @warning Method itself does not activate the buzzer!
     * @note Method sets _last_activation_time_ms to curr_millis
    */
    void activate(unsigned long curr_millis);

    /**
     * @warning Method itself does not deactivate the buzzer!
     * @note Method sets _last_activation_time_ms to 0
    */
    void deactivate();

    bool isBuzzerActive(unsigned long millis);

private:

    const unsigned long _BUZZER_PERIOD_MS = 2000;
    unsigned long _last_activation_time_ms;

};

using BuzzerControllerInstance = etl::singleton<BuzzerController>;

#endif /* BUZZER */