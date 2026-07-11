#ifndef BUZZER
#define BUZZER

/**
 * This class controls the car's buzzer. It is a singleton class (since we only have one buzzer).
 *
 * NOTE: This class does NOT directly control the buzzer. Rather, it does the calculations of whether
 *       or not the buzzer SHOULD be active. The output of this system then goes to the Dashboard,
 *       who will activate the buzzer when this system outputs HIGH.
 */
class BuzzerController
{
public:

    static BuzzerController& getInstance()
    {
        static BuzzerController instance;
        return instance;
    }

    /**
     * Calling this command will activate the buzzer for BUZZER_PERIOD_MS milliseconds.
     */
    void activate(unsigned long curr_millis)
    {
        _last_activation_time_ms = curr_millis;
    }

    /**
     * Immediately kill the buzzer.
     */
    void deactivate()
    {
        _last_activation_time_ms = 0;
    }

    /**
     * Returns whether or not the buzzer should currently be active, according to the system.
     */
    bool buzzer_is_active(unsigned long millis)
    {
        return _last_activation_time_ms != 0 && (millis - _last_activation_time_ms) < _BUZZER_PERIOD_MS;
    }

private:

    const unsigned long _BUZZER_PERIOD_MS = 2000; // Default buzzer period is 2000ms
    unsigned long _last_activation_time_ms;

    BuzzerController()
    {
        _last_activation_time_ms = 0;
    }

};

#endif /* BUZZER */
