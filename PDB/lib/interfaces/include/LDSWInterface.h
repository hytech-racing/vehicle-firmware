#ifndef LDSW_INTERFACE_H
#define LDSW_INTERFACE_H

#include <cstdint>

class LDSWInterface
{
public:
    struct Config_s
    {
        std::uint32_t enable_pin;
        std::uint32_t fault_pin;
        std::uint32_t imon_pin;
        std::uint32_t imon_na_per_count; // current-monitor conversion factor (nanoamps per ADC count)
    };

    explicit LDSWInterface(const Config_s &config);
    void init();
    void enable();
    void disable();
    void sample_fault();   // Sample the active-low fault pin and store whether a fault is present.
    void sample_current(); // Sample the current-monitor ADC pin and update the current in milliamps.

    bool is_enabled() const { return _is_enabled; } // commanded EN state, not hardware output rail status
    bool is_faulted() const { return _is_faulted; } // most recent FLT sample
    std::uint32_t current_mA() const { return _current_mA; }

private:
    /**
     * @brief Convert ADC counts to whole milliamps using the configured scale.
     * current in mA = counts × na_per_count ÷ 1,000,000
     * 
     * @param counts Current-monitor ADC reading.
     * @param na_per_count Nanoamps represented by each ADC count.
     * @return Current in milliamps, truncated to a whole number.
     */
    static std::uint32_t _counts_to_mA(std::uint16_t counts, std::uint32_t na_per_count);

    Config_s _config; // holds this switch's settings.
    bool _is_enabled = false;
    bool _is_faulted = false;
    std::uint32_t _current_mA = 0;
};

#endif // LDSW_INTERFACE_H
