#include "LDSWInterface.h"
#include <Arduino.h>

LDSWInterface::LDSWInterface(const Config_s &config)
    : _config(config)
{
}

void LDSWInterface::init()
{
    pinMode(_config.enable_pin, OUTPUT);
    digitalWrite(_config.enable_pin, LOW);
    pinMode(_config.fault_pin, INPUT);
    pinMode(_config.imon_pin, INPUT_ANALOG);
    _is_enabled = false;
}

void LDSWInterface::enable()
{
    digitalWrite(_config.enable_pin, HIGH);
    _is_enabled = true;
}

void LDSWInterface::disable()
{
    digitalWrite(_config.enable_pin, LOW);
    _is_enabled = false;
}

void LDSWInterface::sample_fault()
{
    _is_faulted = (digitalRead(_config.fault_pin) == LOW);
}

void LDSWInterface::sample_current()
{
    const std::uint16_t counts = static_cast<std::uint16_t>(analogRead(_config.imon_pin)); // set ADC resolution once.
    _current_mA = _counts_to_mA(counts, _config.imon_na_per_count);
}

std::uint32_t LDSWInterface::_counts_to_mA(std::uint16_t counts, std::uint32_t na_per_count)
{
    constexpr std::uint64_t kNanoampsPerMilliamp = 1'000'000ULL;
    const std::uint64_t current_nA = static_cast<std::uint64_t>(counts) * na_per_count;
    return static_cast<std::uint32_t> (current_nA / kNanoampsPerMilliamp);
}
