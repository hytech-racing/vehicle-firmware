#ifndef TEENSY_ADC_HPP
#define TEENSY_ADC_HPP

#include "AnalogSensorsInterface.hpp"
#include "Filter_IIR.h"

static const int TEENSY_ADC_DEFAULT_RESOLUTION = 12;

template <int TEENSY_ADC_NUM_CHANNELS>
class Teensy_ADC : public AnalogMultiSensor<TEENSY_ADC_NUM_CHANNELS>, public FilterIIRMulti<uint16_t, TEENSY_ADC_NUM_CHANNELS>
{
  public:
    Teensy_ADC(const int* pins);
    void init();
    void tick();
    void sample();
    void setAlphas(int pin, float channel);
    uint16_t translatePin(int pin);

  private:
    int _pins[TEENSY_ADC_NUM_CHANNELS];
};

#include "Teensy_ADC.tpp"

#endif // TEENSY_ADC_HPP