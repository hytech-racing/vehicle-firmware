#ifndef __TEENSY_ADC_H__
#define __TEENSY_ADC_H__
#include "AnalogSensorsInterface.h"
#include "Filter_IIR.h"
const int TEENSY_ADC_DEFAULT_RESOLUTION = 12;

template <int TEENSY_ADC_NUM_CHANNELS>
class Teensy_ADC : public AnalogMultiSensor<TEENSY_ADC_NUM_CHANNELS>, public FilterIIRMulti<uint16_t, TEENSY_ADC_NUM_CHANNELS>
{
private:
    int _pins[TEENSY_ADC_NUM_CHANNELS];
public:
    Teensy_ADC(const int* pins);
    void init();
    void tick();
    void sample();
    void setAlphas(int pin, float channel);
    uint16_t translatePin(int pin);
};
#include "Teensy_ADC.tpp"







#endif /*__TEENSY_ADC_H__*/