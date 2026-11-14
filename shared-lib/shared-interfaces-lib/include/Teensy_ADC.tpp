#include "Teensy_ADC.hpp"


template <int TEENSY_ADC_NUM_CHANNELS>
Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::Teensy_ADC(const int* pins)
{
    analogReadResolution(TEENSY_ADC_DEFAULT_RESOLUTION);
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        _pins[i] = pins[i];
        this->_channels[i] = AnalogChannel();
        this->filter_channels_[i] = Filter_IIR<uint16_t>(0, analogRead(_pins[i]));
    }
}

template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::init()
{
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        pinMode(_pins[i], INPUT);
    }
}

template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::tick()
{
    _sample();
    this->_convert();
}

template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::setAlphas(int pin, float alpha)
{
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        if (pin == this->_pins[i])
        {
            FilterIIRMulti<uint16_t, TEENSY_ADC_NUM_CHANNELS>::setAlphas(i, alpha);
            break;
        }
    }
}

template <int TEENSY_ADC_NUM_CHANNELS>
uint16_t Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::translatePin(int pin)
{
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        if (pin == this->_pins[i])
        {
            return i;
        }
    }
}

template <int TEENSY_ADC_NUM_CHANNELS>
void Teensy_ADC<TEENSY_ADC_NUM_CHANNELS>::_sample()
{
    for (int i = 0; i < TEENSY_ADC_NUM_CHANNELS; i++)
    {
        uint16_t val = analogRead(_pins[i]);
        this->_channels[i].lastSample = (this->filter_channels_[i].filtered_result(val) & (~(1 << TEENSY_ADC_DEFAULT_RESOLUTION)));
    }
}