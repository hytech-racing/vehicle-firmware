#ifndef TEENSY_ADC_HPP
#define TEENSY_ADC_HPP

#include "AnalogSensorsInterface.hpp"
#include "Filter_IIR.h"


static const int TEENSY_ADC_DEFAULT_RESOLUTION = 12;

/**
 * @brief Interface for the Teensy's onboard ADC. Reads a set of analog pins with analogRead(),
 *        runs each reading through an IIR filter, and stores the result in the matching AnalogChannel.
 * @tparam TEENSY_ADC_NUM_CHANNELS number of analog pins (channels) being read
 */
template <int TEENSY_ADC_NUM_CHANNELS>
class Teensy_ADC : public AnalogMultiSensor<TEENSY_ADC_NUM_CHANNELS>, public FilterIIRMulti<uint16_t, TEENSY_ADC_NUM_CHANNELS>
{
  public:
    /**
     * @brief Sets the ADC read resolution, stores the pins, and seeds each channel's filter with an initial reading.
     *
     * @param pins array of Teensy analog pin numbers, size TEENSY_ADC_NUM_CHANNELS.
     */
    Teensy_ADC(const int* pins);

    /**
     * @brief Configures every pin as an INPUT. Call once during setup before sampling.
     */
    void init();

    /**
     * @brief Samples every channel, then converts the raw samples using each channel's scale and offset.
     */
    void tick() override;

    /**
     * @brief Sets the IIR filter alpha for the channel that reads the given pin.
     *        Does nothing if the pin is not one of this ADC's pins.
     *
     * @param pin Teensy analog pin number
     * @param alpha IIR filter alpha for that pin's channel
     */
    void setAlphas(int pin, float alpha);

    /**
     * @brief Gets the channel index that reads the given pin.
     *
     * @param pin Teensy analog pin number
     * @return index of the channel that reads the pin
     */
    uint16_t translatePin(int pin);

  protected:
    /**
     * @brief Reads every pin, filters the reading, and stores it as the channel's lastSample.
     */
    void _sample() override;

  private:
    int _pins[TEENSY_ADC_NUM_CHANNELS];
};

#include "Teensy_ADC.tpp"

#endif // TEENSY_ADC_HPP
