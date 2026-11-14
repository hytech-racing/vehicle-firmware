#ifndef MCP_ADC_HPP
#define MCP_ADC_HPP

#include "AnalogSensorsInterface.hpp"
#include <SPI.h>

constexpr int MCP_ADC_DEFAULT_SPI_SPEED = 2'000'000;

/**
 * @brief Interface for the MCP320x family of SPI ADCs (4-channel or 8-channel). Samples every channel
 *        over the global SPI bus and stores the result in the matching AnalogChannel.
 *
 * @note MUST call SPI.begin() once and only once before instantiating any MCP_ADC object!!
 * @tparam MCP_ADC_NUM_CHANNELS number of channels on the ADC
 */
template <int MCP_ADC_NUM_CHANNELS>
class MCP_ADC : public AnalogMultiSensor<MCP_ADC_NUM_CHANNELS>
{
  public:
    /**
     * @brief Stores the SPI settings and each channel's scale and offset, then drives the CS pin HIGH (idle).
     *
     * @param spiPinCS chip select pin for this ADC
     * @param spiSpeed SPI clock speed in Hz
     * @param scales array of per-channel scales, size MCP_ADC_NUM_CHANNELS
     * @param offsets array of per-channel offsets, size MCP_ADC_NUM_CHANNELS
     */
    MCP_ADC(
        const int spiPinCS,
        const int spiSpeed,
        const float scales[MCP_ADC_NUM_CHANNELS],
        const float offsets[MCP_ADC_NUM_CHANNELS]);

    /**
     * @brief Samples every channel, then converts the raw samples using each channel's scale and offset.
     *        After calling tick(), this ADC's data can be accessed using get().
     */
    void tick() override;

  protected:
    /**
     * @brief Reads every channel over SPI and stores the raw 12-bit value as the channel's lastSample.
     */
    void _sample() override;

  private:
    const int _spiPinCS;
    const int _spiSpeed;
};

#include "MCP_ADC.tpp"

#endif // MCP_ADC_HPP
