#ifndef MCP_DMA_ADC_HPP
#define MCP_DMA_ADC_HPP

/**
 * MCP_DMA_ADC - non-blocking DMA driver for the MCP320x family of SPI ADCs.
 *
 * Talks to the same chip as MCP_ADC (MCP_ADC.hpp), but works differently:
 *  - MCP_ADC::tick() reads every channel back to back with blocking SPI.transfer()
 *    calls, so the CPU waits on the SPI bus for the whole read.
 *  - MCP_DMA_ADC::tick() starts a DMA transfer for ONE channel and returns right away.
 *    When the transfer finishes, the EventResponder callback (transferComplete())
 *    saves the sample and moves to the next channel. After the last channel, the
 *    next tick() runs _convert() and starts again from channel 0, so a full
 *    refresh takes (num channels + 1) ticks.
 *
 * setCommand() and setValue() build the MCP320x command and decode its 12-bit reply.
 * This class is MCP-only: a different SPI ADC should get its own driver that inherits
 * from SPIDMAInterface (SPI_DMA.hpp).
 *
 * NOTE: call initDMA() before the first tick(). Several devices can share one SPI
 * bus: tick() does nothing while any of them is mid-transfer (see isBusBusy()).
 */

#include "AnalogSensorsInterface.hpp"
#include "SPI_DMA.hpp"

#include <EventResponder.h>

/**
 * How each MCP input is read. One entry per channel (index = channel number).
 *
 * Differential pairs are always CH0/CH1, CH2/CH3, CH4/CH5, CH6/CH7 (MCP320x
 * pseudo-differential mode: IN- must stay near AGND, and IN+ < IN- reads 0).
 * Set DIFFERENTIAL or INV_DIFFERENTIAL on the EVEN channel of the pair; the result
 * is stored in that even channel, and the odd partner is skipped (mark it NOT_USED).
 */
enum class ChannelType_e
{
    SINGLE,           // CHn measured against AGND
    DIFFERENTIAL,     // CHn = IN+, CHn+1 = IN-  (n even)
    INV_DIFFERENTIAL, // CHn+1 = IN+, CHn = IN-  (n even)
    NOT_USED,         // skipped
};

template <int N>
using ChannelTypeArray = std::array<ChannelType_e, N>;


namespace mcp_adc_defaults
{
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_CS = 10;
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_SDI = 12;
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_SDO = 11;
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_CLK = 13;
    constexpr const int MCP_ADC_DEFAULT_SPI_SPEED = 2000000;
}; // namespace mcp_adc_defaults

template <size_t bufferSize, int ADC_NUM_CHANNELS>
class MCP_DMA_ADC : public SPIDMAInterface<bufferSize>, public AnalogMultiSensor<ADC_NUM_CHANNELS>
{
  public:
    MCP_DMA_ADC(
        EventResponder& eventRef,
        const float scales[ADC_NUM_CHANNELS],
        const float offsets[ADC_NUM_CHANNELS],
        const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes,
        SPIParams_s spiParams = {
            .spiPinCS = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_CS,
            .spiPinSDI = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_SDI,
            .spiPinSDO = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_SDO,
            .spiPinCLK = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_CLK,
        });

    MCP_DMA_ADC(
        SPIParams_s spiParams,
        EventResponder& eventRef,
        const float scales[ADC_NUM_CHANNELS],
        const float offsets[ADC_NUM_CHANNELS],
        const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes);

    void _sample() override;
    void tick() override;
    void transferComplete() override;
    virtual uint16_t setValue();
    virtual void setCommand();

    int getCurrChannel() { return _currChannel; }
    uint32_t getCycles();

  protected:
    ChannelTypeArray<ADC_NUM_CHANNELS> _channelTypes;
    volatile int _currChannel;
};

#include "MCP_DMA_ADC.tpp"


#endif // MCP_DMA_ADC_HPP