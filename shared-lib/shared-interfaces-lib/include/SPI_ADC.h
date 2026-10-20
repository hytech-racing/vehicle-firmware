#ifndef SPIADC_H_
#define SPIADC_H_

#include "SPI_DMA.h"
#include "AnalogSensorsInterface.h"

using namespace std;

enum class ChannelType_e
{
    SINGLE, ///< single channel
    DIFFERENTIAL, ///< +- differential pair
    INV_DIFFERENTIAL, ///< -+ differential pair
    NOT_USED,
};

template <int N>
#ifdef DIFF_CHANNELS
using ChannelTypeArray = std::array<ChannelType_e, N / 2>;
#else
using ChannelTypeArray = std::array<ChannelType_e, N>;
#endif

namespace mcp_adc_defaults
{
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_CS    = 10;
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_SDI   = 12;
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_SDO   = 11;
    constexpr const uint8_t MCP_ADC_DEFAULT_SPI_CLK   = 13;
    constexpr const int MCP_ADC_DEFAULT_SPI_SPEED = 2000000;
};


template <size_t bufferSize, int ADC_NUM_CHANNELS>
class SPI_ADC : public SPIDMAInterface<size_t bufferSize>, public AnalogMultiSensor<ADC_NUM_CHANNELS> {
    public:
    SPI_ADC(EventResponder* eventRef, 
                    const float scales[ADC_NUM_CHANNELS], 
                    const float offsets[ADC_NUM_CHANNELS],
                    const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes,
                    SPIParams_s spiParams = {
                        .spiPinCS = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_CS,
                        .spiPinSDI = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_SDI,
                        .spiPinSDO = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_SDO,
                        .spiPinCLK = mcp_adc_defaults::MCP_ADC_DEFAULT_SPI_CLK,
                    });

    SPI_ADC(SPIParams_s spiParams, EventResponder* eventRef, 
                    const float scales[ADC_NUM_CHANNELS], 
                    const float offsets[ADC_NUM_CHANNELS],
                    const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes, 
                    bool SSTRBpresent = false, uint8_t SSTRBpin = 0);

    void _sample() override;
    void tick() override;
    void dmaCallback(SPIClass* SPI) override;
    virtual void setValue();
 
    int getCurrChannel() { return _currChannel; }

    protected:
    ChannelTypeArray<ChannelType_e, ADC_NUM_CHANNELS> _channelTypes;
    bool _SSTRBpresent = false;
    uint8_t _SSTRBpin = 0;
    int _currChannel;
};


#endif // SPIADC_H_