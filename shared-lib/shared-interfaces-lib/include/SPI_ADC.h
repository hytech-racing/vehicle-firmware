#ifndef SPIADC_H_
#define SPIADC_H_

#include "SPI_DMA.h"
#include "AnalogSensorsInterface.h"
#include <EventResponder.h>

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

template <size_t bufferSize, int ADC_NUM_CHANNELS>
class SPI_ADC : public SPIDMAInterface<bufferSize>, public AnalogMultiSensor<ADC_NUM_CHANNELS> {
    public:
    SPI_ADC(EventResponder& eventRef, 
                    const float scales[ADC_NUM_CHANNELS], 
                    const float offsets[ADC_NUM_CHANNELS],
                    const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes, 
                    SPIParams_s spiParams,
                    bool SSTRBpresent = false, 
                    uint8_t SSTRBpin = 0);

    void _sample() override;
    void tick() override;
    void transferComplete() override;
    virtual uint16_t setValue();
    virtual void setCommand();
 
    int getCurrChannel() { return _currChannel; }
    uint32_t getCycles();

    protected:
    ChannelTypeArray<ADC_NUM_CHANNELS> _channelTypes;
    bool _SSTRBpresent = false;
    uint8_t _SSTRBpin = 0;
    volatile int _currChannel;
    unsigned long lastSampleTime;
    uint32_t _cycles = 0;
    uint32_t _isrCycles = 0;
    uint32_t _totCycles = 0;
};

#include "SPI_ADC.tpp" 


#endif // SPIADC_H_