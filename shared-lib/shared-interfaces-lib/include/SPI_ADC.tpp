#include "SPI_ADC.h"

template <size_t bufferSize, int ADC_NUM_CHANNELS>
SPIADCInterface<bufferSize, ADC_NUM_CHANNELS>::SPIADCInterface(
    EventResponder* eventRef,
    const float scales[ADC_NUM_CHANNELS], 
    const float offsets[ADC_NUM_CHANNELS],
    const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes,
    SPIParams_s spiParams)
    : SPIADCInterface(spiParams, eventRef, scales, offsets, channelTypes)
    {};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
SPIADCInterface<bufferSize, ADC_NUM_CHANNELS>::SPIADCInterface(SPIParams_s spiParams,
        EventResponder* eventRef,
        const float scales[ADC_NUM_CHANNELS], 
        const float offsets[ADC_NUM_CHANNELS],
        const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes,
        bool SSTRBpresent,
        uint8_t SSTRBpin)
    : SPIDMAInterface<bufferSize>(spiParams, eventRef,
                                  std::array<uint8_t, bufferSize>{},
                                  std::array<uint8_t, bufferSize>{}),
      _channelTypes(channelTypes),
      _SSTRBpresent(SSTRBpresent),
      _SSTRBpin(SSTRBpin),
      _currChannel(0),
{
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);
    }
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPIADCInterface<bufferSize, ADC_NUM_CHANNELS>::tick() {
    if (_dmaBusy || (_SSTRBpresent && digitalRead(_SSTRBpin) == HIGH)) {
        return;
    }
    _sample();
    this->_convert();
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPIADCInterface<bufferSize, ADC_NUM_CHANNELS>::_sample() {
    _currChannel = 0;
#ifdef DIFF_CHANNELS
    ChannelType_e channelType = _channelTypes[_currChannel / 2];
#else
    ChannelType_e channelType = _channelTypes[_currChannel];
#endif
    while (_currChannel < ADC_NUM_CHANNELS) {
        setCommand();
        transfer(&SPI);
        if (_channelTypes[_currChannel] == ChannelType_e::NOT_USED) {
#ifdef DIFF_CHANNELS
            _currChannel++;
#endif
            _currChannel++;
        }
    }
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPIADCInterface<bufferSize, ADC_NUM_CHANNELS>::dmaCallback(SPIClass* SPI) {
#ifdef DIFF_CHANNELS
    ChannelType_e channelType = _channelTypes[_currChannel / 2];
    if (channelType == ChannelType_e::DIFFERENTIAL || channelType = ChannelType_e::INV_DIFFERENTIAL) {
        _currChannel++;
    }
#endif
    _currChannel++;
    uint16_t value = setValue();
    SPIADCInterface<bufferSize, ADC_NUM_CHANNELS>::_channels[_currChannel].lastSample = value;
    digitalWrite(_spiParams.spiPinCS, HIGH);
    SPI->endTransaction();
    this->_dmaBusy = false;
}