#include "MCP_DMA_ADC.hpp"

template <size_t bufferSize, int ADC_NUM_CHANNELS>
MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::MCP_DMA_ADC(
    EventResponder& eventRef,
    const float scales[ADC_NUM_CHANNELS],
    const float offsets[ADC_NUM_CHANNELS],
    const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes,
    SPIParams_s spiParams)
    : MCP_DMA_ADC(spiParams, eventRef, scales, offsets, channelTypes){};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::MCP_DMA_ADC(
    SPIParams_s spiParams,
    EventResponder& eventRef,
    const float scales[ADC_NUM_CHANNELS],
    const float offsets[ADC_NUM_CHANNELS],
    const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes)
    : SPIDMAInterface<bufferSize>(spiParams, eventRef),
      _channelTypes(channelTypes),
      _currChannel(0)
{
    for (int i = 0; i < ADC_NUM_CHANNELS; i++)
    {
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);
    }
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::tick()
{
    if (this->isBusBusy())
    {
        return;
    }
    _sample();
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::_sample()
{
    while (_currChannel < ADC_NUM_CHANNELS && _channelTypes[_currChannel] == ChannelType_e::NOT_USED)
    {
        _currChannel++;
    }

    if (_currChannel >= ADC_NUM_CHANNELS)
    {
        this->_convert();
        _currChannel = 0;
        return;
    }

    setCommand();
    this->transfer();
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::transferComplete()
{
    SPIDMAInterface<bufferSize>::transferComplete();
    MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::_channels[_currChannel].lastSample = setValue();

    // A differential read uses this channel and its odd partner, so skip the partner.
    const ChannelType_e channelType = _channelTypes[_currChannel];
    const bool isDifferential =
        (channelType == ChannelType_e::DIFFERENTIAL || channelType == ChannelType_e::INV_DIFFERENTIAL);
    _currChannel += isDifferential ? 2 : 1;
}

/* Default implementation for MCP ADCs */
template <size_t bufferSize, int ADC_NUM_CHANNELS>
uint16_t MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::setValue()
{
    uint16_t value = (this->_rxBuffer[0] & 0x01) << 11 | (this->_rxBuffer[1] & 0xFF) << 3 | (this->_rxBuffer[2] & 0xE0) >> 5;
    value = (value & 0x0FFF);
    return value;
}

/* Default implementation for MCP ADCs */
template <size_t bufferSize, int ADC_NUM_CHANNELS>
void MCP_DMA_ADC<bufferSize, ADC_NUM_CHANNELS>::setCommand()
{
    const ChannelType_e channelType = _channelTypes[_currChannel];

    // SGL/DIFF bit: 1 = single-ended, 0 = differential
    uint8_t singleEndedBit = 1;
    // D2 D1 D0: single-ended = channel number. Differential = even channel of the
    // pair, with D0 = 1 swapping IN+/IN- (e.g. 000 = CH0+/CH1-, 001 = CH1+/CH0-).
    uint8_t channelBits = _currChannel & 0x07;

    if (channelType == ChannelType_e::DIFFERENTIAL)
    {
        singleEndedBit = 0;
        channelBits = _currChannel & 0x06;
    }
    else if (channelType == ChannelType_e::INV_DIFFERENTIAL)
    {
        singleEndedBit = 0;
        channelBits = (_currChannel & 0x06) | 0x01;
    }

    this->_txBuffer.fill(0);
    this->_txBuffer[0] = (0x01 << 7)              // start bit
                         | (singleEndedBit << 6)  // single-ended or differential
                         | (channelBits << 3);    // channel / pair select
}