#include "SPI_ADC.h"

template <size_t bufferSize, int ADC_NUM_CHANNELS>
SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::SPI_ADC(
        EventResponder& eventRef,
        const float scales[ADC_NUM_CHANNELS], 
        const float offsets[ADC_NUM_CHANNELS],
        const ChannelTypeArray<ADC_NUM_CHANNELS>& channelTypes,
        SPIParams_s spiParams,
        bool SSTRBpresent,
        uint8_t SSTRBpin)
    : SPIDMAInterface<bufferSize>(spiParams, eventRef,
                                  std::array<uint8_t, bufferSize>{},
                                  std::array<uint8_t, bufferSize>{}),
      _channelTypes(channelTypes),
      _SSTRBpresent(SSTRBpresent),
      _SSTRBpin(SSTRBpin),
      _currChannel(0)
{
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);
    }
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::tick() {
    // for MCP ADCs, wait at least 1 microsecond before sampling next channel
    if (this->_dmaBusy || micros() - lastSampleTime < 1) {
        return;
    }
    _sample();
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::_sample() {
#ifdef DIFF_CHANNELS
    const int group = 2;
#else
    const int group = 1;
#endif
    while (_currChannel < ADC_NUM_CHANNELS && _channelTypes[_currChannel] == ChannelType_e::NOT_USED) {
            _currChannel += group;
    }

    if (_currChannel >= ADC_NUM_CHANNELS) {      // sweep finished
        this->_convert();
        _totCycles = _cycles + _isrCycles;
        Serial.println("Total CPU Cycles: "); 
        Serial.println(_totCycles);
        _cycles = 0;
        _isrCycles = 0;
        _currChannel = 0;
        return;
    }

    const uint32_t start = ARM_DWT_CYCCNT;
    setCommand();
    this->transfer();
    lastSampleTime = micros();
    _cycles += (ARM_DWT_CYCCNT - start);
};

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::transferComplete() {
    const uint32_t isrStart = ARM_DWT_CYCCNT;
    SPIDMAInterface<bufferSize>::transferComplete(); 
    SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::_channels[_currChannel].lastSample = setValue();
#ifdef DIFF_CHANNELS
    ChannelType_e channelType = _channelTypes[_currChannel / 2];
    if (channelType == ChannelType_e::DIFFERENTIAL || channelType == ChannelType_e::INV_DIFFERENTIAL) {
        _currChannel++;
    }
#endif
    _currChannel++;
    _isrCycles += (ARM_DWT_CYCCNT - isrStart);
}

template <size_t bufferSize, int ADC_NUM_CHANNELS>
uint16_t SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::setValue() {
    uint16_t value = (this->_rxBuf[0] & 0x01) << 11 | (this->_rxBuf[1] & 0xFF) << 3 | (this->_rxBuf[2] & 0xE0) >> 5;
    value = (value & 0x0FFF);
    return value;
}

template <size_t bufferSize, int ADC_NUM_CHANNELS>
void SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::setCommand() {
    this->_txBuf.fill(0);
    this->_txBuf[0] = ((0x01 << 7) |                    // start bit
                   (0x01 << 6) |                        // single or differential
                   ((_currChannel & 0x07) << 3)); 
}

template <size_t bufferSize, int ADC_NUM_CHANNELS>
uint32_t SPI_ADC<bufferSize, ADC_NUM_CHANNELS>::getCycles() {
    return _totCycles;
}