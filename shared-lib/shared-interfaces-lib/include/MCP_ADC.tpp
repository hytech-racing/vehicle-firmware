#include "MCP_ADC.hpp"

template <int MCP_ADC_NUM_CHANNELS>
MCP_ADC<MCP_ADC_NUM_CHANNELS>::MCP_ADC(
    const int spiPinCS,
    const int spiSpeed,
    const float scales[MCP_ADC_NUM_CHANNELS],
    const float offsets[MCP_ADC_NUM_CHANNELS])
: _spiPinCS(spiPinCS),
  _spiSpeed(spiSpeed)
{
    for (int i = 0; i < MCP_ADC_NUM_CHANNELS; i++)
    {
        this->_channels[i] = AnalogChannel();
        this->setChannelScaleAndOffset(i, scales[i], offsets[i]);
    }
    pinMode(_spiPinCS, OUTPUT);
    digitalWrite(_spiPinCS, HIGH);
}

template <int MCP_ADC_NUM_CHANNELS>
void MCP_ADC<MCP_ADC_NUM_CHANNELS>::tick()
{
    _sample();
    this->_convert();
}

template <int MCP_ADC_NUM_CHANNELS>
void MCP_ADC<MCP_ADC_NUM_CHANNELS>::_sample()
{
    // REQUIRED: call SPI.begin() before this
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));

    for (int channelIndex = 0; channelIndex < MCP_ADC_NUM_CHANNELS; channelIndex++)
    {
        digitalWrite(_spiPinCS, LOW);
        const uint8_t command = ((0x01 << 7) |                    // start bit
                                 (0x01 << 6) |                    // single-ended mode
                                 ((channelIndex & 0x07) << 3));   // channel number
        const uint8_t b0 = SPI.transfer(command);
        const uint8_t b1 = SPI.transfer(0x00);
        const uint8_t b2 = SPI.transfer(0x00);

        const uint16_t value = (b0 & 0x01) << 11 | (b1 & 0xFF) << 3 | (b2 & 0xE0) >> 5;
        this->_channels[channelIndex].lastSample = (value & 0x0FFF);
        digitalWrite(_spiPinCS, HIGH);
        delayMicroseconds(1); // MCP_ADC Tcsh = 500ns
    }

    SPI.endTransaction();
}
