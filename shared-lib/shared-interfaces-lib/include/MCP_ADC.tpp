#include "MCP_ADC.h"
#include <SPI.h>
#include <array>

template <int MCP_ADC_NUM_CHANNELS>
MCP_ADC<MCP_ADC_NUM_CHANNELS>::MCP_ADC(const int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int spiSpeed, const float scales[MCP_ADC_NUM_CHANNELS], const float offsets[MCP_ADC_NUM_CHANNELS])
: _spiPinCS(spiPinCS)
, _spiPinSDI(spiPinSDI)
, _spiPinSDO(spiPinSDO)
, _spiPinCLK(spiPinCLK)
, _spiSpeed(spiSpeed)
{
    for (int i = 0; i < MCP_ADC_NUM_CHANNELS; i++)
    {
        MCP_ADC<MCP_ADC_NUM_CHANNELS>::_channels[i] = AnalogChannel();
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
    // uint16_t command = (
    //     (0b1 << 15) |    // start bit
    //     (0b1 << 14)      // single ended mode
    // );
    byte command, b0, b1, b2;

    // initialize SPI bus. REQUIRED: call SPI.begin() before this
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));

    for (int channelIndex = 0; channelIndex < MCP_ADC_NUM_CHANNELS; channelIndex++)
    {
        digitalWrite(_spiPinCS, LOW);
        command = ((0x01 << 7) |                    // start bit
                   (0x01 << 6) |                    // single or differential
                   ((channelIndex & 0x07) << 3));   // channel number
        b0 = SPI.transfer(command);
        b1 = SPI.transfer(0x00);
        b2 = SPI.transfer(0x00);

        // uint16_t value = SPI.transfer16(command | channelIndex << 11);
        uint16_t value = (b0 & 0x01) << 11 | (b1 & 0xFF) << 3 | (b2 & 0xE0) >> 5;
        MCP_ADC<MCP_ADC_NUM_CHANNELS>::_channels[channelIndex].lastSample = (value & 0x0FFF);
        digitalWrite(_spiPinCS, HIGH);
        delayMicroseconds(1); // MCP_ADC Tcsh = 500ns
    }
    
    SPI.endTransaction();
}