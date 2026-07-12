#ifndef __MCP_ADC_H__
#define __MCP_ADC_H__

#include "SPI.h"
#include "AnalogSensorsInterface.h"

// Definitions
const int MCP_ADC_DEFAULT_SPI_SDI   = 12;
const int MCP_ADC_DEFAULT_SPI_SDO   = 11;
const int MCP_ADC_DEFAULT_SPI_CLK   = 13;
const int MCP_ADC_DEFAULT_SPI_SPEED = 2000000;

/**
 * The MCP_ADC is a concrete subclass of the AnalogMultiSensor parent class. This allows
 * for SPI communication with the MCP_ADC, which can be a 4-channel or 8-channel sensor.
 * IMPORTANT - must call SPI.begin() once and only once before instantiating any MCP_ADC object!!
 */
template <int MCP_ADC_NUM_CHANNELS>
class MCP_ADC : public AnalogMultiSensor<MCP_ADC_NUM_CHANNELS>
{
private:
    const int _spiPinCS;
    const int _spiPinSDI;
    const int _spiPinSDO;
    const int _spiPinCLK;
    const int _spiSpeed;

    /**
     * Samples the MCP_ADC over SPI. Samples all eight channels and, in accordance with the AnalogMultiSensor's function
     * contract, stores the raw sampled values into each AnalogChannel's lastSample instance variable.
     */
    void _sample() override;
    
public:
    /* Constructors */
    MCP_ADC(int spiPinCS, const int spiPinSDI, const int spiPinSDO, const int spiPinCLK, const int spiSpeed, const float scales[MCP_ADC_NUM_CHANNELS], const float offsets[MCP_ADC_NUM_CHANNELS]);

    /* Functions */

    /**
     * Calls sample() and convert(). After calling tick(), this MCP_ADC's data can be accessed using the get() command.
     */
    void tick() override;

};

#include "MCP_ADC.tpp"

#endif /* __MCP_ADC_H__ */