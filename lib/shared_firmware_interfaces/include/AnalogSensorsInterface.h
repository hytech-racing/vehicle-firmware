#ifndef ANALOGSENSORSINTERFACE
#define ANALOGSENSORSINTERFACE

#include <algorithm>
#include "SharedFirmwareTypes.h"

/**
 * The AnalogChannel class represents one individual "channel" of an ADC. Each Channel
 * has its own scale, offsets, and clamp. An AnalogMultiSensor has several AnalogChannels.
 * To use an AnalogChannel properly, you need to first sample the data and place the raw
 * conversion into lastSample. Then, you can call convert() to retrieve the data.
 */
class AnalogChannel
{
public:
    /* Data */
    float scale;
    float offset;
    bool clamp;
    float clampLow;
    float clampHigh;
    int lastSample;

    /* Constructors */
    AnalogChannel(float scale, float offset, bool clamp, float clampLow, float clampHigh)
    : scale(scale),
      offset(offset),
      clamp(clamp),
      clampLow(clampLow),
      clampHigh(clampHigh) {}
    AnalogChannel(float scale, float offset)
    : AnalogChannel(scale, offset, false, __FLT_MIN__, __FLT_MAX__) {}
    AnalogChannel()
    : AnalogChannel(1.0, 0.0, false, __FLT_MIN__, __FLT_MAX__) {}
    
    /* Functions*/
    /**
     * Calculates sensor output and whether result is in the sensor's defined bounds. This DOES NOT SAMPLE.
     * @return This AnalogChannel's output AnalogConversion_s (raw reading, real value, and status).
     */
    AnalogConversion_s convert()
    {
        float conversion = (lastSample* scale) + offset;
        float clampedConversion = std::min(std::max(conversion, clampLow), clampHigh);
        AnalogSensorStatus_e returnStatus = AnalogSensorStatus_e::ANALOG_SENSOR_GOOD;
        if (clamp && (conversion > clampHigh || conversion < clampLow))
            returnStatus = AnalogSensorStatus_e::ANALOG_SENSOR_CLAMPED;
        return {
            lastSample,
            clamp ? clampedConversion : conversion,
            returnStatus
        };
    }
};

/**
 * The AnalogMultiSensor is the base class for specific analog sensor interfaces. Each specific
 * analog sensor (the MCP, Teensy, etc) are children of this class. To use this class, you need
 * to call sample() and convert().
 */
template <int N>
class AnalogMultiSensor
{
protected:

    /**
     * Performs unit conversions on all channels.
     * @post The data field will be updated with the new conversions of each channel.
     */
    void _convert()
    {
        for (int i = 0; i < N; i++)
        {
            data.conversions[i] = _channels[i].convert();
        }
    }

    /**
     * Commands the underlying device to sample all channels and internall store the results.
     * @post The lastSample field of each AnalogChannel in channels_ must contain the new value.
     */
    virtual void _sample();

protected:
    AnalogChannel _channels[N];
public:
    AnalogConversionPacket_s<N> data;

    /* Functions */

    /**
     * The tick() function in each subclass should call sample() and then convert() to update the data field.
     */
    virtual void tick();

    /**
     * Used by systems to retrieve the data field.
     */
    const AnalogConversionPacket_s<N>& get()
    {
        return data;
    }

    /**
     * Sets the scale of the internal conversion channel.
     */
    void setChannelScale(int channel, float scale)
    {
        if (channel < N)
            _channels[channel].scale = scale;
    }

    /**
     * Sets the offset of the internal conversion channel.
     */
    void setChannelOffset(int channel, float offset)
    {
        if (channel < N)
            _channels[channel].offset = offset;
    }

    /**
     * Sets the clamp of the internal conversion channel. The min and max values (to clamp to)
     * are in actual (post-conversion) units. Calling this function will also set the channel's
     * clamp boolean to true to enable clamping. There is no way to programmatically disable clamping.
     */
    void setChannelClamp(int channel, float clampLow, float clampHigh)
    {
        if (channel < N) {
            _channels[channel].clampLow = clampLow;
            _channels[channel].clampHigh = clampHigh;
            _channels[channel].clamp = true;
        }
    }

    /**
     * Sets both the scale and offset of the given channel by calling setChannelOffset and setChannelClamp.
     */
    void setChannelScaleAndOffset(int channel, float scale, float offset)
    {
        setChannelScale(channel, scale);
        setChannelOffset(channel, offset);
    }


};

#endif /* ANALOGSENSORSINTERFACE */