#ifndef ANALOG_SENSORS_INTERFACE_HPP
#define ANALOG_SENSORS_INTERFACE_HPP

#include "SharedFirmwareTypes.h"

#include <algorithm>

/**
 * The AnalogChannel class represents one individual "channel" of an ADC. Each Channel
 * has its own scale, offsets, and clamp. An AnalogMultiSensor has several AnalogChannels.
 * To use an AnalogChannel properly, you need to first sample the data and place the raw
 * conversion into lastSample. Then, you can call convert() to retrieve the data.
 */
class AnalogChannel
{
  public:
    // DATA
    float scale;
    float offset;
    bool isChannelClamped;
    float clampLow;
    float clampHigh;
    int lastSample;

    // CONSTRUCTORS
    AnalogChannel(float scale, float offset, bool isChannelClamped, float clampLow, float clampHigh)
        : scale(scale), offset(offset), isChannelClamped(isChannelClamped), clampLow(clampLow), clampHigh(clampHigh),
          lastSample(0)
    {
    }
    AnalogChannel(float scale, float offset)
        : AnalogChannel(scale, offset, false, -__FLT_MAX__, __FLT_MAX__)
    {
    }
    AnalogChannel()
        : AnalogChannel(1.0f, 0.0f)
    {
    }

    // FUNCTIONS
    /**
     * @brief Calculates sensor output and whether result is in the sensor's defined bounds. This DOES NOT SAMPLE.
     * @return This AnalogChannel's output AnalogConversion_s (raw reading, real value, and status).
     */
    AnalogConversion_s convert()
    {
        float conversion = (lastSample * scale) + offset;
        float clampedConversion = std::min(std::max(conversion, clampLow), clampHigh);
        AnalogSensorStatus_e returnStatus = AnalogSensorStatus_e::ANALOG_SENSOR_GOOD;
        if (isChannelClamped && (conversion > clampHigh || conversion < clampLow))
            returnStatus = AnalogSensorStatus_e::ANALOG_SENSOR_CLAMPED;
        return {lastSample, isChannelClamped ? clampedConversion : conversion, returnStatus};
    }
};

/**
 * The AnalogMultiSensor is the base class for specific analog sensor interfaces. Each specific
 * analog sensor (the MCP, Teensy, etc) are children of this class. Each subclass must implement
 * _sample() and tick(). To use a sensor, call tick() to update its readings, then call get()
 * to retrieve the converted data.
 */
template <int NUM_CHANNELS>
class AnalogMultiSensor
{
  public:
    // DATA
    AnalogConversionPacket_s<NUM_CHANNELS> data;

    // DESTRUCTOR
    virtual ~AnalogMultiSensor() = default;

    // FUNCTIONS
    /**
     * @brief Updates the data field with fresh readings.
     * @note Each subclass's implementation should call _sample() and then _convert().
     */
    virtual void tick() = 0;

    /**
     * @brief Used by systems to retrieve the data field.
     */
    const AnalogConversionPacket_s<NUM_CHANNELS& get() const { return data; }

    /**
     * @brief Sets the scale of the internal conversion channel.
     */
    void setChannelScale(int channel, float scale)
    {
        if (channel >= 0 && channel < NUM_CHANNELS)
            _channels[channel].scale = scale;
    }

    /**
     * @brief Sets the offset of the internal conversion channel.
     */
    void setChannelOffset(int channel, float offset)
    {
        if (channel >= 0 && channel < NUM_CHANNELS)
            _channels[channel].offset = offset;
    }

    /**
     * @brief Sets the clamp of the internal conversion channel and enables clamping on it.
     *
     * Calling this function sets the channel's isChannelClamped flag to true.
     * There is no way to programmatically disable clamping.
     * @note The min and max values (to clamp to) are/should be in actual (post-conversion) units.
     */
    void setChannelClamp(int channel, float clampLow, float clampHigh)
    {
        if (channel >= 0 && channel < NUM_CHANNELS)
        {
            _channels[channel].clampLow = clampLow;
            _channels[channel].clampHigh = clampHigh;
            _channels[channel].isChannelClamped = true;
        }
    }

    /**
     * @brief Sets both the scale and offset of the given channel by calling setChannelScale and setChannelOffset.
     */
    void setChannelScaleAndOffset(int channel, float scale, float offset)
    {
        setChannelScale(channel, scale);
        setChannelOffset(channel, offset);
    }

  protected:
    // DATA
    AnalogChannel _channels[NUM_CHANNELS];

    // FUNCTIONS
    /**
     * @brief Performs unit conversions on all channels.
     * @post The data field will be updated with the new conversions of each channel.
     */
    void _convert()
    {
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            data.conversions[i] = _channels[i].convert();
        }
    }

    /**
     * @brief Commands the underlying device to sample all channels and internally store the results.
     * @post The lastSample field of each AnalogChannel in _channels must contain the new value.
     */
    virtual void _sample() = 0;
};

#endif // ANALOG_SENSORS_INTERFACE_HPP