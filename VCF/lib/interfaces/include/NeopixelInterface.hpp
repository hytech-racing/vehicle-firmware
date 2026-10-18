#ifndef NEOPIXEL_INTERFACE_H
#define NEOPIXEL_INTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include <Adafruit_NeoPixel.h>


class NeopixelInterface
{
public:

    NeopixelInterface(uint16_t neopixel_count,
                    int16_t neopixel_pin
    ) : _neopixels(neopixel_count, neopixel_pin, NEO_GRBW + NEO_KHZ800)
    {}

    /**
     * @brief Initializes the neopixels hardware. Call once at startup.
    */
    void init();

    /**
     * @brief Wrapper for the Adafruit method
    */
    void setPixelColor(uint16_t id, uint32_t color);

    /**
     * @brief Wrapper for the Adafruit method
    */
    void setBrightness(uint8_t brightness);

    /**
     * @brief Wrapper for the Adafruit method
    */
    void show();

private:

    Adafruit_NeoPixel _neopixels;

};

using NeopixelInterfaceInstance = etl::singleton<NeopixelInterface>;

#endif /* NEOPIXEL_INTERFACE_H */