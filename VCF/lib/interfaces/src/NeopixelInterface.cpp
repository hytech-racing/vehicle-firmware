#include "NeopixelInterface.hpp"


void NeopixelInterface::init()
{
    _neopixels.begin();
}

void NeopixelInterface::setPixelColor(uint16_t id, uint32_t color)
{
    _neopixels.setPixelColor(id, color);
}

void NeopixelInterface::setBrightness(uint8_t brightness)
{
    _neopixels.setBrightness(brightness);
}

void NeopixelInterface::show()
{
    _neopixels.show();
}