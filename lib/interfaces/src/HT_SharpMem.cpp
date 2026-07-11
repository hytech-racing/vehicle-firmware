#include "HT_SharpMem.h"

#define TOGGLE_VCOM                                                        \
    do {                                                                   \
        _sharpmem_vcom = _sharpmem_vcom ? 0x00 : SHARPMEM_BIT_VCOM;        \
    } while (0);

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                \
    {                                                                      \
        int16_t t = a;                                                     \
        a = b;                                                             \
        b = t;                                                             \
    }
#endif


/**
 * @brief Construct a new Adafruit_SharpMem object with software SPI
 *
 * @param clk The clock pin
 * @param mosi The MOSI pin
 * @param cs The display chip select pin - **NOTE** this is ACTIVE HIGH!
 * @param width The display width
 * @param height The display height
 * @param frequency The SPI clock frequency desired (unlikely to be that fast in soft
 * spi mode!)
 */
HyTech_SharpMem::HyTech_SharpMem(uint8_t cs,
                                uint16_t width,
                                uint16_t height,
                                uint32_t frequency
) : Adafruit_GFX(width, height)
{
    //_stm_spi = stm_spi;
    _cs = cs;
    // _heightt = 240;
    // _widthh = 320;
}

/**
 * @brief Start the driver object, setting up pins and configuring a buffer for
 * the screen contents
 *
 * @return boolean true: success false: failure
 */
bool HyTech_SharpMem::begin(void)
{
    // this display is weird in that _cs is active HIGH not LOW like every other SPI device
    digitalWrite(_cs, LOW);

    // Set the vcom bit to a defined state
    _sharpmem_vcom = SHARPMEM_BIT_VCOM;

    // sharpmem_buffer = (uint8_t *)malloc(((_display_width * _display_height) / 8) + (2*_display_height));
    // create a buffer that is the size of the display (bytes) + the 2 extra pixels on edge

    // _size_of_buffer =  ((_display_width * _display_height) / 8) + (2*_display_height); // in bytes

    int bytes_per_line = (42);

    // if (!sharpmem_buffer)
    //   return false;

    // setRotation(0);
    for (int i = 0; i < _size_of_buffer; i++)
    {
        _display_buffer[i] = 0xFF;
    }

    for (int i = 41; i < _size_of_buffer; i += bytes_per_line)
    {
        _display_buffer[i] = 0x00; // 00s at the end of the line
    }

    for (int i = 0; i < _size_of_buffer; i += bytes_per_line)
    {
        // save address byte
        _display_buffer[i] = ((i) / (bytes_per_line)) + 1; // line is i divided by bytes bytes per line + 1
    }

  return true;
}

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t PROGMEM set[] = {1, 2, 4, 8, 16, 32, 64, 128},
                             clr[] = {(uint8_t)~1,  (uint8_t)~2,  (uint8_t)~4,
                                      (uint8_t)~8,  (uint8_t)~16, (uint8_t)~32,
                                      (uint8_t)~64, (uint8_t)~128};

/**
 * @brief Draws a single pixel in image buffer is adjusted to be able to send one continuous message with DMA
 * @param x The x position (0 based)
 * @param y The y position (0 based)
 * @param color The color to set:
 *              **0**: BlacK
 *              **1**: White
*/
void HyTech_SharpMem::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (x >= _display_width) || (y < 0) || (y >= _display_height))
    {
        return;
    }

    switch (rotation)
    {
        case 1:
        {
            _swap_int16_t(x, y);
            x = _display_width - 1 - x;
            break;
        }
        case 2:
        {
            x = _width - 1 - x;
            y = _height - 1 - y;
            break;
        }
        case 3:
        {
            _swap_int16_t(x, y);
            y = _height - 1 - y;
            break;
        }
    }

    if (color)
    {
        _display_buffer[(y * 42) + (x / 8) + 1] |= pgm_read_byte(&set[x & 7]);
    }
    else
    {
        _display_buffer[(y * 42) + (x / 8) + 1] &= pgm_read_byte(&clr[x & 7]);
    }
}

void HyTech_SharpMem::clear_display_buffer()
{
    for (int y = 0; y < _height; y++)
    {
        for (int x = 0; x < _width; x++)
        {
            _display_buffer[(y * 42) + (x / 8) + 1] = 0xFF;
        }
    }
}

