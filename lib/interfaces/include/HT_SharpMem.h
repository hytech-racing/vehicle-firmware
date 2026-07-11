#ifndef HT_SHARPMEM_H
#define HT_SHARPMEM_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Adafruit_GFX.h>
#include <Arduino.h>

#define SHARPMEM_BIT_WRITECMD (0x01) // 0x80 in LSB format
#define SHARPMEM_BIT_VCOM (0x02)     // 0x40 in LSB format
#define SHARPMEM_BIT_CLEAR (0x04)    // 0x20 in LSB format

/**
 * @brief Class to control the Sharp memory display
 *
 */
class HyTech_SharpMem : public Adafruit_GFX
{
public:

    HyTech_SharpMem(uint8_t CS_PIN,
                    uint16_t WIDTH = 320,
                    uint16_t HEIGHT = 240,
                    uint32_t FREQUENCY = 2000000
    );

    bool begin();

    /**
     * The reason this specific method doesn't follow our naming convention is that Adafruit_GFX is an
     * abstract base class. Meaning, that we must implement drawPixel. We cannot simply change the name. 
     */
    void drawPixel(int16_t x, int16_t y, uint16_t color);

    void clear_display_buffer();

    /**
     * @brief Get a pointer to the display buffer.
     * This allows direct access to the internal framebuffer.
     *
     * @return uint8_t* Pointer to the framebuffer memory.
     */
    uint8_t *getBuffer() { return _display_buffer; }

    uint16_t getBufferSize() { return _size_of_buffer; }

private:

    uint8_t _display_buffer[10080]; //320*240/8 = 9600 + 480 extra for line addresses
    uint16_t _size_of_buffer = 10080;
    uint16_t _display_width = 320;
    uint16_t _display_height = 240;
    uint8_t _cs;
    uint8_t * _stm_spi;
    uint8_t _sharpmem_vcom;

};

#endif