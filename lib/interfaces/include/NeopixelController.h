#ifndef NEOPIXEL_CONTROLLER_H
#define NEOPIXEL_CONTROLLER_H

/* Neopixel Controller Defines */
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 3
#define BRIGHTNESS_STEPS 4
#define STEP_BRIGHTNESS ((MAX_BRIGHTNESS - MIN_BRIGHTNESS) / BRIGHTNESS_STEPS)
// Note from Justin: I know that this sort of breaks the paradigm that we've put
// in place for most of our code, but other libraries do this all the time
// I don't really see these as externally configurable, so I don't see why we should
// define a new type of struct, add an arg to the constructor, etc.
// this is how it was implemented on STM32 dash and I want to be fast :)

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Adafruit_NeoPixel.h>
#include "SharedFirmwareTypes.h"
#include "DashCANInterfaceImpl.h"

enum LED_ID_e
{
    SHUTDOWN = 0,
    INVERTER_ERR = 1,
    TORQUE_MODE = 2,
    BRAKE = 3,
    BMS = 4,
    GLV = 5,
    PACK = 6,
    IMD = 7,
    IMPLAUSE = 8,
    RDY_DRIVE = 9,
    LATCH = 10,
    CRIT_CHARGE = 11,
    END1 = 12,
    END2 = 13,
    END3 = 14,
    END4 = 15
};

enum class LED_color_e
{
    OFF = 0x00,
    GREEN = 0xFF00,
    YELLOW = 0xFFFF00,
    RED = 0xFF0000,
    INIT_COLOR = 0xFF007F,
    BLUE = 0xFF,
    PURPLE = 0x703fab,
    ORANGE = 0xf5a742,
};

class NeopixelController
{
public:

    NeopixelController() = delete;

    NeopixelController(uint32_t neopixel_count,
                    uint32_t neopixel_pin
    ) : _neopixels(neopixel_count, neopixel_pin, NEO_GRBW + NEO_KHZ800),
        _current_brightness(50),
        _neopixel_count(neopixel_count)
    {};

    void init_neopixels();

    void dim_neopixels();

    void refresh_neopixels(CANInterfaces_s &interfaces);

    void set_neopixel_color(LED_ID_e led, LED_color_e color);

private:

    Adafruit_NeoPixel _neopixels;
    uint8_t _current_brightness;
    uint8_t _neopixel_count;

};

using NeopixelControllerInstance = etl::singleton<NeopixelController>;

#endif /* NEOPIXEL_CONTROLLER_H */