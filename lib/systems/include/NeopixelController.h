#ifndef NEOPIXEL_CONTROLLER_H
#define NEOPIXEL_CONTROLLER_H

/* Neopixel Controller Defines */
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 3
#define BRIGHTNESS_STEPS 4
#define STEP_BRIGHTNESS ((MAX_BRIGHTNESS - MIN_BRIGHTNESS) / BRIGHTNESS_STEPS)
// Note from Justin: I know that this sort of breaks the paradigm that we've put
// in place for most of our code, but
// - other libraries do this all the time
// - I don't really see these as externally configurable, so I don't see why we should
// define a new type of struct, add an arg to the constructor, etc.
// - this is how it was implemented on STM32 dash and I want to be fast :)

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include <Adafruit_NeoPixel.h>
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "VCFCANInterfaceImpl.h"


struct MinCellMonitoringThresholds_s
{
    float max_level = 3.85;
    float second_level = 3.7;
    float third_level = 3.65;
    float fourth_level = 3.6;
    float fifth_level = 3.5;
    float critical_charge_level = 3.4;
};

enum LED_ID_e
{
    INVERTER_ERR_WING = 0,
    BMS_WING = 1,
    SHUTDOWN = 2,
    INVERTER_ERR = 3,
    TORQUE_MODE = 4,
    BRAKE = 5,
    BMS = 6,
    GLV = 7,
    PACK = 8,
    IMD = 9,
    IMPLAUSE = 10,
    RDY_DRIVE = 11,
    LATCH = 12,
    CRIT_CHARGE = 13,
    LATCH_WING= 14,
    IMD_WING = 15
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

    NeopixelController(uint32_t neopixel_count,
                    uint32_t neopixel_pin
    ) : _neopixels(neopixel_count, neopixel_pin, NEO_GRBW + NEO_KHZ800),
        _current_brightness(64),
        _neopixel_count(neopixel_count)
    {};

    void init_neopixels();

    void dim_neopixels();

    void set_neopixel(uint16_t id, uint32_t c);

    void refresh_neopixels(const PedalsSystemData_s &pedals_data, CANInterfaces_s &interfaces);

    void set_neopixel_color(LED_ID_e led, LED_color_e color);

private:

    Adafruit_NeoPixel _neopixels;
    uint8_t _current_brightness;
    uint8_t _neopixel_count;
    const uint8_t _hv_threshold_voltage = 60;
    MinCellMonitoringThresholds_s _min_cell_thresholds;
    
};

using NeopixelControllerInstance = etl::singleton<NeopixelController>;

#endif /* NEOPIXEL_CONTROLLER_H */