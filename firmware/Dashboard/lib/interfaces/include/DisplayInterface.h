#ifndef DISPLAY_INTERFACE_H
#define DISPLAY_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "bitmaps.h"
#include "HT_SPI.h"
#include "alysa_frames.h"
#include "HT_SharpMem.h"
#include <Adafruit_GFX.h>

// Fonts
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

// Defines
#define LED_PIN PA3
#define SHARP_CS PB4
#define SHARP_CLK PB10
#define SHARP_MOSI PB15


namespace conversions
{
    // MPH conversion
    const float GEARBOX_RATIO = 11.86;                                                     // TODO: Need to update
    const float WHEEL_DIAMETER = 0.4064;                                                   // meters NEED to update
    const float RPM_TO_METERS_PER_SECOND = WHEEL_DIAMETER * 3.1415 / GEARBOX_RATIO / 60.0; // TODO: Need to update
    const float METERS_PER_SECOND_TO_RPM = 1.0 / RPM_TO_METERS_PER_SECOND;
    const float METERS_PER_SECOND_TO_MPH = 2.2369;
}

enum StartupAnimations_e
{
    NONE,
    MIKHAIL_CAT,
    DAVID_KNIGHT_GLIZZY,
    DAVID_KNIGHT_2,
    ICE_SPICE
};

class DisplayInterface
{
public:

    DisplayInterface(uint8_t cs) : _display(cs, _DISPLAY_WIDTH, _DISPLAY_HEIGHT) {};

    void init(SPI_HandleTypeDef *hspi);

    /* Start-Up Animations */
    void startup();
    void hytech_animation();
    void alysa_animation();

    // void driver_animation(StartupAnimations);

    /* Background and Layout */
    void draw_background();
    void clear_display_buffer() { _display.clear_display_buffer(); }
    void send_display_buffer(SPI_HandleTypeDef *hspi);
    void set_cursor(int x, int y);
    void invert_display(bool invert_criteria);

    /* Data Display */
    void draw_vertical_pedal_bar(float val, int initial_x_coord);
    void draw_battery_bar(int percent);
    void display_speeds(float rpm);
    void display_mode(int mode);
    void display_min_cell(float min_cell_voltage);
    void display_all_temps(veh_vec<int> temps);
    void display_max_temps(int inverter_temp, int motor_temp);

    /* Status Icons */
    void draw_icons(uint8_t vn_status, VehicleState_e car_state, bool db_in_ctrl);
    void draw_popup(String title);

private:

    static constexpr uint16_t _DISPLAY_WIDTH  = 320;
    static constexpr uint16_t _DISPLAY_HEIGHT = 240;

    HyTech_SharpMem _display; // bigger display is 320x240 smaller one is 400x240
    bool _last_blink = false;
    uint32_t _last_blink_millis = 0;
    uint16_t _black = 0x00;
    uint16_t _white = 0xFF;

    /* SPI Sending */
    uint8_t vcom = SHARPMEM_BIT_VCOM; // VCOM toggle command
    SPI_HandleTypeDef *_hspi = NULL;
    HAL_StatusTypeDef _spi_status;

    void _draw_rectangle_right_corner(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    String _twoDigits(int number);

    bool _blink();

};

using DisplayInterfaceInstance = etl::singleton<DisplayInterface>;

#endif /* LCDINTERFACE_H */