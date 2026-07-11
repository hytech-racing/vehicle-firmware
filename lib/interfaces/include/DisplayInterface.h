#ifndef DisplayInterface_H
#define DisplayInterface_H

/* External Includes */
#include "SharedFirmwareTypes.h"
#include <SPI.h>
#include <DMAChannel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

/* Local Interface Includes */
#include "ACUInterface.h"
#include "ButtonInterface.h"
#include "ChargerStateMachine.h"
#include "EMInterface.h"
#include "RotaryEncoderInterface.h"

using pin = uint8_t;


namespace default_display_params
{
    constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 100UL;  // ms
    constexpr unsigned long CYCLE_BUTTON_HOLD_TIME_RESET_MS = 2000UL; // ms
    constexpr unsigned long SLIDING_WINDOW_DISPLAY_INTERVAL_MS = 2500UL;
    constexpr float DATA_SCALAR = 10.0F;
    constexpr uint8_t BYTE_SHIFT = 8;
};

enum DisplayView_e
{
    VIEW_CHARGE_STATUS = 0,
    VIEW_CHARGER,
    VIEW_VOLTAGE,
    VIEW_BOARD_TEMPERATURE,
    VIEW_CELL_TEMPERATURE,
    NUM_VIEWS
};

struct DisplayPinout_s
{
    pin teensy_lcd_cs_pin;
    pin teensy_lcd_sck_pin;
    pin teensy_lcd_miso_pin;
    pin teensy_lcd_mosi_pin;
    pin teensy_lcd_reset_pin;
    pin teensy_lcd_dc_pin;

    pin cycle_display_view_pin;
};

struct DisplayInterfaceParams_s
{
    DisplayPinout_s pinout;
    float display_time;
    float display_update_interval;
};

struct DisplayConfig_s
{
    unsigned long display_update_interval_ms;
    unsigned long cycle_button_hold_time_reset_ms;
    unsigned long last_display_timestamp;
    unsigned long sliding_window_display_interval_ms;
};

class DisplayInterface
{
public:
    DisplayInterface(DisplayPinout_s pinout,
                    DisplayConfig_s config = {
                        .display_update_interval_ms = default_display_params::DISPLAY_UPDATE_INTERVAL_MS,
                        .cycle_button_hold_time_reset_ms = default_display_params::CYCLE_BUTTON_HOLD_TIME_RESET_MS,
                        .last_display_timestamp = 0,
                        .sliding_window_display_interval_ms = default_display_params::SLIDING_WINDOW_DISPLAY_INTERVAL_MS
                    }
    ) : Display(
            pinout.teensy_lcd_cs_pin,
            pinout.teensy_lcd_dc_pin,
            pinout.teensy_lcd_mosi_pin,
            pinout.teensy_lcd_sck_pin,
            pinout.teensy_lcd_reset_pin,
            pinout.teensy_lcd_miso_pin
        ),
        _pinout(pinout),
        _config(config),
        _display_time(0),
        _cycle_display_view_button(pinout.cycle_display_view_pin),
        _display_view(DisplayView_e::VIEW_CHARGE_STATUS)
    {};

    void init();

    void display_data(unsigned long current_millis, bool is_120_switched);

    void refresh_display_data(unsigned long curr_millis);

    void update(unsigned long current_millis);

    void handle_button_events(unsigned long current_millis);

    void cycle_view();

    Adafruit_ILI9341 Display;

private:

    DisplayPinout_s _pinout;
    DisplayConfig_s _config;
    unsigned long _display_time;
    ButtonInterface _cycle_display_view_button;
    DisplayView_e _display_view;

};

using DisplayInterfaceInstance = etl::singleton<DisplayInterface>;

#endif