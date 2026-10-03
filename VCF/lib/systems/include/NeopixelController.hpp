#ifndef NEOPIXEL_CONTROLLER_H
#define NEOPIXEL_CONTROLLER_H

/* Neopixel Controller Defines */
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 3
#define BRIGHTNESS_STEPS 4
#define STEP_BRIGHTNESS ((MAX_BRIGHTNESS - MIN_BRIGHTNESS) / BRIGHTNESS_STEPS)

/* ETL Library Includes */
#include <etl/singleton.h>
#include <etl/delegate.h>
#include "SharedFirmwareTypes.h"


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

struct NeopixelControllerInputs_s
{
    bool are_pedals_calibrating;
    bool is_steering_calibrating;
    TorqueLimit_e torque_limit_mode;
    VehicleState_e vehicle_state;
    bool is_drivebrain_in_control;
    veh_vec<int> bus_voltages;
    bool is_inverter_errored;
    bool is_imd_ok;
    bool is_bms_ok;
    float min_cell_voltage;
};

class NeopixelController
{
public:

    /**
     * @param neopixel_interface Strip hardware the controller writes colors to (not owned; must outlive the controller)
     * @param neopixel_count Number of LEDs on the strip
     */
    NeopixelController(etl::delegate<void(uint16_t, uint32_t)> setPixelColor,
                    etl::delegate<void(uint8_t)> setBrightness,
                    etl::delegate<void()> show,
                    NeopixelControllerInputs_s controller_inputs,
                    uint32_t neopixel_count
    ) : _setPixelColor(setPixelColor),
        _setBrightness(setBrightness),
        _show(show),
        _controller_inputs(controller_inputs),
        _current_brightness(64),
        _neopixel_count(neopixel_count)
    {};

    void init();

    void dimNeopixels();

    void setNeopixel(uint16_t id, uint32_t c);

    void refreshNeopixels(const PedalsSystemData_s &pedals_data);

    void setNeopixelColor(LED_ID_e led, LED_color_e color);

private:

    etl::delegate<void(uint16_t, uint32_t)> _setPixelColor;
    etl::delegate<void(uint8_t)> _setBrightness;
    etl::delegate<void()> _show;
    NeopixelControllerInputs_s _controller_inputs;
    uint8_t _current_brightness;
    uint8_t _neopixel_count;
    const uint8_t _hv_threshold_voltage = 60;
    MinCellMonitoringThresholds_s _min_cell_thresholds;

};

using NeopixelControllerInstance = etl::singleton<NeopixelController>;

#endif /* NEOPIXEL_CONTROLLER_H */