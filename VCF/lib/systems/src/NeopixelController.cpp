#include "NeopixelController.h"


void NeopixelController::init_neopixels()
{
    _neopixels.begin();
    _neopixels.setBrightness(_current_brightness);

    //set init color for every led
    for (int i = 0; i < _neopixel_count; i++)
    {
        // BMS and IMD are off according to rules
        if (i == LED_ID_e::BMS || i == LED_ID_e::IMD || i == LED_ID_e::BMS_WING || i == LED_ID_e::IMD_WING)
        {
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::OFF);
        }
        else
        {
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::INIT_COLOR);
        }
    }
    // write data to neopixels
    _neopixels.show();
}

void NeopixelController::dim_neopixels()
{
    _current_brightness -= STEP_BRIGHTNESS;
    // set current brightness to 0xFF (255) if less than min brightness - sid :) DO NOT CHANGE
    if (_current_brightness < MIN_BRIGHTNESS) { _current_brightness |= 0xFF; } // NOLINT (bitmask with 255)
    _neopixels.setBrightness(_current_brightness);
}

void NeopixelController::set_neopixel(uint16_t id, uint32_t c)
{
    _neopixels.setPixelColor(id, c);
}

void NeopixelController::refresh_neopixels(const PedalsSystemData_s &pedals_data, CANInterfaces_s &interfaces)
{
    // If we are in pedals recalibration state, LIGHT UP DASHBOARD ALL RED.
    if (interfaces.vcr_interface.is_in_pedals_calibration_state() || interfaces.vcr_interface.is_in_steering_calibration_state())
    {
        set_neopixel_color(LED_ID_e::BRAKE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::TORQUE_MODE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::LATCH, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::CRIT_CHARGE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::SHUTDOWN, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::IMPLAUSE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::PACK, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::INVERTER_ERR, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::RDY_DRIVE, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::GLV, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::INVERTER_ERR_WING, LED_color_e::RED);
        set_neopixel_color(LED_ID_e::LATCH_WING, LED_color_e::RED);
        _neopixels.show();
        return;
    }

    LED_color_e brake_light_color = LED_color_e::OFF;
    if (pedals_data.brake_is_pressed && !pedals_data.implausibility_has_exceeded_max_duration)
    {
        brake_light_color = LED_color_e::GREEN;
    }
    else if (pedals_data.implausibility_has_exceeded_max_duration)
    {
        brake_light_color = LED_color_e::RED;
    }

    LED_color_e pack_color = LED_color_e::OFF;
    if (interfaces.acu_interface.get_cell_voltage() > _min_cell_thresholds.max_level)
    {
        pack_color = LED_color_e::PURPLE;
    }
    else if (interfaces.acu_interface.get_cell_voltage() > _min_cell_thresholds.second_level)
    {
        pack_color = LED_color_e::BLUE;
    }
    else if (interfaces.acu_interface.get_cell_voltage() > _min_cell_thresholds.third_level)
    {
        pack_color = LED_color_e::GREEN;
    }
    else if (interfaces.acu_interface.get_cell_voltage() > _min_cell_thresholds.fourth_level)
    {
        pack_color = LED_color_e::YELLOW;
    }
    else if (interfaces.acu_interface.get_cell_voltage() > _min_cell_thresholds.fifth_level)
    {
        pack_color = LED_color_e::ORANGE;
    }
    else if (interfaces.acu_interface.get_cell_voltage() < _min_cell_thresholds.critical_charge_level)
    {
        pack_color = LED_color_e::RED;
    }

    LED_color_e torque_mode_color = LED_color_e::OFF;
    switch (interfaces.vcr_interface.get_torque_limit_mode())
    {
        case TorqueLimit_e::TCMUX_LOW_TORQUE:
        {
            torque_mode_color = LED_color_e::RED;
            break;
        }
        case TorqueLimit_e::TCMUX_MID_TORQUE:
        {
            torque_mode_color = LED_color_e::YELLOW;
            break;
        }
        case TorqueLimit_e::TCMUX_FULL_TORQUE:
        {
            torque_mode_color = LED_color_e::GREEN;
            break;
        }
        default:
        {
            torque_mode_color = LED_color_e::OFF;
            break;
        }
    }

    LED_color_e ready_drive_color = LED_color_e::OFF;
    switch (interfaces.vcr_interface.get_vehicle_state())
    {
        case VehicleState_e::READY_TO_DRIVE:
        {
            interfaces.vcr_interface.get_db_in_ctrl() ? ready_drive_color = LED_color_e::BLUE : ready_drive_color = LED_color_e::GREEN;
            break;
        }
        case VehicleState_e::WANTING_READY_TO_DRIVE:
        {
            ready_drive_color = LED_color_e::YELLOW;
            break;
        }
        default:
        {
            ready_drive_color = LED_color_e::RED;
            break;
        }
    }

    bool hv_present = interfaces.vcr_interface.get_dc_bus_voltage().voltage.FL > _hv_threshold_voltage ||
                      interfaces.vcr_interface.get_dc_bus_voltage().voltage.FR > _hv_threshold_voltage ||
                      interfaces.vcr_interface.get_dc_bus_voltage().voltage.RL > _hv_threshold_voltage ||
                      interfaces.vcr_interface.get_dc_bus_voltage().voltage.RR > _hv_threshold_voltage;


    constexpr float glv_critical_voltage = 22.0f;

    /* SHUTDOWN LEDS */
    set_neopixel_color(LED_ID_e::LATCH, hv_present ? LED_color_e::PURPLE : LED_color_e::GREEN);
    set_neopixel_color(LED_ID_e::IMD, interfaces.dash_interface.imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::BMS, interfaces.dash_interface.bms_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::SHUTDOWN, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::IMD_WING, interfaces.dash_interface.imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::BMS_WING, interfaces.dash_interface.bms_ok ? LED_color_e::GREEN : LED_color_e::RED);

    /* DRIVETRAIN LEDS */
    set_neopixel_color(LED_ID_e::BRAKE, brake_light_color);
    set_neopixel_color(LED_ID_e::INVERTER_ERR, interfaces.vcr_interface.get_inverter_error() ? LED_color_e::RED : LED_color_e::GREEN);
    set_neopixel_color(LED_ID_e::RDY_DRIVE, ready_drive_color);
    set_neopixel_color(LED_ID_e::TORQUE_MODE, torque_mode_color);
    set_neopixel_color(LED_ID_e::IMPLAUSE, pedals_data.brake_and_accel_pressed_implausibility_high ? LED_color_e::RED : LED_color_e::GREEN); // Unused for now

    set_neopixel_color(LED_ID_e::LATCH_WING, hv_present ? LED_color_e::PURPLE : LED_color_e::GREEN); // Unused for now
    set_neopixel_color(LED_ID_e::INVERTER_ERR_WING, interfaces.vcr_interface.get_inverter_error() ? LED_color_e::RED : LED_color_e::GREEN);

    /* VOLTAGE MONITOR */
    set_neopixel_color(LED_ID_e::PACK, pack_color); // Unused for now
    set_neopixel_color(LED_ID_e::CRIT_CHARGE, LED_color_e::OFF); // Unused for now
    set_neopixel_color(LED_ID_e::GLV, LED_color_e::OFF); // No sensor there yet


    _neopixels.show();
}

void NeopixelController::set_neopixel_color(LED_ID_e led, LED_color_e color)
{
    _neopixels.setPixelColor(led, (uint32_t) color);
}