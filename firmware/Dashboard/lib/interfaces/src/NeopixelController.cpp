#include "NeopixelController.h"


void NeopixelController::init_neopixels()
{
    _neopixels.begin();
    _neopixels.setBrightness(_current_brightness);

    // set init color for every led
    for (int i = 0; i < _neopixel_count; i++)
    {
        _neopixels.setPixelColor(i, (uint32_t) LED_color_e::INIT_COLOR);
        // BMS and IMD are off according to rules
        if (i == LED_ID_e::BMS || i == LED_ID_e::IMD)
        {
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::GREEN);
        }
    }
    // write data to neopixels
    _neopixels.show();
}

void NeopixelController::dim_neopixels()
{
    _current_brightness -= STEP_BRIGHTNESS;
    // set current brightness to 0xFF (255) if less than min brightness - sid :) DO NOT CHANGE
    if (_current_brightness < MIN_BRIGHTNESS)
    {
        _current_brightness |= 0xFF; // NOLINT (bitmask with 255)
    }

    _neopixels.setBrightness(_current_brightness);
}

//TODO: Update for dash CAN
void NeopixelController::refresh_neopixels(CANInterfaces_s &interfaces)
{

    // // If we are in pedals recalibration state, LIGHT UP DASHBOARD ALL RED.
    if (interfaces.vcr_interface.is_in_pedals_calibration_state())
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

        set_neopixel_color(LED_ID_e::END1, LED_color_e::OFF);
        set_neopixel_color(LED_ID_e::END2, LED_color_e::OFF);
        set_neopixel_color(LED_ID_e::END3, LED_color_e::BLUE);
        set_neopixel_color(LED_ID_e::END4, LED_color_e::BLUE);
        _neopixels.show();
        return;
    }

    LED_color_e brake_light_color = LED_color_e::RED;
    // if (vcf_data.system_data.pedals_system_data.brake_is_pressed && !vcf_data.system_data.pedals_system_data.implausibility_has_exceeded_max_duration) {
    //     brake_light_color = LED_color_e::GREEN;
    // } else if (vcf_data.system_data.pedals_system_data.implausibility_has_exceeded_max_duration) {
    //     brake_light_color = LED_color_e::RED;
    // }

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
    switch (interfaces.vcr_interface.get_curr_car_state())
    {
        case VehicleState_e::READY_TO_DRIVE:
        {
            interfaces.vcr_interface.get_drivebrain_in_control() ? ready_drive_color = LED_color_e::BLUE : ready_drive_color = LED_color_e::GREEN;
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

    constexpr float glv_critical_voltage = 22.0f;

    /* SHUTDOWN LEDS */
    // set_neopixel_color(LED_ID_e::LATCH, LED_color_e::OFF); // Unused for now
    // set_neopixel_color(LED_ID_e::IMD, interfaces.acu_interface.get_curr_data().imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    // set_neopixel_color(LED_ID_e::BMS, interfaces.acu_interface.get_curr_data().bms_ok ? LED_color_e::GREEN : LED_color_e::RED);
    // set_neopixel_color(LED_ID_e::SHUTDOWN, LED_color_e::OFF); // Unused for now

    // /* DRIVETRAIN LEDS */
    // set_neopixel_color(LED_ID_e::BRAKE, brake_light_color);
    // set_neopixel_color(LED_ID_e::INVERTER_ERR, LED_color_e::OFF);
    // set_neopixel_color(LED_ID_e::RDY_DRIVE, ready_drive_color);
    // set_neopixel_color(LED_ID_e::TORQUE_MODE, torque_mode_color);
    // set_neopixel_color(LED_ID_e::IMPLAUSE, LED_color_e::OFF); // Unused for now

    // /* VOLTAGE MONITOR */
    // set_neopixel_color(LED_ID_e::PACK, LED_color_e::OFF); // Unused for now
    // set_neopixel_color(LED_ID_e::CRIT_CHARGE, LED_color_e::OFF); // Unused for now
    // set_neopixel_color(LED_ID_e::GLV, interfaces.vcf_interface.get_control_mode() ? LED_color_e::GREEN : LED_color_e::RED); // No sensor there yet

    // set_neopixel_color(LED_ID_e::END1, LED_color_e::OFF);
    // set_neopixel_color(LED_ID_e::END2, LED_color_e::OFF);
    // set_neopixel_color(LED_ID_e::END3, LED_color_e::BLUE);
    // set_neopixel_color(LED_ID_e::END4, LED_color_e::BLUE);

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

    set_neopixel_color(LED_ID_e::END1, LED_color_e::BLUE);
    set_neopixel_color(LED_ID_e::END2, LED_color_e::BLUE);
    set_neopixel_color(LED_ID_e::END3, LED_color_e::BLUE);
    set_neopixel_color(LED_ID_e::END4, LED_color_e::BLUE);

    set_neopixel_color(LED_ID_e::IMD, interfaces.acu_interface.get_curr_data().imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    set_neopixel_color(LED_ID_e::BMS, interfaces.acu_interface.get_curr_data().bms_ok ? LED_color_e::GREEN : LED_color_e::RED);

    _neopixels.show();

}

void NeopixelController::set_neopixel_color(LED_ID_e led, LED_color_e color)
{
    _neopixels.setPixelColor(led, (uint32_t) color);
}