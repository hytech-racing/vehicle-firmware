#include "NeopixelController.hpp"


void NeopixelController::init()
{
    _setBrightness(_current_brightness);

    for (int i = 0; i < _neopixel_count; i++)
    {
        if (i == LED_ID_e::BMS || i == LED_ID_e::IMD || i == LED_ID_e::BMS_WING || i == LED_ID_e::IMD_WING)
        {
            _setPixelColor(i, (uint32_t) LED_color_e::OFF);
        }
        else
        {
            _setPixelColor(i, (uint32_t) LED_color_e::INIT_COLOR);
        }
    }
    _show();
}

void NeopixelController::dimNeopixels()
{
    if (_current_brightness < MIN_BRIGHTNESS + STEP_BRIGHTNESS)
    {
        _current_brightness = MAX_BRIGHTNESS;
    }
    else
    {
        _current_brightness = static_cast<uint8_t>(_current_brightness - STEP_BRIGHTNESS);
    }
    _setBrightness(_current_brightness);
}

void NeopixelController::setNeopixel(uint16_t id, uint32_t c)
{
    _setPixelColor(id, c);
}

void NeopixelController::refreshNeopixels(const PedalsSystemData_s &pedals_data)
{

    if (_controller_inputs.are_pedals_calibrating || _controller_inputs.is_steering_calibrating)
    {
        setNeopixelColor(LED_ID_e::BRAKE, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::TORQUE_MODE, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::LATCH, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::CRIT_CHARGE, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::SHUTDOWN, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::IMPLAUSE, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::PACK, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::INVERTER_ERR, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::RDY_DRIVE, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::GLV, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::INVERTER_ERR_WING, LED_color_e::RED);
        setNeopixelColor(LED_ID_e::LATCH_WING, LED_color_e::RED);
        _show();
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
    if (_controller_inputs.min_cell_voltage > _min_cell_thresholds.max_level)
    {
        pack_color = LED_color_e::PURPLE;
    }
    else if (_controller_inputs.min_cell_voltage > _min_cell_thresholds.second_level)
    {
        pack_color = LED_color_e::BLUE;
    }
    else if (_controller_inputs.min_cell_voltage > _min_cell_thresholds.third_level)
    {
        pack_color = LED_color_e::GREEN;
    }
    else if (_controller_inputs.min_cell_voltage > _min_cell_thresholds.fourth_level)
    {
        pack_color = LED_color_e::YELLOW;
    }
    else if (_controller_inputs.min_cell_voltage > _min_cell_thresholds.fifth_level)
    {
        pack_color = LED_color_e::ORANGE;
    }
    else if (_controller_inputs.min_cell_voltage < _min_cell_thresholds.critical_charge_level)
    {
        pack_color = LED_color_e::RED;
    }

    LED_color_e torque_mode_color = LED_color_e::OFF;
    switch (_controller_inputs.torque_limit_mode)
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
    switch (_controller_inputs.vehicle_state)
    {
        case VehicleState_e::READY_TO_DRIVE:
        {
            _controller_inputs.is_drivebrain_in_control ? ready_drive_color = LED_color_e::BLUE : ready_drive_color = LED_color_e::GREEN;
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

    bool hv_present = _controller_inputs.bus_voltages.FL > _hv_threshold_voltage ||
                      _controller_inputs.bus_voltages.FR > _hv_threshold_voltage ||
                      _controller_inputs.bus_voltages.RL > _hv_threshold_voltage ||
                      _controller_inputs.bus_voltages.RR > _hv_threshold_voltage;

    /* SHUTDOWN LEDS */
    setNeopixelColor(LED_ID_e::LATCH, hv_present ? LED_color_e::PURPLE : LED_color_e::GREEN);
    setNeopixelColor(LED_ID_e::IMD, _controller_inputs.is_imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    setNeopixelColor(LED_ID_e::BMS, _controller_inputs.is_bms_ok ? LED_color_e::GREEN : LED_color_e::RED);
    setNeopixelColor(LED_ID_e::SHUTDOWN, LED_color_e::OFF);
    setNeopixelColor(LED_ID_e::IMD_WING, _controller_inputs.is_imd_ok ? LED_color_e::GREEN : LED_color_e::RED);
    setNeopixelColor(LED_ID_e::BMS_WING, _controller_inputs.is_bms_ok ? LED_color_e::GREEN : LED_color_e::RED);

    /* DRIVETRAIN LEDS */
    setNeopixelColor(LED_ID_e::BRAKE, brake_light_color);
    setNeopixelColor(LED_ID_e::INVERTER_ERR, _controller_inputs.is_inverter_errored ? LED_color_e::RED : LED_color_e::GREEN);
    setNeopixelColor(LED_ID_e::RDY_DRIVE, ready_drive_color);
    setNeopixelColor(LED_ID_e::TORQUE_MODE, torque_mode_color);
    setNeopixelColor(LED_ID_e::IMPLAUSE, pedals_data.brake_and_accel_pressed_implausibility_high ? LED_color_e::RED : LED_color_e::GREEN);

    setNeopixelColor(LED_ID_e::LATCH_WING, hv_present ? LED_color_e::PURPLE : LED_color_e::GREEN);
    setNeopixelColor(LED_ID_e::INVERTER_ERR_WING, _controller_inputs.is_inverter_errored ? LED_color_e::RED : LED_color_e::GREEN);

    /* VOLTAGE MONITOR */
    setNeopixelColor(LED_ID_e::PACK, pack_color);
    setNeopixelColor(LED_ID_e::CRIT_CHARGE, LED_color_e::OFF);
    setNeopixelColor(LED_ID_e::GLV, LED_color_e::OFF);

    _show();
}

void NeopixelController::setNeopixelColor(LED_ID_e led, LED_color_e color)
{
    _setPixelColor(led, (uint32_t) color);
}