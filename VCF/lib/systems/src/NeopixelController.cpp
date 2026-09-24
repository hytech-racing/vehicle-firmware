#include "NeopixelController.h"


void NeopixelController::init()
{
    _neopixels.begin();
    _neopixels.setBrightness(_current_brightness);

    for (int i = 0; i < _neopixel_count; i++)
    {
        if (i == LED_ID_e::BMS || i == LED_ID_e::IMD || i == LED_ID_e::BMS_WING || i == LED_ID_e::IMD_WING)
        {
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::OFF);
        }
        else
        {
            _neopixels.setPixelColor(i, (uint32_t) LED_color_e::INIT_COLOR);
        }
    }
    _neopixels.show();
}

void NeopixelController::dimNeopixels()
{
    _current_brightness -= STEP_BRIGHTNESS;
    if (_current_brightness < MIN_BRIGHTNESS) { _current_brightness |= 0xFF; } // NOLINT (bitmask with 255)
    _neopixels.setBrightness(_current_brightness);
}

void NeopixelController::setNeopixel(uint16_t id, uint32_t c)
{
    _neopixels.setPixelColor(id, c);
}

void NeopixelController::refreshNeopixels(const PedalsSystemData_s &pedals_data, CANInterfaces_s &interfaces)
{
    auto& acu_int = interfaces.acu_interface;
    auto& vcr_int = interfaces.vcr_interface;

    if (vcr_int.arePedalsCalibrating() || vcr_int.isSteeringCalibrating())
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
    if (acu_int.getMinCellVoltage() > _min_cell_thresholds.max_level)
    {
        pack_color = LED_color_e::PURPLE;
    }
    else if (acu_int.getMinCellVoltage() > _min_cell_thresholds.second_level)
    {
        pack_color = LED_color_e::BLUE;
    }
    else if (acu_int.getMinCellVoltage() > _min_cell_thresholds.third_level)
    {
        pack_color = LED_color_e::GREEN;
    }
    else if (acu_int.getMinCellVoltage() > _min_cell_thresholds.fourth_level)
    {
        pack_color = LED_color_e::YELLOW;
    }
    else if (acu_int.getMinCellVoltage() > _min_cell_thresholds.fifth_level)
    {
        pack_color = LED_color_e::ORANGE;
    }
    else if (interfaces.acu_interface.getMinCellVoltage() < _min_cell_thresholds.critical_charge_level)
    {
        pack_color = LED_color_e::RED;
    }

    LED_color_e torque_mode_color = LED_color_e::OFF;
    switch (vcr_int.getTorqueLimitMode())
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
    switch (vcr_int.getVehicleState())
    {
        case VehicleState_e::READY_TO_DRIVE:
        {
            vcr_int.isDrivebrainInControl() ? ready_drive_color = LED_color_e::BLUE : ready_drive_color = LED_color_e::GREEN;
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

    bool hv_present = vcr_int.getDCBusVoltages().voltage.FL > _hv_threshold_voltage ||
                      vcr_int.getDCBusVoltages().voltage.FR > _hv_threshold_voltage ||
                      vcr_int.getDCBusVoltages().voltage.RL > _hv_threshold_voltage ||
                      vcr_int.getDCBusVoltages().voltage.RR > _hv_threshold_voltage;

    /* SHUTDOWN LEDS */
    setNeopixelColor(LED_ID_e::LATCH, hv_present ? LED_color_e::PURPLE : LED_color_e::GREEN);
    setNeopixelColor(LED_ID_e::IMD, interfaces.dash_interface.isIMDOk() ? LED_color_e::GREEN : LED_color_e::RED);
    setNeopixelColor(LED_ID_e::BMS, interfaces.dash_interface.isBMSOk() ? LED_color_e::GREEN : LED_color_e::RED);
    setNeopixelColor(LED_ID_e::SHUTDOWN, LED_color_e::OFF);
    setNeopixelColor(LED_ID_e::IMD_WING, interfaces.dash_interface.isIMDOk() ? LED_color_e::GREEN : LED_color_e::RED);
    setNeopixelColor(LED_ID_e::BMS_WING, interfaces.dash_interface.isBMSOk() ? LED_color_e::GREEN : LED_color_e::RED);

    /* DRIVETRAIN LEDS */
    setNeopixelColor(LED_ID_e::BRAKE, brake_light_color);
    setNeopixelColor(LED_ID_e::INVERTER_ERR, vcr_int.isInverterErrored() ? LED_color_e::RED : LED_color_e::GREEN);
    setNeopixelColor(LED_ID_e::RDY_DRIVE, ready_drive_color);
    setNeopixelColor(LED_ID_e::TORQUE_MODE, torque_mode_color);
    setNeopixelColor(LED_ID_e::IMPLAUSE, pedals_data.brake_and_accel_pressed_implausibility_high ? LED_color_e::RED : LED_color_e::GREEN);

    setNeopixelColor(LED_ID_e::LATCH_WING, hv_present ? LED_color_e::PURPLE : LED_color_e::GREEN);
    setNeopixelColor(LED_ID_e::INVERTER_ERR_WING, vcr_int.isInverterErrored() ? LED_color_e::RED : LED_color_e::GREEN);

    /* VOLTAGE MONITOR */
    setNeopixelColor(LED_ID_e::PACK, pack_color);
    setNeopixelColor(LED_ID_e::CRIT_CHARGE, LED_color_e::OFF);
    setNeopixelColor(LED_ID_e::GLV, LED_color_e::OFF);

    _neopixels.show();
}

void NeopixelController::setNeopixelColor(LED_ID_e led, LED_color_e color)
{
    _neopixels.setPixelColor(led, (uint32_t) color);
}