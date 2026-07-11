#include "DashboardInterface.h"

void DashboardInterface::init()
{
    pinMode(_dashboard_gpios.START_BUTTON, INPUT_PULLUP);
    pinMode(_dashboard_gpios.PRESET_BUTTON, INPUT_PULLUP);
    pinMode(_dashboard_gpios.MC_CYCLE_BUTTON, INPUT_PULLUP);
    pinMode(_dashboard_gpios.BRIGHTNESS_CONTROL_PIN, INPUT_PULLUP);
    pinMode(_dashboard_gpios.DATA_BUTTON, INPUT_PULLUP);
    pinMode(_dashboard_gpios.BUTTON_2, INPUT_PULLUP);

    _dash_created_millis = sys_time::hal_millis();

    _i2c_bus.begin();

    _io_expander.init();
    _io_expander.portMode(MCP23017Port::A, 0b00000000); // 0b0000 0000 = 0
    _io_expander.portMode(MCP23017Port::B, 0b01111111); // 0b0111 1111 = 127
    _io_expander.writeRegister(MCP23017Register::GPPU_B, 0xFF); // Internal pull-ups
    _io_expander.writeRegister(MCP23017Register::IPOL_B, 0xFF); // Polarity (inverted)
}

void DashboardInterface::sync_dashboard_stored_state()
{
    _dashboard_stored_state = _dashboard_outputs;
}

void DashboardInterface::receive_ACU_OK(const CAN_message_t &can_msg)
{
    ACU_OK_t unpacked_msg;
    Unpack_ACU_OK_hytech(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT (implicitly decay pointer)

    bms_ok = unpacked_msg.bms_ok;
    imd_ok = unpacked_msg.imd_ok;
}

void DashboardInterface::set_dial_state(ControllerMode_e mode)
{
    _dashboard_outputs.dial_state = mode;
}

void DashboardInterface::read_ioexpander()
{
    uint16_t data = _io_expander.read(); // read data from IOExpander
    ControllerMode_e new_mode = ControllerMode_e::MODE_0; // default to mode 0

    // check for value of dial
    if (IOExpanderUtilities::getBit(data, (bool) MCP23017Port::B, 0)) // NOLINT 0 is pos of bit
    {
        new_mode = ControllerMode_e::MODE_0;
    }
    else if (IOExpanderUtilities::getBit(data, (bool) MCP23017Port::B, 1)) // NOLINT 1 is pos of bit
    {
        new_mode = ControllerMode_e::MODE_1;
    }
    else if (IOExpanderUtilities::getBit(data, (bool) MCP23017Port::B, 2)) // NOLINT 2 is pos of bit
    {
        new_mode = ControllerMode_e::MODE_2;
    }
    else if (IOExpanderUtilities::getBit(data, (bool) MCP23017Port::B, 3)) // NOLINT 3 is pos of bit
    {
        new_mode = ControllerMode_e::MODE_3;
    }
    else if (IOExpanderUtilities::getBit(data, (bool) MCP23017Port::B, 4)) // NOLINT 4 is pos of bit
    {
        new_mode = ControllerMode_e::MODE_4;
    }
    else if (IOExpanderUtilities::getBit(data, (bool) MCP23017Port::B, 5)) // NOLINT 5 is pos of bit
    {
        new_mode = ControllerMode_e::MODE_5;
    }

    _dashboard_outputs.dial_state = new_mode; // set new mode

    // write to 8-seg display based on current mode
    switch (_dashboard_outputs.dial_state)
    {
        case ControllerMode_e::MODE_0:
        {
            _io_expander.writePort(MCP23017Port::A, 0b00000010); // NOLINT 0b0000 0010 = 2
            break;
        }
        case ControllerMode_e::MODE_1:
        {
            _io_expander.writePort(MCP23017Port::A, 0b01010111); // NOLINT 0b0101 0111 = 87
            break;
        }
        case ControllerMode_e::MODE_2:
        {
            _io_expander.writePort(MCP23017Port::A, 0b00011000); // NOLINT 0b0001 1000 = 24
            break;
        }
        case ControllerMode_e::MODE_3:
        {
            _io_expander.writePort(MCP23017Port::A, 0b00010100); // NOLINT 0b0001 0100 = 20
            break;
        }
        case ControllerMode_e::MODE_4:
        {
            _io_expander.writePort(MCP23017Port::A, 0b01000101); // NOLINT 0b0100 0101 = 69
            break;
        }
        case ControllerMode_e::MODE_5:
        {
            _io_expander.writePort(MCP23017Port::A, 0b00100100); // NOLINT 0b0010 0100 = 36
            break;
        }
        default:
        {
            _io_expander.writePort(MCP23017Port::A, 0b11110000); // NOLINT 0b1111 0000 = 240
            break;
        }
    }
}

/* Button reads */
DashInputState_s DashboardInterface::get_dashboard_outputs()
{
    _dashboard_outputs.brightness_ctrl_btn_is_pressed = !digitalRead(_dashboard_gpios.BRIGHTNESS_CONTROL_PIN);
    _dashboard_outputs.preset_btn_is_pressed = !digitalRead(_dashboard_gpios.PRESET_BUTTON);
    _dashboard_outputs.mc_reset_btn_is_pressed = !digitalRead(_dashboard_gpios.MC_CYCLE_BUTTON);
    _dashboard_outputs.start_btn_is_pressed = !digitalRead(_dashboard_gpios.START_BUTTON);
    _dashboard_outputs.data_btn_is_pressed = !digitalRead(_dashboard_gpios.DATA_BUTTON);
    _dashboard_outputs.BUTTON_2 = !digitalRead(_dashboard_gpios.BUTTON_2);

    return _dashboard_outputs;
}

DashInputState_s DashboardInterface::get_dashboard_stored_state()
{
    return _dashboard_stored_state;
}