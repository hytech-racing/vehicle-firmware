#include "DisplayInterface.hpp"


void DisplayInterface::init()
{
    _display.begin();
    _display.setRotation(3);
    _display.setTextSize(2);
    _display.fillScreen(ILI9341_BLACK);
}

void DisplayInterface::displayData(unsigned long current_millis, bool is_120_switched)
{
    _display.fillScreen(ILI9341_BLACK);
    _display.setCursor(0, 0);
    _display.setRotation(3);

    switch (_display_view)
    {
    case DisplayView_e::VIEW_CHARGE_STATUS:
    {
        _display.setTextSize(2);
        _display.print("Set to ");
        _display.print(is_120_switched ? "120" : "240");
        _display.println("V Charging");
        _display.print("CCU SM: ");
        _display.println(ChargerStateMachineInstance::instance().getStateName());
        _display.print("Cell Voltage max: ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().high_voltage, 3);
        _display.print("Cell Voltage min: ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().low_voltage, 3);
        _display.print("Cell Voltage avg: ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().average_voltage, 3);
        _display.print("Cell Voltage delta: ");
        _display.println((ACUInterfaceInstance::instance().getLatestData().high_voltage - ACUInterfaceInstance::instance().getLatestData().low_voltage), 3);
        _display.print("Total Pack Volts: ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().pack_voltage, 3);
        _display.print("Max Board Temp (C): ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().max_board_temp, 3);
        _display.print("Max Cell Temp (C): ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().max_cell_temp, 3);
        _display.print("Min Cell Temp (C): ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().min_cell_temp, 3);
        _display.print("Avg Cell Temp (C): ");
        _display.println(
            (ACUInterfaceInstance::instance().getLatestData().max_cell_temp + ACUInterfaceInstance::instance().getLatestData().min_cell_temp) / 2,
            3);
        _display.print("Current Scalar: ");
        _display.print(RotaryEncoderInterfaceInstance::instance().getValue());
        _display.println("%");
        _display.print("EM current (A): ");
        _display.println(EnergyMeterInterfaceInstance::instance().getLatestEMData().current_amps, 3);
        _display.print("State of Charge (%): ");
        _display.println(ACUInterfaceInstance::instance().getLatestData().state_of_charge, 2);

        break;
    }
    case DisplayView_e::VIEW_CHARGER:
    {
        auto& charger = ChargerInterfaceInstance::instance();
        auto dc_output_v = charger.getDCOutputVoltageV();
        auto ac_input_v = charger.getACInputVoltageV();
        auto current_output_a = charger.getOutputCurrentA();
        _display.setTextSize(2);

        _display.print("Charger Output Current (A): ");
        _display.println(current_output_a, 2);
        _display.print("Charger DC Output Voltage (V): ");
        _display.println(dc_output_v, 2);
        _display.print("Charger AC Input Voltage (V): ");
        _display.println(ac_input_v, 2);
        _display.print("EM current (A): ");
        _display.println(EnergyMeterInterfaceInstance::instance().getLatestEMData().current_amps, 3);

        break;
    }
    case DisplayView_e::VIEW_VOLTAGE:
    {
        if (current_millis - _config.last_display_timestamp < _config.sliding_window_display_interval_ms)
        {
            break;
        }

        auto cell_voltages = ACUInterfaceInstance::instance().getLatestData().cell_voltages;
        _display.setTextSize(1);

        constexpr size_t CELLS_PER_ROW = 3;
        constexpr size_t ROWS_PER_PAGE = 14;
        constexpr size_t CELLS_PER_PAGE = CELLS_PER_ROW * ROWS_PER_PAGE;

        constexpr size_t TOTAL_CELLS = default_acu_params::NUM_CELLS;
        constexpr size_t TOTAL_ROWS = (TOTAL_CELLS + CELLS_PER_ROW - 1) / CELLS_PER_ROW;

        static size_t start_voltage_row = 0;
        const size_t start_index = start_voltage_row * CELLS_PER_ROW;

        for (size_t i = 0; i < CELLS_PER_PAGE; i++)
        {
            size_t idx = start_index + i;

            if (idx >= TOTAL_CELLS)
            {
                idx -= TOTAL_CELLS;
            }

            _display.print("C");
            _display.print(idx);
            _display.print(":");

            if (cell_voltages[idx].has_value())
            {
                _display.print(cell_voltages[idx].value(), 3);
            }
            else
            {
                _display.print("-.--");
            }

            if ((i + 1) % CELLS_PER_ROW == 0)
            {
                _display.println();
                _display.println();
            }
            else
            {
                _display.print("  ");
            }
        }

        start_voltage_row++;

        if (start_voltage_row >= TOTAL_ROWS)
        {
            start_voltage_row = 0;
        }

        break;
    }
    case DisplayView_e::VIEW_BOARD_TEMPERATURE:
    {
        if (current_millis - _config.last_display_timestamp < _config.sliding_window_display_interval_ms)
        {
            break;
        }

        auto board_temps = ACUInterfaceInstance::instance().getLatestData().board_temps;
        _display.setTextSize(1);

        constexpr size_t BOARD_TEMPS_PER_ROW = 3;

        for (size_t row = 0; row < default_acu_params::NUM_BOARD_TEMPS / BOARD_TEMPS_PER_ROW; row++)
        {
            for (size_t i = 0; i < BOARD_TEMPS_PER_ROW; i++)
            {
                size_t bt_index = (row * BOARD_TEMPS_PER_ROW) + i;

                _display.print("BT");
                _display.print(bt_index);
                _display.print(":");

                if (*board_temps[bt_index])
                {
                    _display.print(*board_temps[bt_index], 3);
                }
                else
                {
                    _display.print("--.-");
                }

                if ((i + 1) % BOARD_TEMPS_PER_ROW == 0)
                {
                    _display.println();
                    _display.println();
                }
                else
                {
                    _display.print("  ");
                }
            }
        }

        break;
    }
    case DisplayView_e::VIEW_CELL_TEMPERATURE:
    {
        if (current_millis - _config.last_display_timestamp < _config.sliding_window_display_interval_ms)
        {
            break;
        }

        auto cell_temps = ACUInterfaceInstance::instance().getLatestData().cell_temps;
        _display.setTextSize(1);

        constexpr size_t TEMPS_PER_ROW = 3;
        constexpr size_t ROWS_PER_PAGE = 14;
        constexpr size_t TEMPS_PER_PAGE = TEMPS_PER_ROW * ROWS_PER_PAGE;

        constexpr size_t TOTAL_CELL_TEMPS = default_acu_params::NUM_CELL_TEMPS;
        constexpr size_t TOTAL_ROWS = (TOTAL_CELL_TEMPS + TEMPS_PER_ROW - 1) / TEMPS_PER_ROW;

        static size_t start_temps_row = 0;
        const size_t start_index = start_temps_row * TEMPS_PER_ROW;

        for (size_t i = 0; i < TEMPS_PER_PAGE; i++)
        {
            size_t ct_idx = start_index + i;

            if (ct_idx >= TOTAL_CELL_TEMPS)
            {
                ct_idx -= TOTAL_CELL_TEMPS;
            }

            _display.print("CT");
            _display.print(ct_idx);
            _display.print(":");

            if (cell_temps[ct_idx].has_value())
            {
                _display.print(cell_temps[ct_idx].value(), 3);
            }
            else
            {
                _display.print("-.--");
            }

            if ((i + 1) % TEMPS_PER_ROW == 0)
            {
                _display.println();
                _display.println();
            }
            else
            {
                _display.print("  ");
            }
        }

        start_temps_row++;

        if (start_temps_row >= TOTAL_ROWS)
        {
            start_temps_row = 0;
        }

        break;
    }
    default:
    {
        break;
    }
    }
}

void DisplayInterface::refreshDisplayData(unsigned long curr_millis)
{
    if ((curr_millis - _display_time) >= _config.display_update_interval_ms)
    {
        _display_time = curr_millis;
    }
}

void DisplayInterface::update(unsigned long current_millis)
{
    _cycle_display_view_button.update(current_millis);
    _handleButtonEvents(current_millis);
}

void DisplayInterface::_handleButtonEvents(unsigned long current_millis)
{
    if (_cycle_display_view_button.getHoldDurationMs(current_millis) > _config.cycle_button_hold_time_reset_ms)
    {
        _display_view = DisplayView_e::VIEW_CHARGE_STATUS;
    }
    else if (_cycle_display_view_button.isPressed())
    {
        _cycleView();
    }
}

void DisplayInterface::_cycleView()
{
    _display_view = static_cast<DisplayView_e>((static_cast<size_t>(_display_view) + 1) % static_cast<size_t>(DisplayView_e::NUM_VIEWS));
}
