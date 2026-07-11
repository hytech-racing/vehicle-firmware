#include "DisplayInterface.h"


void DisplayInterface::init(SPI_HandleTypeDef *hspi)
{
    // set non-needed Display pins low
    pinMode(PB4, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PA3, OUTPUT);
    digitalWrite(PB4, LOW);
    digitalWrite(PB1, LOW);
    _display.begin();

    _hspi = hspi;

}

void DisplayInterface::startup()
{
    _display.drawBitmap(hytech_logo_x, hytech_logo_y, epd_bitmap_Hytech_Logo, hytech_logo_size, hytech_logo_size, 0);
    //hytech_animation();
    // driver_animation(StartupAnimations::NONE);
}

/**
 * @brief Function to display general hytech startup animation
 * @note Taken from 2024 dash code
 */
void DisplayInterface::hytech_animation()
{
    _display.clear_display_buffer();
    _display.setRotation(0);
    _display.drawBitmap(hytech_logo_x, hytech_logo_y, epd_bitmap_Hytech_Logo, hytech_logo_size, hytech_logo_size, _black);
    send_display_buffer(_hspi);
    delay(500);
    for (int i = 0; i > -116; i -= 3)
    {
        _display.clear_display_buffer();
        _display.drawBitmap(hytech_logo_x + i, hytech_logo_y, epd_bitmap_Hytech_Logo, hytech_logo_size, hytech_logo_size, _black);
        send_display_buffer(_hspi);
        delay(60);
    }
    _display.drawBitmap(hytech_words_x + 45, hytech_words_y, epd_bitmap_HytechWords, hytech_words_x_size, hytech_words_y_size, _black);
    send_display_buffer(_hspi);
    delay(60);
    _display.setFont(&FreeSans12pt7b);
    String greeting = "NEEEOOOWWMM";
    int length = greeting.length();
    _display.setCursor(hytech_logo_x - length * 3, hytech_logo_y + hytech_logo_size + 30);
    _display.setTextColor(_black);
    _display.setTextSize(1);
    _display.println(greeting);
    send_display_buffer(_hspi);
    delay(1000);
    _display.clear_display_buffer();
}

// void DisplayInterface::alysa_animation()
// {
//     _display.setRotation(0);
//     for (int i = 0; i < ALYSA_FRAME_COUNT; i++)
//     {
//         const uint8_t *frame = alysa_animation_array[i];
//         _display.clearDisplayBuffer();
//         _display.drawBitmap(52, 0, frame, 216, 240, _black);
//         send_display_buffer(_hspi);
//         delay(400);
//     }
//     _display.clearDisplayBuffer();
// }

void DisplayInterface::draw_background()
{
    _display.clear_display_buffer();
    _display.fillRect(0, 0, 320, 240, _white);

    //_display.drawBitmap(0, 0, epd_bitmap_hytech_dashboard, 320, 240, _black);
    _display.fillRect(320 - 40, 30, 40, 200, _white);
    //_display.fillRect(283, 36, 305 - 283, 210 - 36, _black);
    _display.fillRect(283 - 3, (36 + 210 - 36) / 2 + 15, 25, 7, _white);
    _display.fillRect(0, 215, 130, 25, _white);

}

void DisplayInterface::send_display_buffer(SPI_HandleTypeDef *hspi)
{
    digitalWrite(PB4, HIGH); // set CS high before transmit, low in callback after transmit
    // SerialUSB.println("Starting DMA Transmit");

    uint8_t toggle_vcom[] = {vcom | SHARPMEM_BIT_WRITECMD};
    HAL_SPI_Transmit(hspi, toggle_vcom, sizeof(toggle_vcom), HAL_MAX_DELAY); // Test transmit to ensure SPI is working
    vcom = vcom ? 0x00 : SHARPMEM_BIT_VCOM;

    SCB_CleanDCache_by_Addr((uint32_t *)_display.getBuffer(), _display.getBufferSize()); // Clean D-Cache before DMA transfer
    hspi2.Instance->CR1 &= ~SPI_CR1_SPE;
    HAL_SPI_Transmit_DMA(hspi, _display.getBuffer(), _display.getBufferSize()); // Transmit the display buffer using DMA
}

void DisplayInterface::set_cursor(int x, int y)
{
    _display.setCursor(x, y);
}

void DisplayInterface::invert_display(bool invert_criteria)
{
    if (invert_criteria)
    {
        _black = 1;
        _white = 0;
    }
    else
    {
        _black = 0;
        _white = 1;
    }
}

// draws _white rect top down
void DisplayInterface::draw_vertical_pedal_bar(float val, int initial_x_coord)
{
    _display.fillRect(initial_x_coord, 33, 22, 189, _black);
    double ZERO_PERCENT_VAL = 185;
    val = std::max(0.0f, std::min(100.0f, val));
    int i = (int)(100 - val) * (ZERO_PERCENT_VAL / 100.0);
    _display.fillRect(initial_x_coord + 2, 35, 18, i, _white);
}

void DisplayInterface::draw_battery_bar(int percent)
{
    // 0%: 59
    // 100% 0
    _display.fillRect(60, 5, 250, 20, _black);
    int w = (100 - percent) * (246 / 100);
    DisplayInterface::_draw_rectangle_right_corner(306, 7, w, 16, _white);
}

void DisplayInterface::display_speeds(float rpm)
{
    _display.setFont(&FreeSans24pt7b);
    _display.setTextSize(2);
    _display.setTextColor(_black);

    _display.setCursor(100, 140);
    /** TODO: convert from RPM to MPH*/
    double wheelspeed = abs(rpm * conversions::RPM_TO_METERS_PER_SECOND);
    // SerialUSB.println(wheelspeed);
    uint16_t mph = (int)(wheelspeed * conversions::METERS_PER_SECOND_TO_MPH);
    // SerialUSB.println(mph);
    _display.println(DisplayInterface::_twoDigits(mph));
    // _display.println(HYTECH_low_voltage_ro_fromS(bms_voltages->low_voltage_ro));

    // _display.println(mph);
    _display.setTextSize(1);
    _display.setFont(&FreeSans12pt7b);
    _display.setCursor(125, 165);
    _display.print("MPH");
    _display.setFont(&FreeSans12pt7b);
}

void DisplayInterface::display_mode(int mode)
{
    _display.setFont(&FreeSansBold18pt7b);
    _display.setTextSize(1);
    _display.setTextColor(_black);

    _display.setCursor(10, 27);

    _display.println(DisplayInterface::_twoDigits(mode));

    // SerialUSB.println(mph);

}

void DisplayInterface::display_min_cell(float min_cell_voltage)
{
    _display.setFont(&FreeSansBold9pt7b);
    _display.setTextSize(1);
    _display.setTextColor(_black);
    int x_coord = 65;
    _display.setCursor(x_coord, 50);
    _display.print("CELL: ");
    // _display.setCursor(x_coord + 55, 55);
    _display.setFont(&FreeSans9pt7b);
    _display.print(min_cell_voltage);
}

void DisplayInterface::display_all_temps(veh_vec<int> temps)
{
    _display.setFont(&FreeSans9pt7b);
    _display.setTextSize(1);
    _display.setTextColor(_black);

    _display.setCursor(220, 120);
    _display.print(temps.FL);
    _display.setCursor(250, 120);
    _display.print(temps.FR);
    _display.setCursor(220, 140);
    _display.print(temps.RL);
    _display.setCursor(250, 140);
    _display.print(temps.RR);
}

void DisplayInterface::display_max_temps(int inverter_temp, int motor_temp)
{
    _display.setFont(&FreeSansBold9pt7b);
    _display.setTextSize(1);
    _display.setTextColor(_black);
    int x_coord = 65;
    int y_coord = 66;
    _display.setCursor(x_coord, y_coord);
    _display.print("TEMPS");

    _display.setCursor(x_coord, y_coord + 16);
    _display.print("INV: ");
    _display.setFont(&FreeSans9pt7b);
    _display.print(inverter_temp);

    _display.setCursor(x_coord, y_coord + 32);
    _display.setFont(&FreeSansBold9pt7b);
    _display.print("MTR: ");
    _display.setFont(&FreeSans9pt7b);
    _display.println(motor_temp);
}

void DisplayInterface::draw_icons(uint8_t vn_status, VehicleState_e car_state, bool db_in_ctrl)
{

    /* no gps icon   = 0 */
    /* vn flashing   = 1 */
    /* vn solid      = 2 */

    /* all units are pixels */
    int offset = 3;
    int icon_size = 27;

    /* horizontal icons on top of the screen */
    int gps_icon_pos_x = 270 - icon_size - offset;
    int rtd_icon_pos_x = gps_icon_pos_x - icon_size - offset;
    int db_ctrl_icon_pos_x = rtd_icon_pos_x - icon_size - offset;
    int icon_pos_y = 40;

    // if (vn_status >= 2)
    // {
    //     _display.drawBitmap(gps_icon_pos_x, icon_pos_y, epd_bitmap_gps, 27, 27, _black);
    // }
    // else if (vn_status == 1)
    // {
    //     if (DisplayInterface::blink())
    //     {
    //         _display.drawBitmap(gps_icon_pos_x, icon_pos_y, epd_bitmap_gps, 27, 27, _black);
    //     }
    // }
    // else if (vn_status == 0)
    // {
    //     _display.drawBitmap(gps_icon_pos_x, icon_pos_y, epd_bitmap_nogps, 27, 27, _black);
    // }

    if (car_state == VehicleState_e::READY_TO_DRIVE)
    {
        _display.drawBitmap(rtd_icon_pos_x, icon_pos_y, epd_bitmap_rtd, 27, 27, _black);
    }

    else
    {
        if (DisplayInterface::_blink())
        {
            _display.drawBitmap(rtd_icon_pos_x, icon_pos_y, epd_bitmap_rtd, 27, 27, _black);
        }
    }

    if (db_in_ctrl)
    {
        _display.drawBitmap(db_ctrl_icon_pos_x, icon_pos_y, epd_bitmap_db_ctrl_symbol, 27, 27, _black);
    }

    // if (check_latched(m)) {
    //     _display.drawBitmap(latched_icon_pos_x, icon_pos_y, epd_bitmap_latch_symbol, 27, 27, BLACK);
    // } else {
    //     if (blink()) { _display.drawBitmap(latched_icon_pos_x, icon_pos_y, epd_bitmap_latch_symbol, 27, 27, BLACK); }
    // }
}

void DisplayInterface::draw_popup(String title)
{
    int width = 300;
    int height = 150;
    _display.fillRect(160 - (width / 2), 120 - (height / 2), width, height, _black);
    width -= 10;
    height -= 10;
    _display.fillRect(160 - (width / 2), 120 - (height / 2), width, height, _white);
    _display.setFont(&FreeSansBold12pt7b);
    _display.setCursor(160 - (width / 2) + 25, 120 - (height / 2) + 50);
    int x = _display.getCursorX();
    _display.println(title);
    _display.setFont(&FreeSans12pt7b);
    _display.setCursor(x, _display.getCursorY());
}

void DisplayInterface::_draw_rectangle_right_corner(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    for (int16_t i = x; i > x - w; i--)
    {
        _display.writeFastVLine(i, y, h, color);
    }
    // _display.refresh();
}

String DisplayInterface::_twoDigits(int number)
{
    if (number <= 9)
    {
        return "0" + String(number);
    }
    if (number >= 10 && number <= 99)
    {
        return String(number);
    }
    if (number >= 100 && number <= 999)
    {
        return String(number / 10);
    }
    return String(number);
}

bool DisplayInterface::_blink()
{
    if ((millis() - DisplayInterface::_last_blink_millis) > 200)
    {
        DisplayInterface::_last_blink = !_last_blink;
        DisplayInterface::_last_blink_millis = millis();
    }
    return DisplayInterface::_last_blink;
}
