#include "Dash_Tasks.h"

veh_vec<int> fake_temps = {30, 32, 28, 31};


void initialize_all_interfaces()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    /* ACU Interface */
    ACUInterfaceInstance::create();

    /* Display Interface */
    DisplayInterfaceInstance::create(SHARP_CS);
    DisplayInterfaceInstance::instance().init(&hspi2);
    DisplayInterfaceInstance::instance().startup();

    /* VCF Interface */
    VCFInterfaceInstance::create(sys_time::hal_millis(), 50UL); // TODO: needs to be updated to use constexpr

    /* VCR Interface */
    VCRInterfaceInstance::create();

    /* Neopixel Controller */
    NeopixelControllerInstance::create(NEOPIXEL_COUNT, NEOPIXEL_CONTROL_PIN);
    NeopixelControllerInstance::instance().init_neopixels();

    /* CAN Interfaces */
    CANInterfacesInstance::create(
        VCFInterfaceInstance::instance(),
        ACUInterfaceInstance::instance(),
        VCRInterfaceInstance::instance()
    );

    HT_SPI_Init();
    FDCAN_init();
    FDCAN_set_interfaces(CANInterfacesInstance::instance());
};

HT_TASK::TaskResponse run_update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    NeopixelControllerInstance::instance().refresh_neopixels(CANInterfacesInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse screen_refresh(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    if (spi_tx_complete)
    {
        DisplayInterfaceInstance::instance().draw_background();
        DisplayInterfaceInstance::instance().invert_display(VCFInterfaceInstance::instance().is_mech_brake_pressed());
        DisplayInterfaceInstance::instance().draw_vertical_pedal_bar(VCFInterfaceInstance::instance().get_curr_data().stamped_pedals.pedals_data.brake_percent * 100, 5);
        DisplayInterfaceInstance::instance().draw_vertical_pedal_bar(VCFInterfaceInstance::instance().get_curr_data().stamped_pedals.pedals_data.accel_percent * 100, 34);

        DisplayInterfaceInstance::instance().draw_battery_bar((ACUInterfaceInstance::instance().get_curr_data().pack_voltage - 460) / 70 * 100.0 + 1);
        DisplayInterfaceInstance::instance().draw_icons(1, VCRInterfaceInstance::instance().get_curr_car_state(), VCRInterfaceInstance::instance().get_drivebrain_in_control());
        DisplayInterfaceInstance::instance().display_mode(VCFInterfaceInstance::instance().get_control_mode());
        // DisplayInterfaceInstance::instance().set_cursor(65, 50);
        DisplayInterfaceInstance::instance().display_min_cell(ACUInterfaceInstance::instance().get_curr_data().min_cell_voltage);
        DisplayInterfaceInstance::instance().display_max_temps(VCRInterfaceInstance::instance().get_inverter_max_temp(), VCRInterfaceInstance::instance().get_motor_max_temp());
        //DisplayInterfaceInstance::instance().display_speeds(VCRInterfaceInstance::instance().get_curr_wheel_data().actual_speed);

        if (ACUInterfaceInstance::instance().get_curr_data().imd_ok == false || ACUInterfaceInstance::instance().get_curr_data().bms_ok == false)
        {
            DisplayInterfaceInstance::instance().draw_popup("GET OUT!");
        }

        DisplayInterfaceInstance::instance().send_display_buffer(&hspi2);
        spi_tx_complete = false;
    }
    // DisplayInterfaceInstance::instance().invert_display(VCFInterfaceInstance::instance().is_mech_brake_pressed());
    // DisplayInterfaceInstance::instance().clear_display_buffer();
    // DisplayInterfaceInstance::instance().draw_vertical_pedal_bar(VCFInterfaceInstance::instance().get_curr_data().stamped_pedals.pedals_data.brake_percent, 17);
    // DisplayInterfaceInstance::instance().draw_battery_bar(ACUInterfaceInstance::instance().get_curr_data().pack_voltage * 100.0 / 530.0);
    // DisplayInterfaceInstance::instance().draw_icons(1/*DrivebrainInterfaceInstance::instance().get_db_state_data().vn_status*/, 1, 1, 0);

    // switch (DisplayInterfaceInstance::instance().current_page)
    // {
    //     case 0:
    //         DisplayInterfaceInstance::instance().display_speeds(VCRInterfaceInstance::instance().get_curr_wheel_data().actual_speed);
    //         break;
    // }

    // if (!(ACUInterfaceInstance::instance().imd_ok))
    // {
    //     DisplayInterfaceInstance::instance().draw_popup("DANGER! GET OUT FAST!");
    // }

    // DisplayInterfaceInstance::instance().display_refresh();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse can_read(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    // CAN messages are now processed in interrupt handler
    // This task just yields - interrupts handle message reception
    return HT_TASK::TaskResponse::YIELD;
}