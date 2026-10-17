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
    static DashScreens::ScreenSelector screen_selector;

    if (spi_tx_complete)
    {
        DisplayInterface &display = DisplayInterfaceInstance::instance();
        VCFInterface &vcf = VCFInterfaceInstance::instance();
        VCRInterface &vcr = VCRInterfaceInstance::instance();
        const ACUData_s acu_data = ACUInterfaceInstance::instance().get_curr_data();
        const VCFCANInterfaceData_s vcf_data = vcf.get_curr_data();

        /* Pick the screen from the (debounced) dial position */
        const DashScreen_e screen = screen_selector.update(vcf.get_dial_mode());

        /* Gather everything the screens need once per frame */
        DashScreens::ScreenData_s data;
        data.speed_mph = DisplayInterface::rpm_to_mph(vcr.get_curr_wheel_data().actual_speed_rpm);
        data.soc_percent = acu_data.soc_percent;
        data.soe_percent = acu_data.soe_percent;
        data.min_cell_voltage = acu_data.min_cell_voltage;
        data.pack_voltage = acu_data.pack_voltage;
        data.inverter_max_temp = vcr.get_inverter_max_temp();
        data.motor_max_temp = vcr.get_motor_max_temp();
        data.dial_mode = screen_selector.confirmed_mode();

        /* Battery bar: real SoC once the ACU has sent it, otherwise the old pack-voltage estimate (460-530 V) */
        const int battery_percent = (data.soc_percent >= 0.0f)
                                        ? static_cast<int>(data.soc_percent + 0.5f)
                                        : static_cast<int>((acu_data.pack_voltage - 460) / 70 * 100.0 + 1);

        /* Set the colours first so the background and the widgets drawn after it agree (brake pressed = inverted) */
        display.invert_display(vcf.is_mech_brake_pressed());
        display.draw_background();

        /* Shared frame, identical on every screen */
        display.draw_vertical_pedal_bar(vcf_data.stamped_pedals.pedals_data.brake_percent * 100, 5);
        display.draw_vertical_pedal_bar(vcf_data.stamped_pedals.pedals_data.accel_percent * 100, 34);
        display.draw_battery_bar(battery_percent);
        display.draw_icons(1, vcr.get_curr_car_state(), vcr.get_drivebrain_in_control());
        display.display_mode(static_cast<int>(data.dial_mode));

        /* Screen-specific content */
        switch (screen)
        {
            case DashScreen_e::ACCELERATION:
                display.draw_screen_acceleration(data);
                break;
            case DashScreen_e::HANDLING:
                display.draw_screen_handling(data);
                break;
            case DashScreen_e::ENDURANCE:
                display.draw_screen_endurance(data);
                break;
            case DashScreen_e::COOLING:
                display.draw_screen_cooling(data);
                break;
            case DashScreen_e::STANDARD:
            default:
                display.draw_screen_standard(data);
                break;
        }

        /* Fault popup sits on top of every screen */
        if (acu_data.imd_ok == false || acu_data.bms_ok == false)
        {
            display.draw_popup("GET OUT!");
        }

        display.send_display_buffer(&hspi2);
        spi_tx_complete = false;
    }
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse can_read(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    // CAN messages are now processed in interrupt handler
    // This task just yields - interrupts handle message reception
    return HT_TASK::TaskResponse::YIELD;
}