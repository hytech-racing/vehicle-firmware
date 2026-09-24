#include "VCF_InterfaceTasks.h"

void initializeAllInterfaces()
{
    SPI.begin();
    Serial.begin(VCFInterfaces::SERIAL_BAUDRATE); // NOLINT

    /* Watchdog Interface */
    WatchdogInterfaceInstance::create(WatchdogPinout_s {
                                        VCFInterfaces::WATCHDOG_KICK_PIN,
                                        VCFInterfaces::SOFTWARE_OK_PIN
                                    }
    );
    WatchdogInterfaceInstance::instance().init();

    /* ACU Interface */
    ACUInterfaceInstance::create();

    /* ADC Interface */
    ADCInterfaceInstance::create(
        ADCPinout_s
        {
            VCFInterfaces::ADC0_CS,
            VCFInterfaces::ADC1_CS
        },
        ADCChannels_s
        {
            VCFInterfaces::PEDAL_REF_2V5_CHANNEL,
            VCFInterfaces::STEERING_1_CHANNEL,
            VCFInterfaces::STEERING_2_CHANNEL,
            VCFInterfaces::ACCEL_1_CHANNEL,
            VCFInterfaces::ACCEL_2_CHANNEL,
            VCFInterfaces::BRAKE_1_CHANNEL,
            VCFInterfaces::BRAKE_2_CHANNEL,

            VCFInterfaces::SHDN_H_CHANNEL,
            VCFInterfaces::SHDN_D_CHANNEL,
            VCFInterfaces::FL_LOADCELL_CHANNEL,
            VCFInterfaces::FR_LOADCELL_CHANNEL,
            VCFInterfaces::FR_SUS_POT_CHANNEL,
            VCFInterfaces::FL_SUS_POT_CHANNEL,
            VCFInterfaces::BRAKE_PRESSURE_FRONT_CHANNEL,
            VCFInterfaces::BRAKE_PRESSURE_REAR_CHANNEL
        },
        ADCScales_s
        {
            VCFInterfaces::PEDAL_REF_2V5_SCALE,
            VCFInterfaces::STEERING_1_SCALE,
            VCFInterfaces::STEERING_2_SCALE,
            VCFInterfaces::ACCEL_1_SCALE,
            VCFInterfaces::ACCEL_2_SCALE,
            VCFInterfaces::BRAKE_1_SCALE,
            VCFInterfaces::BRAKE_2_SCALE,

            VCFInterfaces::SHDN_H_SCALE,
            VCFInterfaces::SHDN_D_SCALE,
            VCFInterfaces::FL_LOADCELL_SCALE,
            VCFInterfaces::FR_LOADCELL_SCALE,
            VCFInterfaces::FR_SUS_POT_SCALE,
            VCFInterfaces::FL_SUS_POT_SCALE,
            VCFInterfaces::BRAKE_PRESSURE_FRONT_SCALE,
            VCFInterfaces::BRAKE_PRESSURE_REAR_SCALE
        },
        ADCOffsets_s
        {
            VCFInterfaces::PEDAL_REF_2V5_OFFSET,
            VCFInterfaces::STEERING_1_OFFSET,
            VCFInterfaces::STEERING_2_OFFSET,
            VCFInterfaces::ACCEL_1_OFFSET,
            VCFInterfaces::ACCEL_2_OFFSET,
            VCFInterfaces::BRAKE_1_OFFSET,
            VCFInterfaces::BRAKE_2_OFFSET,

            VCFInterfaces::SHDN_H_OFFSET,
            VCFInterfaces::SHDN_D_OFFSET,
            VCFInterfaces::FL_LOADCELL_OFFSET,
            VCFInterfaces::FR_LOADCELL_OFFSET,
            VCFInterfaces::FR_SUS_POT_OFFSET,
            VCFInterfaces::FL_SUS_POT_OFFSET,
            VCFInterfaces::BRAKE_PRESSURE_FRONT_OFFSET,
            VCFInterfaces::BRAKE_PRESSURE_REAR_OFFSET
        }
    );

    /* Brake Rotor Temp Interface */
    BrakeRotorTempInterfaceInstance::create();

    /* Dashboard Interface */
    DashboardGPIOs_s dashboard_gpios = {
        .BRIGHTNESS_CONTROL_PIN = VCFInterfaces::BRIGHTNESS_CONTROL_PIN,
        .PRESET_BUTTON = VCFInterfaces::BTN_PRESET_READ,
        .MC_CYCLE_BUTTON = VCFInterfaces::BTN_MC_CYCLE_READ,
        .START_BUTTON = VCFInterfaces::BTN_START_READ,
        .DATA_BUTTON = VCFInterfaces::BTN_DATA_READ,
        .BUTTON_2 = VCFInterfaces::BUTTON_2
    };
    DashboardInterfaceInstance::create(dashboard_gpios, VCFSystems::IO_EXPANDER_ADDR, Wire2); //NOLINT
    DashboardInterfaceInstance::instance().init();

    /* Orbis Interface */
    OrbisInterfaceInstance::create(&Serial2);

    /* VCR Interface */
    VCRInterfaceInstance::create();

    /* CAN Interfaces */
    CANInterfacesInstance::create(ACUInterfaceInstance::instance(),
                                BrakeRotorTempInterfaceInstance::instance(),
                                DashboardInterfaceInstance::instance(),
                                VCRInterfaceInstance::instance()
    );

    VCFCANInterfaceInstance::create(etl::delegate<void(CANInterfaces_s&, const CAN_message_t&, unsigned long, CANInterfaceType_e)>::create<VCFCANInterfaceImpl::vcf_recv_switch>());

    handle_CAN_setup(VCFCANInterfaceInstance::instance().TELEM_CAN, VCFConstants::TELEM_CAN_BAUDRATE, &VCFCANInterfaceImpl::on_telem_can_recv);
    handle_CAN_setup(VCFCANInterfaceInstance::instance().FRONT_AUX_CAN, VCFConstants::FAUX_CAN_BAUDRATE, &VCFCANInterfaceImpl::on_front_aux_can_recv);

    /* Ethernet */
    VCFEthernetInterfaceInstance::create();
    VCFEthernetInterfaceInstance::instance().initEthernetDevice();
}

HT_TASK::TaskResponse readADC0Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Updates all eight channels.
    ADCInterfaceInstance::instance().tickADC0();
    PedalsSystemInstance::instance().set_pedals_sensor_data(PedalSensorData_s {
        .accel_1 = static_cast<uint32_t>(ADCInterfaceInstance::instance().getAcceleration1().conversion),
        .accel_2 = static_cast<uint32_t>(ADCInterfaceInstance::instance().getAcceleration2().conversion),
        .brake_1 = static_cast<uint32_t>(ADCInterfaceInstance::instance().getBrake1().conversion),
        .brake_2 = static_cast<uint32_t>(ADCInterfaceInstance::instance().getBrake2().conversion)
    });

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse readADC1Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Samples all eight channels.
    ADCInterfaceInstance::instance().tickADC1();
    ADCInterfaceInstance::instance().updateFilteredCalues(VCFInterfaces::LOADCELL_IIR_FILTER_ALPHA);
    OrbisInterfaceInstance::instance().sample();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse kickWatchdogTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInterfaceInstance::instance().updateWatchdogState(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

// bool init_read_gpio_task()
// {
//     // Setting digital/analog buttons D10-D6, A8 as inputs
//     pinMode(BTN_DIM_READ, INPUT);
//     pinMode(BTN_PRESET_READ, INPUT);
//     pinMode(BTN_MC_CYCLE_READ, INPUT);
//     pinMode(BTN_MODE_READ, INPUT);
//     pinMode(BTN_START_READ, INPUT);
//     pinMode(BTN_DATA_READ, INPUT);

//     return HT_TASK::TaskResponse::YIELD;
// }
// bool run_read_gpio_task()
// {
//     // Doing digital read on all digital inputs
//     int dimButton = digitalRead(BTN_DIM_READ);
//     int presetButton = digitalRead(BTN_PRESET_READ);
//     int mcCycleButton = digitalRead(BTN_MC_CYCLE_READ);
//     int modeButton = digitalRead(BTN_MODE_READ);
//     int startButton = digitalRead(BTN_START_READ);
//     int dataButton = digitalRead(BTN_DATA_READ);

//     vcf_data.interface_data.dash_input_state.dim_btn_is_pressed = dimButton;
//     vcf_data.interface_data.dash_input_state.preset_btn_is_pressed = presetButton;
//     vcf_data.interface_data.dash_input_state.mc_reset_btn_is_pressed = mcCycleButton;
//     vcf_data.interface_data.dash_input_state.mode_btn_is_pressed = modeButton;
//     vcf_data.interface_data.dash_input_state.start_btn_is_pressed = startButton;
//     vcf_data.interface_data.dash_input_state.data_btn_is_pressed = dataButton;

//     return HT_TASK::TaskResponse::YIELD;
// }

HT_TASK::TaskResponse init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    pinMode(VCFInterfaces::BUZZER_CONTROL_PIN, OUTPUT);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    bool buzzer_is_active = BuzzerControllerInstance::instance().isBuzzerActive(sys_time::hal_millis()); //NOLINT

    digitalWrite(VCFInterfaces::BUZZER_CONTROL_PIN, buzzer_is_active);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_CAN_send(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFCANInterfaceImpl::send_all_CAN_msgs(VCFCANInterfaceInstance::instance().telem_can_tx_buffer, &VCFCANInterfaceInstance::instance().TELEM_CAN);
    VCFCANInterfaceImpl::send_all_CAN_msgs(VCFCANInterfaceInstance::instance().front_aux_can_tx_buffer, &VCFCANInterfaceInstance::instance().FRONT_AUX_CAN);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse send_dash_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CANInterfaces_s can_interfaces = CANInterfacesInstance::instance();
    DashInputState_s dash_outputs = can_interfaces.dash_interface.get_dashboard_outputs();

    DASH_INPUT_t msg_out;

    msg_out.dim_button = dash_outputs.btn_dim_read_is_pressed;
    msg_out.preset_button = dash_outputs.preset_btn_is_pressed;
    msg_out.mode_button = 0; // dont exist but i dont wanna bother changing can msgs
    msg_out.motor_controller_cycle_button = dash_outputs.mc_reset_btn_is_pressed;
    msg_out.start_button = dash_outputs.start_btn_is_pressed;
    msg_out.data_button_is_pressed = dash_outputs.data_btn_is_pressed;
    msg_out.left_shifter_button = 0;
    msg_out.right_shifter_button = dash_outputs.BUTTON_2;
    msg_out.led_dimmer_button = dash_outputs.brightness_ctrl_btn_is_pressed;
    msg_out.dash_dial_mode = static_cast<int>(DashboardInterfaceInstance::instance().get_dashboard_outputs().dial_state);

    CAN_util::enqueue_msg(&msg_out, &Pack_DASH_INPUT_hytech, VCFCANInterfaceInstance::instance().telem_can_tx_buffer);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_front_suspension_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    FRONT_SUSPENSION_t msg_out;

    msg_out.fr_load_cell = ADCInterfaceInstance::instance().getFilteredFRLoadcell();
    msg_out.fl_load_cell = ADCInterfaceInstance::instance().getFilteredFLLoadcell();
    msg_out.fr_shock_pot_ro = HYTECH_fr_shock_pot_ro_toS(ADCInterfaceInstance::instance().getFilteredFRSuspot());
    msg_out.fl_shock_pot_ro = HYTECH_fl_shock_pot_ro_toS(ADCInterfaceInstance::instance().getFilteredFLSuspot());

    CAN_util::enqueue_msg(&msg_out, &Pack_FRONT_SUSPENSION_hytech, VCFCANInterfaceInstance::instance().telem_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFEthernetInterfaceInstance::instance().initEthernetDevice();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    hytech_msgs_VCFData_s msg = VCFEthernetInterfaceInstance::instance().make_vcf_data_msg(ADCInterfaceInstance::instance(),
                                                                                        DashboardInterfaceInstance::instance(),
                                                                                        PedalsSystemInstance::instance(),
                                                                                        SteeringSystemInstance::instance(),
                                                                                        BrakeRotorTempInterfaceInstance::instance()
    );
    VCFEthernetInterfaceInstance::instance().handleSendVCFETHData(msg);
    return HT_TASK::TaskResponse::YIELD;
}

// HT_TASK::TaskResponse init_handle_receive_vcr_ethernet_data() {
//     VCFEthernetInterface::VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

//     return HT_TASK::TaskResponse::YIELD;
// }

// HT_TASK::TaskResponse run_handle_receive_vcr_ethernet_data() {
//     etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&VCFEthernetInterface::VCF_socket, &hytech_msgs_VCRData_s_msg);

//     return HT_TASK::TaskResponse::YIELD;
// }


HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    bool was_dim_btn_pressed = DashboardInterfaceInstance::instance().get_dashboard_stored_state().brightness_ctrl_btn_is_pressed; //NOLINT (linter thinks variable uninitialized)
    DashInputState_s current_state = DashboardInterfaceInstance::instance().get_dashboard_outputs();

    if (!current_state.preset_btn_is_pressed) //preset btn tied to brightness control on schematic
    {
        VCRInterfaceInstance::instance().disablePedalsCalibration();
    }

    if (!current_state.data_btn_is_pressed)
    {
        VCRInterfaceInstance::instance().disableSteeringCalibration();
    }

    // Checks if dim btn has been clicked (falling edge)
    if (was_dim_btn_pressed && !current_state.brightness_ctrl_btn_is_pressed)
    {
        NeopixelControllerInstance::instance().dimNeopixels();
    }

    DashboardInterfaceInstance::instance().readIOExpander();

    DashboardInterfaceInstance::instance().syncDashboardStoredState();

    return HT_TASK::TaskResponse::YIELD;
}