#include "VCF_InterfaceTasks.h"

void initialize_all_interfaces()
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
    OrbisInterfaceInstance::create(&Serial2); // fix t

    /* VCR Interface */
    VCRInterfaceInstance::create();

    /* CAN Interfaces */
    CANInterfacesInstance::create(ACUInterfaceInstance::instance(),
                                BrakeRotorTempInterfaceInstance::instance(),
                                DashboardInterfaceInstance::instance(),
                                VCRInterfaceInstance::instance()
    );
    handle_CAN_setup(VCFCANInterfaceInstance::instance().TELEM_CAN, VCFConstants::TELEM_CAN_BAUDRATE, &VCFCANInterfaceImpl::on_telem_can_recv);
    handle_CAN_setup(VCFCANInterfaceInstance::instance().FRONT_AUX_CAN, VCFConstants::FAUX_CAN_BAUDRATE, &VCFCANInterfaceImpl::on_front_aux_can_recv);

    // Create Ethernet singletons
    VCFEthernetInterfaceInstance::create();
    VCFEthernetInterfaceInstance::instance().init_ethernet_device();
}

HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Updates all eight channels.
    ADCInterfaceInstance::instance().tick_adc0();
    PedalsSystemInstance::instance().set_pedals_sensor_data(PedalSensorData_s {
        .accel_1 = static_cast<uint32_t>(ADCInterfaceInstance::instance().get_acceleration_1().conversion),
        .accel_2 = static_cast<uint32_t>(ADCInterfaceInstance::instance().get_acceleration_2().conversion),
        .brake_1 = static_cast<uint32_t>(ADCInterfaceInstance::instance().get_brake_1().conversion),
        .brake_2 = static_cast<uint32_t>(ADCInterfaceInstance::instance().get_brake_2().conversion)
    });

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Samples all eight channels.
    ADCInterfaceInstance::instance().tick_adc1();
    ADCInterfaceInstance::instance().update_filtered_values(VCFInterfaces::LOADCELL_IIR_FILTER_ALPHA);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInterfaceInstance::instance().update_watchdog_state(sys_time::hal_millis());
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

    bool buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()); //NOLINT

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

    msg_out.fr_load_cell = ADCInterfaceInstance::instance().get_filtered_FR_load_cell();
    msg_out.fl_load_cell = ADCInterfaceInstance::instance().get_filtered_FL_load_cell();
    msg_out.fr_shock_pot_ro = HYTECH_fr_shock_pot_ro_toS(ADCInterfaceInstance::instance().get_filtered_FR_sus_pot());
    msg_out.fl_shock_pot_ro = HYTECH_fl_shock_pot_ro_toS(ADCInterfaceInstance::instance().get_filtered_FL_sus_pot());

    CAN_util::enqueue_msg(&msg_out, &Pack_FRONT_SUSPENSION_hytech, VCFCANInterfaceInstance::instance().telem_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFEthernetInterfaceInstance::instance().init_ethernet_device();
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
    VCFEthernetInterfaceInstance::instance().handle_send_ethernet_vcf_data(msg);
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
        VCRInterfaceInstance::instance().disable_calibration_state();
    }

    if (!current_state.data_btn_is_pressed)
    {
        VCRInterfaceInstance::instance().disable_steering_calibration_state();
    }

    // Checks if dim btn has been clicked (falling edge)
    if (was_dim_btn_pressed && !current_state.brightness_ctrl_btn_is_pressed)
    {
        NeopixelControllerInstance::instance().dim_neopixels();
    }

    DashboardInterfaceInstance::instance().read_ioexpander();

    DashboardInterfaceInstance::instance().sync_dashboard_stored_state();

    return HT_TASK::TaskResponse::YIELD;
}

namespace async_tasks
{
    // these are async tasks. we want these to run as fast as possible p much
    void handle_async_CAN_receive() //NOLINT caps for CAN
    {
        process_ring_buffer(VCFCANInterfaceInstance::instance().telem_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), VCFCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::TELEM);
        process_ring_buffer(VCFCANInterfaceInstance::instance().front_aux_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), VCFCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::FAUX);
    }

    void handle_async_recvs()
    {
        // ethernet, etc...

        handle_async_CAN_receive();
    }

    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handle_async_recvs();

        // SteeringSystemInstance::instance().evaluate_steering(
        //     ADCInterfaceInstance::instance().get_steering_degrees_cw().conversion,
        //     // OrbisInterfaceInstance::instance().getLastReading(),
        //     sys_time::hal_millis()
        // );

        PedalsSystemInstance::instance().evaluate_pedals(
            PedalsSystemInstance::instance().get_pedals_sensor_data(),
            sys_time::hal_millis()
        );
        return HT_TASK::TaskResponse::YIELD;
    }
};


HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    /* Pedals Info */
    Serial.println("\n\nPedals Info:");
    Serial.println("\tPercent Pressed Implaus Min 1 \tMax 1 \tMin 2 \tMax 2");
    // Accel
    Serial.print("Accel: \t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_percent); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_pressed); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_implausible); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().max_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_2); Serial.print("\t");
    Serial.println(PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
    // Brake
    Serial.print("Brake: \t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_percent); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_pressed); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_implausible); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().max_pedal_1); Serial.print("\t");
    Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_2); Serial.print("\t");
    Serial.println(PedalsSystemInstance::instance().get_brake_params().max_pedal_2);

    /* Steering System Data */
    Serial.println("Steering Sensor Data: ");
    Serial.print("analog adc: ");
    Serial.print(SteeringSystemInstance::instance().get_steering_system_data().analog_raw); Serial.print(" ");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_cw().raw);
    Serial.print("|");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_steering_angle);
    Serial.print("digital adc: ");
    Serial.print(SteeringSystemInstance::instance().get_steering_system_data().digital_raw);
    Serial.print("|");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_steering_angle);
    Serial.print("min_observed_analog: ");
    // Serial.println(SteeringSystemInstance::instance().get_min_observed_analog());
    // Serial.print("max_observed_analog: ");
    // Serial.println(SteeringSystemInstance::instance().get_max_observed_analog());
    Serial.print("analog_steering_angle: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_steering_angle);
    Serial.print("digital_steering_angle: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_steering_angle);
    Serial.print("time: ");
    Serial.println(sys_time::hal_millis());

    Serial.print("output_steering_angle: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().output_steering_angle);

    Serial.print("analog_steering_velocity_deg_s: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_steering_velocity_deg_s);
    Serial.print("digital_steering_velocity_deg_s: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_steering_velocity_deg_s);

    Serial.print("digital_oor_implausibility: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().digital_oor_implausibility);
    Serial.print("analog_oor_implausibility: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().analog_oor_implausibility);
    Serial.print("sensor_disagreement_implausibility: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().sensor_disagreement_implausibility);
    Serial.print("dtheta_exceeded_analog: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().dtheta_exceeded_analog);
    Serial.print("dtheta_exceeded_digital: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().dtheta_exceeded_digital);
    Serial.print("both_sensors_fail: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().both_sensors_fail);
    Serial.print("interface_sensor_error: ");
    Serial.println(SteeringSystemInstance::instance().get_steering_system_data().interface_sensor_error);

    /* ADC Values */
    Serial.println("\nADC Vals:");
    // ADC 0
    Serial.println("ADC 0\t\t  Steering");
    Serial.println("\t2V5 Ref CW \tCCW \tAccel 1 Accel 2 Brake 1 Brake 2");
    // Raw values
    Serial.print("Raw\t");
    Serial.print(ADCInterfaceInstance::instance().pedal_reference().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_cw().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_ccw().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_acceleration_1().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_acceleration_2().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_brake_1().raw); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().get_brake_2().raw);
    // Converted values
    Serial.print("Convert\t");
    Serial.print(ADCInterfaceInstance::instance().pedal_reference().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_cw().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_steering_degrees_ccw().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_acceleration_1().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_acceleration_2().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_brake_1().conversion); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().get_brake_2().conversion);

    // ADC 1
    Serial.println("\nADC 1\t\t\t  Load Cells \t  Sus Pots \t Brake Pressure");
    Serial.println("\tSHDN H \tSHDN D \tFL \tFR \tFR \tFL \tFront \tRear");
    // Raw ADC
    Serial.print("Raw\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_h().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_d().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_FL_load_cell().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_FR_load_cell().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_FR_sus_pot().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_FL_sus_pot().raw); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_brake_pressure_front().raw); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().get_brake_pressure_rear().raw);
    // Conversion ADC
    Serial.print("Convert\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_h().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().shdn_d().conversion); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FL_load_cell()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FR_load_cell()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FR_sus_pot()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_filtered_FL_sus_pot()); Serial.print("\t");
    Serial.print(ADCInterfaceInstance::instance().get_brake_pressure_front().conversion); Serial.print("\t");
    Serial.println(ADCInterfaceInstance::instance().get_brake_pressure_rear().conversion);

    /* Dashboard Info */
    Serial.println("\nDash Buttons / Buzzer:");
    Serial.println("Preset \tReset \tStart \tData \tBuzzer");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().preset_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().mc_reset_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().start_btn_is_pressed); Serial.print("\t");
    Serial.print(DashboardInterfaceInstance::instance().get_dashboard_outputs().data_btn_is_pressed); Serial.print("\t");
    Serial.println(BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()));

    /* Brake Rotor Temp Info */
    Serial.println("\nBrake Rotor Temps:");
    Serial.println("Sensor\tMax\tAvg\tCH0\tCH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7\tCH8\tCH9\tCH10\tCH11\tCH12\tCH13\tCH14\tCH15");

    // Sensor 1
    Serial.print("FL\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fl_sensor.max_temp); Serial.print("\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fl_sensor.avg_temp); Serial.print("\t");

    for (size_t i = 0; i < brake_rotor_temp_default_params::channels_within_brake_temp_sensor; ++i)
    {
        Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fl_sensor.channel_data[i]);
        Serial.print("\t");
    }
    Serial.println();

    // Sensor 2
    Serial.print("FR\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fr_sensor.max_temp); Serial.print("\t");
    Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fr_sensor.avg_temp); Serial.print("\t");

    for (size_t i = 0; i < brake_rotor_temp_default_params::channels_within_brake_temp_sensor; ++i)
    {
        Serial.print(BrakeRotorTempInterfaceInstance::instance().getBrakeRotorTempData().fr_sensor.channel_data[i]);
        Serial.print("\t");
    }
    Serial.println();

    return HT_TASK::TaskResponse::YIELD;
}