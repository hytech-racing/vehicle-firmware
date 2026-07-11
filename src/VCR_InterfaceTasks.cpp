#include "VCR_InterfaceTasks.h"


void initialize_all_interfaces()
{
    SPI.begin();
    analogReadResolution(VCRInterfaces::ANALOG_RESOLUTION);

    /* ---------- Pin Setup ----------*/
    // Should these be in a specific interface?
    pinMode(VCRInterfaces::MOTOR_COOLING_CONTROL_PIN, OUTPUT);
    pinMode(VCRInterfaces::INVERTER_COOLING_CONTROL_PIN, OUTPUT);
    pinMode(VCRInterfaces::INVERTER_ENABLE_PIN, OUTPUT);
    pinMode(VCRInterfaces::BRAKELIGHT_CONTROL_PIN, OUTPUT);

    /* ---------- Watchdog Interface ---------- */
    WatchdogInterfaceInstance::create(WatchdogPinout_s {
                                        VCRInterfaces::WATCHDOG_PIN,
                                        VCRInterfaces::SOFTWARE_OK_PIN
                                    }
    );
    WatchdogInterfaceInstance::instance().init();

    /* ---------- ACU Interface ---------- */
    ACUInterfaceInstance::create(sys_time::hal_millis(),
                                VCRInterfaces::ACU_ACU_OK_MAX_HEARTBEAT_MS
    );

    /* ---------- ADC Interface ---------- */
    ADCInterfaceInstance::create(ADCPinout_s {
                                    VCRInterfaces::ADC0_CS,
                                    VCRInterfaces::ADC1_CS,
                                    VCRInterfaces::BRAKE_HIGH_SENSE_PIN,
                                    VCRInterfaces::CURRENT_HIGH_SENSE_PIN
                                },
                                ADCChannels_s {
                                    VCRInterfaces::GLV_SENSE_CHANNEL,
                                    VCRInterfaces::CURRENT_SENSE_CHANNEL,
                                    VCRInterfaces::REFERENCE_SENSE_CHANNEL,
                                    VCRInterfaces::RL_LOADCELL_CHANNEL,
                                    VCRInterfaces::RR_LOADCELL_CHANNEL,
                                    VCRInterfaces::RL_SUS_POT_CHANNEL,
                                    VCRInterfaces::RR_SUS_POT_CHANNEL,
                                    VCRInterfaces::THERMISTOR_0_CHANNEL,
                                    VCRInterfaces::THERMISTOR_1_CHANNEL,
                                    VCRInterfaces::THERMISTOR_2_CHANNEL,
                                    VCRInterfaces::THERMISTOR_3_CHANNEL,
                                    VCRInterfaces::THERMISTOR_4_CHANNEL,
                                    VCRInterfaces::THERMISTOR_5_CHANNEL,
                                    VCRInterfaces::THERMISTOR_6_CHANNEL,
                                    VCRInterfaces::THERMISTOR_7_CHANNEL
                                },
                                ADCScales_s {
                                    VCRInterfaces::GLV_SENSE_SCALE,
                                    VCRInterfaces::CURRENT_SENSE_SCALE,
                                    VCRInterfaces::REFERENCE_SENSE_SCALE,
                                    VCRInterfaces::RL_LOADCELL_SCALE,
                                    VCRInterfaces::RR_LOADCELL_SCALE,
                                    VCRInterfaces::RL_SUS_POT_SCALE,
                                    VCRInterfaces::RR_SUS_POT_SCALE,
                                    VCRInterfaces::THERMISTOR_0_SCALE,
                                    VCRInterfaces::THERMISTOR_1_SCALE,
                                    VCRInterfaces::THERMISTOR_2_SCALE,
                                    VCRInterfaces::THERMISTOR_3_SCALE,
                                    VCRInterfaces::THERMISTOR_4_SCALE,
                                    VCRInterfaces::THERMISTOR_5_SCALE,
                                    VCRInterfaces::THERMISTOR_6_SCALE,
                                    VCRInterfaces::THERMISTOR_7_SCALE,
                                    VCRInterfaces::COOLANT_TEMP_SCALE
                                },
                                ADCOffsets_s {
                                    VCRInterfaces::GLV_SENSE_OFFSET,
                                    VCRInterfaces::CURRENT_SENSE_OFFSET,
                                    VCRInterfaces::REFERENCE_SENSE_OFFSET,
                                    VCRInterfaces::RL_LOADCELL_OFFSET,
                                    VCRInterfaces::RR_LOADCELL_OFFSET,
                                    VCRInterfaces::RL_SUS_POT_OFFSET,
                                    VCRInterfaces::RR_SUS_POT_OFFSET,
                                    VCRInterfaces::THERMISTOR_0_OFFSET,
                                    VCRInterfaces::THERMISTOR_1_OFFSET,
                                    VCRInterfaces::THERMISTOR_2_OFFSET,
                                    VCRInterfaces::THERMISTOR_3_OFFSET,
                                    VCRInterfaces::THERMISTOR_4_OFFSET,
                                    VCRInterfaces::THERMISTOR_5_OFFSET,
                                    VCRInterfaces::THERMISTOR_6_OFFSET,
                                    VCRInterfaces::THERMISTOR_7_OFFSET,
                                    VCRInterfaces::COOLANT_TEMP_OFFSET
                                }
    );
    ADCInterfaceInstance::instance().init();

    /* ---------- Drivebrain Interface ---------- */
    DrivebrainInterfaceInstance::create(EthernetIPDefsInstance::instance().drivebrain_ip,
                                        EthernetIPDefsInstance::instance().VCRData_port,
                                        VCREthernetInterfaceInstance::instance().get_vcr_data_send_socket()
    );

    /* ---------- IO Expander Interface ---------- */
    IOExpanderInterfaceInstance::create(Wire,
                                        IOExpanderInterfaceParams_s {
                                            VCRInterfaces::IOEXPANDER_I2C_ADDRESS,
                                            IOExpanderPortMode_s {
                                                VCRInterfaces::PORTA_DIRECTIONS,
                                                VCRInterfaces::PORTA_PULLUPS,
                                                VCRInterfaces::PORTA_INVERTED
                                            },
                                            IOExpanderPortMode_s {
                                                VCRInterfaces::PORTB_DIRECTIONS,
                                                VCRInterfaces::PORTB_PULLUPS,
                                                VCRInterfaces::PORTB_INVERTED
                                            }
                                        }
    );

    /* ---------- VCF Interface ---------- */
    VCFInterfaceInstance::create(sys_time::hal_millis(), VCRInterfaces::VCF_PEDALS_MAX_HEARTBEAT_MS);

    /* ---------- Ethernet Interface ---------- */
    VCREthernetInterfaceInstance::create();
    VCREthernetInterfaceInstance::instance().init_ethernet_device();

    /* ---------- CAN Interfaces ---------- */
    CANInterfacesInstance::create(ACUInterfaceInstance::instance(),
                                DrivebrainInterfaceInstance::instance(),
                                fl_inverter_interface,
                                fr_inverter_interface,
                                rl_inverter_interface,
                                rr_inverter_interface,
                                VCFInterfaceInstance::instance()
    );
    handle_CAN_setup(VCRCANInterfaceInstance::instance().INVERTER_CAN, VCRConstants::INVERTER_CAN_BAUDRATE, &VCRCANInterfaceImpl::on_inverter_can_receive);
    handle_CAN_setup(VCRCANInterfaceInstance::instance().TELEM_CAN, VCRConstants::TELEM_CAN_BAUDRATE, &VCRCANInterfaceImpl::on_telem_can_receive);
    handle_CAN_setup(VCRCANInterfaceInstance::instance().REAR_AUX_CAN, VCRConstants::RAUX_CAN_BAUDRATE, &VCRCANInterfaceImpl::on_auxillary_can_receive);
}

HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick_adc0();
    ADCInterfaceInstance::instance().update_filtered_values(VCRInterfaces::LOADCELL_IIR_FILTER_ALPHA);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick_adc1();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_acu_heartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUCANInterfaceData_s data = ACUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis());
    digitalWrite(VCRInterfaces::SOFTWARE_OK_PIN, data.heartbeat_ok);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInterfaceInstance::instance().update_watchdog_state(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse read_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) // TODO: make all of this in a separate IO Expander Interface
{
    IOExpanderInterfaceInstance::instance().read();

    // Inputs on Port A (0)
    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortA(0);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(1);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_fault = IOExpanderInterfaceInstance::instance().getBitPortA(2);
    vcr_data.interface_data.ethernet_is_linked.vn_link = IOExpanderInterfaceInstance::instance().getBitPortA(3);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = IOExpanderInterfaceInstance::instance().getBitPortA(4);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = IOExpanderInterfaceInstance::instance().getBitPortA(5);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_missing = IOExpanderInterfaceInstance::instance().getBitPortA(6);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(7);

    // Inputs on Port B (1)
    // vcr_data.interface_data.shutdown_sensing_data.lv_present = IOExpanderInterfaceInstance::instance().getBitPortB(0);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(1);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(2);
    vcr_data.interface_data.shutdown_sensing_data.vcr_sw_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(3);
    vcr_data.interface_data.ethernet_is_linked.acu_link = IOExpanderInterfaceInstance::instance().getBitPortB(4);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = IOExpanderInterfaceInstance::instance().getBitPortB(5);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = IOExpanderInterfaceInstance::instance().getBitPortB(6);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortB(7);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(VCRInterfaces::BRAKELIGHT_CONTROL_PIN, VCFInterfaceInstance::instance().is_brake_pressed());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enable_motor_cooling(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VehicleState_e vehicle_state = VehicleStateMachineInstance::instance().get_state(); //NOLINT will alway be populated so its ok
    bool enable_state = vehicle_state == VehicleState_e::READY_TO_DRIVE ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_2 ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_3;
    digitalWrite(VCRInterfaces::MOTOR_COOLING_CONTROL_PIN, enable_state ? HIGH : LOW);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enable_inverter_cooling(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VehicleState_e vehicle_state = VehicleStateMachineInstance::instance().get_state(); //NOLINT will alway be populated so its ok
    bool enable_state = vehicle_state == VehicleState_e::TRACTIVE_SYSTEM_ACTIVE ||
                        vehicle_state == VehicleState_e::WANTING_READY_TO_DRIVE ||
                        vehicle_state == VehicleState_e::READY_TO_DRIVE ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_2 ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_5;
    digitalWrite(VCRInterfaces::INVERTER_COOLING_CONTROL_PIN, enable_state ? HIGH : LOW);

    return HT_TASK::TaskResponse::YIELD;
}

// adds rear suspension and vcr status CAN messages to the sent on next mega loop run
HT_TASK::TaskResponse enqueue_suspension_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo )
{
    DrivebrainInterfaceInstance::instance().handle_enqueue_suspension_CAN_data(ADCInterfaceInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_flowmeter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
  DrivebrainInterfaceInstance::instance().handle_enqueue_flowmeter_CAN_data(FlowmeterInterfaceInstance::instance(), millis());
  return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_controls_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRControlsInstance::instance().send_controls_can_messages();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_coolant_temp_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handle_enqueue_coolant_temp_CAN_data(ADCInterfaceInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_inverter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    fl_inverter_interface.send_INV_CONTROL_WORD();
    fl_inverter_interface.send_INV_SETPOINT_COMMAND();

    fr_inverter_interface.send_INV_CONTROL_WORD();
    fr_inverter_interface.send_INV_SETPOINT_COMMAND();

    rl_inverter_interface.send_INV_CONTROL_WORD();
    rl_inverter_interface.send_INV_SETPOINT_COMMAND();

    rr_inverter_interface.send_INV_CONTROL_WORD();
    rr_inverter_interface.send_INV_SETPOINT_COMMAND();

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_dashboard_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFInterfaceInstance::instance().enqueue_vehicle_state_message(VehicleStateMachineInstance::instance().get_state(),
                                                                DrivetrainInstance::instance().get_state(),
                                                                VCRControlsInstance::instance().drivebrain_is_in_control());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceInstance::instance().inverter_can_tx_buffer, &VCRCANInterfaceInstance::instance().INVERTER_CAN);
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceInstance::instance().telem_can_tx_buffer, &VCRCANInterfaceInstance::instance().TELEM_CAN);
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceInstance::instance().rear_aux_can_tx_buffer, &VCRCANInterfaceInstance::instance().REAR_AUX_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_VCR_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handle_send_ethernet_data(VCREthernetInterfaceInstance::instance().make_vcr_data_msg(ADCInterfaceInstance::instance(),
                                                                    vcr_data.system_data.drivetrain_data,
                                                                    VCFInterfaceInstance::instance(),
                                                                    VehicleStateMachineInstance::instance(),
                                                                    DrivetrainInstance::instance(),
                                                                    fl_inverter_interface,
                                                                    fr_inverter_interface,
                                                                    rl_inverter_interface,
                                                                    rr_inverter_interface,
                                                                    VCRControlsInstance::instance())
    );
    return HT_TASK::TaskResponse::YIELD;
}

namespace async_tasks
{
    void handle_async_CAN_receive()
    {
        process_ring_buffer(VCRCANInterfaceInstance::instance().inverter_can_rx_buffer,
                            CANInterfacesInstance::instance(),
                            sys_time::hal_millis(),
                            VCRCANInterfaceInstance::instance().can_recv_switch,
                            CANInterfaceType_e::INVERTER);
        process_ring_buffer(VCRCANInterfaceInstance::instance().telem_can_rx_buffer,
                            CANInterfacesInstance::instance(),
                            sys_time::hal_millis(),
                            VCRCANInterfaceInstance::instance().can_recv_switch,
                            CANInterfaceType_e::TELEM);
        process_ring_buffer(VCRCANInterfaceInstance::instance().rear_aux_can_rx_buffer,
                            CANInterfacesInstance::instance(),
                            sys_time::hal_millis(),
                            VCRCANInterfaceInstance::instance().can_recv_switch,
                            CANInterfaceType_e::RAUX);
    }

    void handle_async_recvs()
    {
        // ethernet, etc...

        handle_async_CAN_receive();
    }

    VCRInterfaceData_s gather_latest_interface_data(CANInterfaces_s &can_interfaces)
    {
        VCRInterfaceData_s ret;

        auto vcf_data = can_interfaces.vcf_interface.get_latest_data();
        auto acu_data = can_interfaces.acu_interface.get_latest_data(sys_time::hal_millis());
        auto drivebrain_telem_data = can_interfaces.db_interface.get_latest_telem_drivebrain_command();
        auto drivebrain_auxillary_data = can_interfaces.db_interface.get_latest_auxillary_drivebrain_command();

        auto fl_inv_mechanics = can_interfaces.fl_inverter_interface.get_motor_mechanics();
        auto fr_inv_mechanics = can_interfaces.fr_inverter_interface.get_motor_mechanics();
        auto rl_inv_mechanics = can_interfaces.rl_inverter_interface.get_motor_mechanics();
        auto rr_inv_mechanics = can_interfaces.rr_inverter_interface.get_motor_mechanics();

        auto fl_inv_status = can_interfaces.fl_inverter_interface.get_status();
        auto fr_inv_status = can_interfaces.fr_inverter_interface.get_status();
        auto rl_inv_status = can_interfaces.rl_inverter_interface.get_status();
        auto rr_inv_status = can_interfaces.rr_inverter_interface.get_status();

        ret.inverter_data.FL.speed_rpm = fl_inv_mechanics.actual_speed;
        ret.inverter_data.FR.speed_rpm = fr_inv_mechanics.actual_speed;
        ret.inverter_data.RL.speed_rpm = rl_inv_mechanics.actual_speed;
        ret.inverter_data.RR.speed_rpm = rr_inv_mechanics.actual_speed;

        ret.inverter_data.FL.dc_bus_voltage = fl_inv_status.dc_bus_voltage;
        ret.inverter_data.FR.dc_bus_voltage = fr_inv_status.dc_bus_voltage;
        ret.inverter_data.RL.dc_bus_voltage = rl_inv_status.dc_bus_voltage;
        ret.inverter_data.RR.dc_bus_voltage = rr_inv_status.dc_bus_voltage;

        ret.recvd_pedals_data = vcf_data.stamped_pedals;
        ret.front_loadcell_data = vcf_data.front_loadcell_data;
        ret.front_suspot_data = vcf_data.front_suspot_data;
        ret.dash_input_state = vcf_data.dash_input_state;
        ret.latest_drivebrain_telem_command = drivebrain_telem_data;
        ret.latest_drivebrain_auxillary_command = drivebrain_auxillary_data;

        return ret;
    }

    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handle_async_recvs();

        bool torque_mode_cycle_button_was_pressed = VCFInterfaceInstance::instance().get_latest_data().dash_input_state.BUTTON_2;

        VCRInterfaceData_s new_interface_data = gather_latest_interface_data(CANInterfacesInstance::instance());

        vcr_data.system_data.drivetrain_data.measuredSpeeds = {
            new_interface_data.inverter_data.FL.speed_rpm,
            new_interface_data.inverter_data.FR.speed_rpm,
            new_interface_data.inverter_data.RL.speed_rpm,
            new_interface_data.inverter_data.RR.speed_rpm
        };

        vcr_data.system_data.drivetrain_data.measuredInverterFLPackVoltage = new_interface_data.inverter_data.FL.dc_bus_voltage;

        if (torque_mode_cycle_button_was_pressed && !new_interface_data.dash_input_state.BUTTON_2)
        {
            VCRControlsInstance::instance().cycle_torque_limit();
            VCFInterfaceInstance::instance().enqueue_torque_mode_LED_message(VCRControlsInstance::instance().get_current_torque_limit());
        }

        vcr_data.system_data.tc_mux_status = VCRControlsInstance::instance().get_tc_mux_status();
        vcr_data.system_data.vehicle_state_machine_state = VehicleStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());
        vcr_data.system_data.drivetrain_state_machine_state = DrivetrainInstance::instance().get_state();
        vcr_data.interface_data = new_interface_data;
        vcr_data.system_data.db_cntrl_status.drivebrain_is_in_control = VCRControlsInstance::instance().drivebrain_is_in_control();
        vcr_data.system_data.db_cntrl_status.drivebrain_controller_timing_failure = VCRControlsInstance::instance().drivebrain_timing_failure();

        return HT_TASK::TaskResponse::YIELD;
    }
}

HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Serial.println("time\t:\taccel\t:\tbrake");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.last_recv_millis);
    // Serial.print("\t:\t");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.pedals_data.accel_percent);
    // Serial.print("\t:\t");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.pedals_data.brake_percent);
    // Serial.println();
    // Serial.print("pedals heartbeat good: "); Serial.print(vcr_data.interface_data.recvd_pedals_data.heartbeat_ok);
    // Serial.println();
    // Serial.print("steering heartbeat good: "); Serial.print(vcr_data.interface_data.recvd_steering_data.heartbeat_ok);
    // Serial.println();
    // Serial.print("Pedals Brake Is Active: "); Serial.print(VCFInterfaceInstance::instance().is_brake_pressed() ? "YES" : "NO");
    // Serial.println();
    // Serial.print("Is Start Button Active: "); Serial.print(VCFInterfaceInstance::instance().is_start_button_pressed() ? "YES" : "NO");
    // Serial.println();


    // Serial.println();
    // Serial.println();

    // Serial.print("Drivetrain system state: ");
    // Serial.println(static_cast<int>(DrivetrainInstance::instance().get_state()));
    // Serial.print("Diagnostic FL #: ");
    // Serial.print(DrivetrainInstance::instance().get_status().inverter_statuses.FL.diagnostic_number);
    // Serial.print(" FR #: ");
    // Serial.print(DrivetrainInstance::instance().get_status().inverter_statuses.FR.diagnostic_number);
    // Serial.print(" RL #: ");
    // Serial.print(DrivetrainInstance::instance().get_status().inverter_statuses.RL.diagnostic_number);
    // Serial.print(" RR #: ");
    // Serial.println(DrivetrainInstance::instance().get_status().inverter_statuses.RR.diagnostic_number);

    // Serial.print("Vehicle state machine state: ");
    // Serial.println(static_cast<int>(VehicleStateMachineInstance::instance().get_state()));
    // Serial.println();
    // Serial.print("launch controller state: ");
    // Serial.println(static_cast<int>(VCRControlsInstance::instance().get_launch_controller().get_launch_state()));

    // Serial.print("Start button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.start_btn_is_pressed);

    // Serial.print("pedal recalibrate button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.preset_btn_is_pressed);

    // Serial.print("mc reset button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.mc_reset_btn_is_pressed);

    // Serial.print("torque mode cycle button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.mode_btn_is_pressed);

    // Serial.println("IOExpander testing");
    // auto& s = vcr_data.interface_data.shutdown_sensing_data;
    // char buf[128]; //NOLINT is debug
    // snprintf(buf, sizeof(buf),
    //     "%-10s %-14s %-13s %-10s %-8s %-6s",
    //     "BSPD OK", "BSPD MISSING", "BSPD FAULTED", "VCR SW OK", "BMS OK", "IMD OK"
    // );
    // Serial.println(buf);

    // snprintf(buf, sizeof(buf),
    //     "%-10d %-14d %-13d %-10d %-8d %-6d",
    //     s.bspd_is_ok, s.bspd_missing, s.bspd_fault,
    //     s.watchdog_is_ok, s.bms_is_ok, s.imd_is_ok
    // );
    // Serial.println(buf);
    // Serial.println();

    // auto& e = vcr_data.interface_data.ethernet_is_linked;

    // snprintf(buf, sizeof(buf),
    //     "%-14s %-10s %-10s %-14s %-12s %-6s",
    //     "ACU LINK", "DB LINK", "VCF LINK", "TEENSY LINK", "DEBUG LINK", "UBIQUITI LINK"
    // );
    // Serial.println(buf);

    // snprintf(buf, sizeof(buf),
    //     "%-14d %-10d %-10d %-14d %-12d %-6d",
    //     e.acu_link, e.drivebrain_link, e.vcf_link,
    //     e.teensy_link, e.debug_link, e.ubiquiti_link
    // );
    // Serial.println(buf);
    // Serial.println();

    // Serial.print("Load Cell RR: ");
    // Serial.println(vcr_data.interface_data.rear_loadcell_data.RR_loadcell_analog);

    // Serial.print("Load Cell RL: ");
    // Serial.println(vcr_data.interface_data.rear_loadcell_data.RL_loadcell_analog);

    // Serial.print("SusPot RR: ");
    // Serial.println(vcr_data.interface_data.rear_suspot_data.RR_sus_pot_analog);

    // Serial.print("SusPot RL: ");
    // Serial.println(vcr_data.interface_data.rear_suspot_data.RL_sus_pot_analog);

    // /* Drivebrain data */
    // Serial.print("Latest Drivebrain data: ");
    // Serial.print(vcr_data.interface_data.inverter_data.FL.commanded_torque);
    // Serial.print(" ");
    // Serial.print(vcr_data.interface_data.inverter_data.FR.commanded_torque);
    // Serial.print(" ");
    // Serial.print(vcr_data.interface_data.inverter_data.RL.commanded_torque);
    // Serial.print(" ");
    // Serial.println(vcr_data.interface_data.inverter_data.RR.commanded_torque);

    // Serial.println("desired speeds, torq lim");
    // Serial.print("FL:   ");
    // Serial.print(VCRControlsInstance::instance()._debug_dt_command.desired_speeds.FL); Serial.print(" ");
    // Serial.println(VCRControlsInstance::instance()._debug_dt_command.torque_limits.FL);

    // Serial.print("FR:   ");
    // Serial.print(VCRControlsInstance::instance()._debug_dt_command.desired_speeds.FR); Serial.print(" ");
    // Serial.println(VCRControlsInstance::instance()._debug_dt_command.torque_limits.FR);

    // Serial.print("RL:   ");
    // Serial.print(VCRControlsInstance::instance()._debug_dt_command.desired_speeds.RL); Serial.print(" ");
    // Serial.println(VCRControlsInstance::instance()._debug_dt_command.torque_limits.RL);

    // Serial.print("RR:   ");
    // Serial.print(VCRControlsInstance::instance()._debug_dt_command.desired_speeds.RR); Serial.print(" ");
    // Serial.println(VCRControlsInstance::instance()._debug_dt_command.torque_limits.RR);

    // Serial.print("Current Controller Mode: ");
    // Serial.println(static_cast<uint8_t>(vcr_data.interface_data.dash_input_state.dial_state));

    /* Thermistor Data */
    // Serial.print("Thermistor 0 Analog: ");

    // Serial.println(ADCInterfaceInstance::instance().read_thermistor_0().conversion);
    // Serial.print(vcr_data.interface_data.thermistor_data.thermistor_0.thermistor_analog);
    // Serial.print(" Thermistor 0 degrees C: ");
    // Serial.println(vcr_data.interface_data.thermistor_data.thermistor_0.thermistor_degrees_C);
    // Serial.print("Thermistor 4 Analog: ");
    // Serial.print(vcr_data.interface_data.thermistor_data.thermistor_4.thermistor_analog);
    // Serial.print(" Thermistor 4 degrees C: ");
    // Serial.println(vcr_data.interface_data.thermistor_data.thermistor_4.thermistor_degrees_C);
    // Serial.print("Thermistor 5 Analog: ");
    // Serial.print(vcr_data.interface_data.thermistor_data.thermistor_5.thermistor_analog);
    // Serial.print(" Thermistor 5 degrees C: ");
    // Serial.println(vcr_data.interface_data.thermistor_data.thermistor_5.thermistor_degrees_C);
    // Serial.print("Thermistor 6 Analog: ");
    // Serial.print(vcr_data.interface_data.thermistor_data.thermistor_6.thermistor_analog);
    // Serial.print(" Thermistor 6 degrees C: ");
    // Serial.println(vcr_data.interface_data.thermistor_data.thermistor_6.thermistor_degrees_C);
    // Serial.print("Thermistor 7 Analog: ");
    // Serial.print(vcr_data.interface_data.thermistor_data.thermistor_7.thermistor_analog);
    // Serial.print(" Thermistor 7 degrees C: ");
    // Serial.println(vcr_data.interface_data.thermistor_data.thermistor_7.thermistor_degrees_C);

    Serial.println();

    return HT_TASK::TaskResponse::YIELD;
}