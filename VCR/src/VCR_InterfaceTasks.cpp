#include "VCR_InterfaceTasks.h"


void initializeAllInterfaces()
{
    Serial.begin(VCRInterfaces::SERIAL_BAUDRATE);
    analogReadResolution(VCRInterfaces::ANALOG_RESOLUTION);

    SPI.begin();

    /* ---------- Pin Setup ----------*/
    // Should these be in a specific interface?
    pinMode(VCRInterfaces::MOTOR_COOLING_CONTROL_PIN, OUTPUT);
    pinMode(VCRInterfaces::INVERTER_COOLING_CONTROL_PIN, OUTPUT);
    // pinMode(VCRInterfaces::INVERTER_ENABLE_PIN, OUTPUT); do we have this pin now?
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
                                        IOExpanderParams_s {
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
    VCREthernetInterfaceInstance::instance().initEthernetDevice();

    /* ---------- CAN Interfaces ---------- */
    CANInterfacesInstance::create(ACUInterfaceInstance::instance(),
                                DrivebrainInterfaceInstance::instance(),
                                fl_inverter_interface,
                                fr_inverter_interface,
                                rl_inverter_interface,
                                rr_inverter_interface,
                                VCFInterfaceInstance::instance()
    );

    VCRCANInterfaceInstance::create(etl::delegate<void(CANInterfaces_s&, const CAN_message_t&, unsigned long, CANInterfaceType_e)>::create<VCRCANInterfaceImpl::receieveIDSwitch>());

    handle_CAN_setup(VCRCANInterfaceInstance::instance().INVERTER_CAN, VCRConstants::INVERTER_CAN_BAUDRATE, &VCRCANInterfaceImpl::onINVERTERCANReceive);
    handle_CAN_setup(VCRCANInterfaceInstance::instance().TELEM_CAN, VCRConstants::TELEM_CAN_BAUDRATE, &VCRCANInterfaceImpl::onTELEMCANReceive);
    handle_CAN_setup(VCRCANInterfaceInstance::instance().REAR_AUX_CAN, VCRConstants::RAUX_CAN_BAUDRATE, &VCRCANInterfaceImpl::onRAUXCANReceive);
}

HT_TASK::TaskResponse readADC0Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick_adc0();
    ADCInterfaceInstance::instance().update_filtered_values(VCRInterfaces::LOADCELL_IIR_FILTER_ALPHA);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse readADC1Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick_adc1();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse updateACUHeartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUCANInterfaceData_s data = ACUInterfaceInstance::instance().getLatestData(sys_time::hal_millis());
    digitalWrite(VCRInterfaces::SOFTWARE_OK_PIN, data.is_heartbeat_ok);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse kickWatchdogTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInterfaceInstance::instance().update_watchdog_state(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse readIOExpanderTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    IOExpanderInterfaceInstance::instance().updatePortAData();
    IOExpanderInterfaceInstance::instance().updatePortBData();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse updateBrakelightTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(VCRInterfaces::BRAKELIGHT_CONTROL_PIN, VCFInterfaceInstance::instance().isBrakePressed());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enableMotorCoolingTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VehicleState_e vehicle_state = VehicleStateMachineInstance::instance().get_state(); //NOLINT will alway be populated so its ok
    bool enable_state = vehicle_state == VehicleState_e::READY_TO_DRIVE ||
                        VCFInterfaceInstance::instance().getLatestData().dash_input_state.dial_state == ControllerMode_e::MODE_2 ||
                        VCFInterfaceInstance::instance().getLatestData().dash_input_state.dial_state == ControllerMode_e::MODE_3;
    digitalWrite(VCRInterfaces::MOTOR_COOLING_CONTROL_PIN, enable_state ? HIGH : LOW);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enableInverterCoolingTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VehicleState_e vehicle_state = VehicleStateMachineInstance::instance().get_state(); //NOLINT will alway be populated so its ok
    bool enable_state = vehicle_state == VehicleState_e::TRACTIVE_SYSTEM_ACTIVE ||
                        vehicle_state == VehicleState_e::READY_TO_DRIVE ||
                        VCFInterfaceInstance::instance().getLatestData().dash_input_state.dial_state == ControllerMode_e::MODE_2 ||
                        VCFInterfaceInstance::instance().getLatestData().dash_input_state.dial_state == ControllerMode_e::MODE_5;
    digitalWrite(VCRInterfaces::INVERTER_COOLING_CONTROL_PIN, enable_state ? HIGH : LOW);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueSuspensionCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo )
{
    DrivebrainInterfaceInstance::instance().handleEnqueueSuspensionCANData(ADCInterfaceInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueFlowmeterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
  DrivebrainInterfaceInstance::instance().handleEnqueueFlowmeterCANData(FlowmeterInterfaceInstance::instance(), millis());
  return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueCoolantTempCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handleEnqueueCoolantTempCANData(ADCInterfaceInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueControlsCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRControlsInstance::instance().enqueueLatencyCANData();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueInverterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    fl_inverter_interface.send_DRIVE_ENABLE();
    // fl_inverter_interface.send_AC_CURRENT();
    // fl_inverter_interface.send_BRAKE_CURRENT();

    fr_inverter_interface.send_DRIVE_ENABLE();
    // fr_inverter_interface.send_AC_CURRENT();
    // fr_inverter_interface.send_BRAKE_CURRENT();

    rl_inverter_interface.send_DRIVE_ENABLE();
    // rl_inverter_interface.send_AC_CURRENT();
    // rl_inverter_interface.send_BRAKE_CURRENT();

    rr_inverter_interface.send_DRIVE_ENABLE();
    // rr_inverter_interface.send_AC_CURRENT();
    // rr_inverter_interface.send_BRAKE_CURRENT();

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueueVehicleStateCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFInterfaceInstance::instance().enqueueVehicleStateCANMessage(VehicleStateMachineInstance::instance().get_state(),
                                                                DrivetrainInstance::instance().getCurrentState(),
                                                                VCRControlsInstance::instance().isDrivebrainInControll()
    );
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handleSendAllCANData(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRCANInterfaceImpl::sendAllCANMsgs(VCRCANInterfaceInstance::instance().inverter_can_tx_buffer, &VCRCANInterfaceInstance::instance().INVERTER_CAN);
    VCRCANInterfaceImpl::sendAllCANMsgs(VCRCANInterfaceInstance::instance().telem_can_tx_buffer, &VCRCANInterfaceInstance::instance().TELEM_CAN);
    VCRCANInterfaceImpl::sendAllCANMsgs(VCRCANInterfaceInstance::instance().rear_aux_can_tx_buffer, &VCRCANInterfaceInstance::instance().REAR_AUX_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse sendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handleSendEthernetData(
        VCREthernetInterfaceInstance::instance().makeVCRDataPBMsg(ADCInterfaceInstance::instance(),
                                                                vcr_data.system_data.drivetrain_data,
                                                                VCFInterfaceInstance::instance(),
                                                                VehicleStateMachineInstance::instance(),
                                                                DrivetrainInstance::instance(),
                                                                fl_inverter_interface,
                                                                fr_inverter_interface,
                                                                rl_inverter_interface,
                                                                rr_inverter_interface,
                                                                VCRControlsInstance::instance()
        )
    );
    return HT_TASK::TaskResponse::YIELD;
}

namespace async_tasks
{
    void handleCANReceive()
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

    VCRInterfaceData_s gather_latest_interface_data(CANInterfaces_s &can_interfaces)
    {
        VCRInterfaceData_s out;

        auto vcf_data = can_interfaces.vcf_interface.getLatestData();
        auto acu_data = can_interfaces.acu_interface.getLatestData(sys_time::hal_millis());
        auto drivebrain_telem_data = can_interfaces.db_interface.getLatestDrivebrainCommandTELEM();
        auto drivebrain_raux_data = can_interfaces.db_interface.getLatestDrivebrainCommandRAUX();

        auto fl_inv_mechanics = can_interfaces.fl_inverter_interface.getMotorMechanics();
        auto fr_inv_mechanics = can_interfaces.fr_inverter_interface.getMotorMechanics();
        auto rl_inv_mechanics = can_interfaces.rl_inverter_interface.getMotorMechanics();
        auto rr_inv_mechanics = can_interfaces.rr_inverter_interface.getMotorMechanics();

        auto fl_inv_status = can_interfaces.fl_inverter_interface.getStatus();
        auto fr_inv_status = can_interfaces.fr_inverter_interface.getStatus();
        auto rl_inv_status = can_interfaces.rl_inverter_interface.getStatus();
        auto rr_inv_status = can_interfaces.rr_inverter_interface.getStatus();

        // Telemetry: full data + limits per corner, now that InverterInterface exposes them
        out.inverter_data.FL = can_interfaces.fl_inverter_interface.getTelemetryData();
        out.inverter_data.FR = can_interfaces.fr_inverter_interface.getTelemetryData();
        out.inverter_data.RL = can_interfaces.rl_inverter_interface.getTelemetryData();
        out.inverter_data.RR = can_interfaces.rr_inverter_interface.getTelemetryData();

        out.inverter_limits.FL = can_interfaces.fl_inverter_interface.getLimitsData();
        out.inverter_limits.FR = can_interfaces.fr_inverter_interface.getLimitsData();
        out.inverter_limits.RL = can_interfaces.rl_inverter_interface.getLimitsData();
        out.inverter_limits.RR = can_interfaces.rr_inverter_interface.getLimitsData();

        out.inverter_data.FL.speed_rpm = fl_inv_mechanics.actual_speed_rpm;
        out.inverter_data.FR.speed_rpm = fr_inv_mechanics.actual_speed_rpm;
        out.inverter_data.RL.speed_rpm = rl_inv_mechanics.actual_speed_rpm;
        out.inverter_data.RR.speed_rpm = rr_inv_mechanics.actual_speed_rpm;

        out.inverter_data.FL.input_voltage = fl_inv_status.dc_bus_voltage;
        out.inverter_data.FR.input_voltage = fr_inv_status.dc_bus_voltage;
        out.inverter_data.RL.input_voltage = rl_inv_status.dc_bus_voltage;
        out.inverter_data.RR.input_voltage = rr_inv_status.dc_bus_voltage;

        out.recvd_pedals_data = vcf_data.stamped_pedals;
        out.front_loadcell_data = vcf_data.front_loadcell_data;
        out.front_suspot_data = vcf_data.front_suspot_data;
        out.dash_input_state = vcf_data.dash_input_state;
        out.latest_drivebrain_telem_command = drivebrain_telem_data;
        out.latest_drivebrain_auxillary_command = drivebrain_raux_data;

        return out;
    }

    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handleCANReceive();

        VCRInterfaceData_s new_interface_data = gather_latest_interface_data(CANInterfacesInstance::instance());

        vcr_data.system_data.drivetrain_data.measured_speeds = {
            new_interface_data.inverter_data.FL.speed_rpm,
            new_interface_data.inverter_data.FR.speed_rpm,
            new_interface_data.inverter_data.RL.speed_rpm,
            new_interface_data.inverter_data.RR.speed_rpm
        };

        vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.FL = new_interface_data.inverter_data.FL.input_voltage;
        vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.FR = new_interface_data.inverter_data.FR.input_voltage;
        vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.RL = new_interface_data.inverter_data.RL.input_voltage;
        vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.RR = new_interface_data.inverter_data.RR.input_voltage;

        vcr_data.system_data.tc_mux_status = VCRControlsInstance::instance().getTCMuxStatus();
        vcr_data.system_data.vehicle_state_machine_state = VehicleStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());
        vcr_data.system_data.drivetrain_state_machine_state = DrivetrainInstance::instance().getCurrentState();
        vcr_data.interface_data = new_interface_data;
        vcr_data.system_data.db_cntrl_status.drivebrain_is_in_control = VCRControlsInstance::instance().isDrivebrainInControll();
        vcr_data.system_data.db_cntrl_status.drivebrain_controller_timing_failure = VCRControlsInstance::instance().getHasTimingFailure();

        return HT_TASK::TaskResponse::YIELD;
    }
}

HT_TASK::TaskResponse debugPrintTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
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