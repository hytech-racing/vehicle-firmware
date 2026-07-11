#include "CCU_InterfaceTasks.h"

void initialize_all_interfaces()
{
    analogReadResolution(CCUInterfaces::ANALOG_READ_RESOLUTION);

    /* ADC Interface */
    ADCInterfaceInstance::create(ADCPinout_s {
                                    CCUInterfaces::SHDN_A_PIN,
                                    CCUInterfaces::SHDN_B_PIN,
                                    CCUInterfaces::SHDN_C_PIN,
                                    CCUInterfaces::SHDN_D_PIN,
                                    CCUInterfaces::SHDN_E_PIN,
                                    CCUInterfaces::SHDN_F_PIN,
                                    CCUInterfaces::SHDN_G_PIN,
                                    CCUInterfaces::SCALED_24V_PIN,
                                    CCUInterfaces::CONTROL_PILOT_PIN,
                                    CCUInterfaces::PROXIMITY_PILOT_PIN,
                                    CCUInterfaces::TEENSY_240_ENABLED_PIN,
                                    CCUInterfaces::TEENSY_240_OK_PIN,
                                    CCUInterfaces::JUMPER_OUT_PIN,
                                    CCUInterfaces::BUTTON2_READ_PIN,
                                },
                                ADCConversions_s {
                                    CCUInterfaces::GLV_CONV_FACTOR,
                                    CCUInterfaces::CONTROL_PILOT_CONV_FACTOR,
                                    CCUInterfaces::PROXIMITY_PILOT_CONV_FACTOR,
                                    CCUInterfaces::JUMPER_OUT_CONV_FACTOR
                                },
                                CCUInterfaces::BIT_RESOLUTION
    );
    ADCInterfaceInstance::instance().init(sys_time::hal_millis());

    /* CAN Interfaces  */
    CANInterfacesInstance::create(ACUInterfaceInstance::instance(), ChargerInterfaceInstance::instance(), EnergyMeterInterfaceInstance::instance());

    /* Charger Interface */
    ChargerInterface(ACUInterfaceInstance::instance());

    /* Display Interface */
    DisplayInterfaceInstance::create(DisplayPinout_s {
                                        CCUInterfaces::LCD_CS_PIN,
                                        CCUInterfaces::LCD_SCK_PIN,
                                        CCUInterfaces::LCD_MISO_PIN,
                                        CCUInterfaces::LCD_MOSI_PIN,
                                        CCUInterfaces::LCD_RESET_PIN,
                                        CCUInterfaces::LCD_DC_PIN,
                                        CCUInterfaces::BUTTON1_READ_PIN,
                                    }
    );
    DisplayInterfaceInstance::instance().init();

    /* Rotary Encoder Interface */
    RotaryEncoderInterfaceInstance::create(RotaryEncoderPinout_s {
                                            CCUInterfaces::ENC_SWITCH_PIN,
                                            CCUInterfaces::ENC_A_PIN,
                                            CCUInterfaces::ENC_B_PIN,
                                        }
    );
    RotaryEncoderInterfaceInstance::instance().init();

    /* Level2 Interface */
    Level2InterfaceInstance::create(Level2_Pinout_s {
                                        CCUInterfaces::CONTROL_PWM_SENSE_PIN,
                                        CCUInterfaces::START_CHARGE_PIN
                                    }
    );
    Level2InterfaceInstance::instance().init();

    /* Watchdog Interface */
    WatchdogInterfaceInstance::create(WatchdogPinout_s {
                                        CCUInterfaces::WATCHDOG_KICK_PIN,
                                        CCUInterfaces::SOFTWARE_OK_PIN
                                    }
    );
    WatchdogInterfaceInstance::instance().init();

    // CCUEthernetInterface::create();
    // CCUEthernetInterface::instance().init_ethernet_device();
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInterfaceInstance::instance().update_watchdog_state(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_read_encoder_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    RotaryEncoderInterfaceInstance::instance().tick(sys_time::hal_millis());

    if (RotaryEncoderInterfaceInstance::instance().switch_pressed())
    {
        RotaryEncoderInterfaceInstance::instance().set_value(0);
    }

    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse handle_enqueue_acu_can_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUInterfaceInstance::instance().enqueue_ccu_status_data();
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse handle_enqueue_charger_can_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ChargerInterfaceInstance::instance().enqueue_charging_data(ACUInterfaceInstance::instance(), MainChargeSystemInstance::instance().get_charge_current());
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse run_send_ethernet(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse run_receive_ethernet(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse handle_send_all_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CCUCANInterfaceImpl::send_all_CAN_msgs(CCUCANInterfaceInstance::instance().acu_can_tx_buffer, &CCUCANInterfaceInstance::instance().ACU_CAN);
    CCUCANInterfaceImpl::send_all_CAN_msgs(CCUCANInterfaceInstance::instance().charger_can_tx_buffer, &CCUCANInterfaceInstance::instance().CHARGER_CAN);
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse sample_can_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    process_ring_buffer(CCUCANInterfaceInstance::instance().acu_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), CCUCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::ACU);
    process_ring_buffer(CCUCANInterfaceInstance::instance().charger_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), CCUCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::CHARGER);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_update_display_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    auto curr_time_ms = sys_time::hal_millis();
    DisplayInterfaceInstance::instance().update(curr_time_ms);
    DisplayInterfaceInstance::instance().display_data(curr_time_ms, Level2SystemInstance::instance().is_120_switched(ADCInterfaceInstance::instance()));
    DisplayInterfaceInstance::instance().refresh_display_data(curr_time_ms);
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse debug_prints(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // const auto& acu_data = ACUInterfaceInstance::instance().get_latest_data();
    // const auto& charger_data = ChargerInterfaceInstance::instance().get_latest_charger_data();
    // const auto& level2_data = Level2InterfaceInstance::instance().get_level_2_data();

    /* ----- General Status ----- */
    //Serial.println(Level2SystemInstance::instance().is_120_switched(ADCInterfaceInstance::instance()) ? "Set to 120 V Charging" : "Set to 240 V Charging");
    // Serial.print("ACU STATE      : "); Serial.println(static_cast<int>(ACUInterfaceInstance::instance().get_latest_data().acu_state));
    // Serial.print("Charging State : "); Serial.println(static_cast<size_t>(ChargerStateMachineInstance::instance().get_state()));
    // Serial.println();

    //Serial.print("READ JUMPER OUT: "); Serial.println(ADCInterfaceInstance::instance().read_jumper_out());
    //Serial.print("READ 240 OK: "); Serial.println(ADCInterfaceInstance::instance().read_240_ok());
    //Serial.print("READ 240 ENABLED: "); Serial.println(ADCInterfaceInstance::instance().read_240_enabled());
    //Serial.println();


    /* ----- Voltage Information ----- */
    // Serial.print("Cell Voltage Max   : "); Serial.println(acu_data.high_voltage);
    // Serial.print("Cell Voltage Min   : "); Serial.println(acu_data.low_voltage);
    // Serial.print("Cell Voltage Avg   : "); Serial.println(acu_data.average_voltage);
    // Serial.print("Cell Voltage Delta : "); Serial.println(acu_data.high_voltage - acu_data.low_voltage);
    // Serial.print("Pack Voltage       : "); Serial.println(acu_data.pack_voltage);
    // Serial.println();


    /* ----- Temperature Information ----- */
    // Serial.print("Max Cell Temp      : "); Serial.println(acu_data.max_cell_temp);
    // Serial.print("Min Cell Temp      : "); Serial.println(acu_data.min_cell_temp);
    // Serial.print("Max Board Temp     : "); Serial.println(acu_data.max_board_temp);
    // Serial.println();


    /* ----- Charge Current Information ----- */
    // Serial.print("Charger Current Actual   : "); Serial.println(charger_data.output_current_low);
    // Serial.print("Calc Charge Current      : "); Serial.println(MainChargeSystemInstance::instance().get_charge_current());
    // Serial.println();


    /* ----- Charge Information ----- */
    // Serial.print("CP PWM  "); Serial.print(level2_data.control_pwm); Serial.print(" V "); Serial.print(level2_data.control_pwm_duty_cycle); Serial.println("%");
    // Serial.print("CP Voltage Sense      "); Serial.println(ADCInterfaceInstance::instance().read_control_pilot());
    // Serial.print("PP Voltage Sense      "); Serial.println(ADCInterfaceInstance::instance().read_proximity_pilot());
    // Serial.println();


    /* ----- SHDN Information ----- */
    // Serial.print("SHDN_A : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_A_voltage() ? "HIGH" : "LOW");
    // Serial.print("SHDN_B : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_B_voltage() ? "HIGH" : "LOW");
    // Serial.print("SHDN_C : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_C_voltage() ? "HIGH" : "LOW");
    // Serial.print("SHDN_D : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_D_voltage() ? "HIGH" : "LOW");
    // Serial.print("SHDN_E : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_E_voltage() ? "HIGH" : "LOW");
    // Serial.print("SHDN_F : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_F_voltage() ? "HIGH" : "LOW");
    // Serial.print("SHDN_G : "); Serial.println(ADCInterfaceInstance::instance().read_shdn_G_voltage() ? "HIGH" : "LOW");
    // Serial.println();


    /* ----- Rotary Encoder ----- */
    // Serial.print("Rotary Encoder Value: ");
    // Serial.println(RotaryEncoderInterfaceInstance::instance().get_value(), 2);
    // Serial.println();


    /* ----- ACU Detailed Cell Voltages ----- */
    // for (int c = 0; c < default_acu_params::NUM_CELLS; c++)
    // {
    //     Serial.print("C"); Serial.print(c); Serial.print(": ");
    //     if (*acu_data.cell_voltages[c])
    //     {
    //         Serial.print(*acu_data.cell_voltages[c], 3);
    //     }
    //     else
    //     {
    //         Serial.print("--.-");
    //     }
    //     Serial.print("\t");
    // }


    /* ----- ACU Detailed Cell Temps ----- */
    // for (int c = 0; c < default_acu_params::NUM_CELL_TEMPS; c++)
    // {
    //     Serial.print("CT"); Serial.print(c); Serial.print(": ");
    //     if (*acu_data.cell_temps[c])
    //     {
    //         Serial.print(*acu_data.cell_temps[c], 3);
    //     }
    //     else
    //     {
    //         Serial.print("--.-");
    //     }
    //     Serial.print("\t");
    // }


    /* ----- ACU Detailed Board Temps ----- */
    // for (int c = 0; c < default_acu_params::NUM_BOARD_TEMPS; c++)
    // {
    //     Serial.print("BT"); Serial.print(c); Serial.print(": ");
    //     if (*acu_data.board_temps[c])
    //     {
    //         Serial.print(*acu_data.board_temps[c], 3);
    //     }
    //     else
    //     {
    //         Serial.print("--.-");
    //     }
    //     Serial.print("\t");
    // }

   return HT_TASK::TaskResponse::YIELD;
}
