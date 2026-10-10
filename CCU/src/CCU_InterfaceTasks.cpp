#include "CCU_InterfaceTasks.hpp"


void initializeAllInterfaces()
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

     /* CAN Interfaces  */
    CANInterfacesInstance::create(ACUInterfaceInstance::instance(),
                                ChargerInterfaceInstance::instance(),
                                EnergyMeterInterfaceInstance::instance()
    );

    CCUCANInterfaceInstance::create(etl::delegate<void(CANInterfaces_s&, const CAN_message_t&, unsigned long, CANInterfaceType_e)>::create<CCUCANInterfaceImpl::CCUReceiveIDSwitch>());

    handle_CAN_setup(CCUCANInterfaceInstance::instance().ACU_CAN, CCUConstants::ACU_CAN_BAUDRATE, &CCUCANInterfaceImpl::onACUCANReceive);
    handle_CAN_setup(CCUCANInterfaceInstance::instance().CHARGER_CAN, CCUConstants::CHARGER_CAN_BAUDRATE, &CCUCANInterfaceImpl::onChargerCANReceive);
}

HT_TASK::TaskResponse kickWatchdogTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInterfaceInstance::instance().updateWatchdogState(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse readEncoderTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    RotaryEncoderInterfaceInstance::instance().tick(sys_time::hal_millis());

    if (RotaryEncoderInterfaceInstance::instance().isSwitchPressed())
    {
        RotaryEncoderInterfaceInstance::instance().setValue(0);
    }

    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse enqueueACUCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUInterfaceInstance::instance().enqueueCCUStatusCANMsg();
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse enqueueChargerCANdataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ChargerInterfaceInstance::instance().enqueueChargingCANMsg(ACUInterfaceInstance::instance(), MainChargeSystemInstance::instance().get_charge_current());
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse sendETHTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse receiveETHTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse clearAllCANBuffersTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CCUCANInterfaceImpl::clearAllCANBuffers(CCUCANInterfaceInstance::instance().acu_can_tx_buffer, &CCUCANInterfaceInstance::instance().ACU_CAN);
    CCUCANInterfaceImpl::clearAllCANBuffers(CCUCANInterfaceInstance::instance().charger_can_tx_buffer, &CCUCANInterfaceInstance::instance().CHARGER_CAN);
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse sampleCANTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    process_ring_buffer(CCUCANInterfaceInstance::instance().acu_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), CCUCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::ACU);
    process_ring_buffer(CCUCANInterfaceInstance::instance().charger_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), CCUCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::CHARGER);
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse updateDisplayTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    auto curr_time_ms = sys_time::hal_millis();
    DisplayInterfaceInstance::instance().update(curr_time_ms);
    DisplayInterfaceInstance::instance().displayData(curr_time_ms, Level2SystemInstance::instance().is_120_switched(ADCInterfaceInstance::instance()));
    DisplayInterfaceInstance::instance().refreshDisplayData(curr_time_ms);
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse debug_prints(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    const auto acu_data = ACUInterfaceInstance::instance().getLatestData();
    const auto charger_data = ChargerInterfaceInstance::instance().get_latest_charger_data();
    const auto level2_data = Level2InterfaceInstance::instance().getLevel2Data();
    auto& adc = ADCInterfaceInstance::instance();

    /* ----- General Status ----- */
    Serial.println(Level2SystemInstance::instance().is_120_switched(adc) ? "Set to 120 V Charging" : "Set to 240 V Charging");
    Serial.print("ACU State          : "); Serial.println(static_cast<int>(acu_data.acu_state));
    Serial.print("Charging State     : "); Serial.println(static_cast<int>(ChargerStateMachineInstance::instance().getState()));
    Serial.println();

    Serial.print("Jumper Out         : "); Serial.println(adc.readJumperOut());
    Serial.print("240 OK             : "); Serial.println(adc.read240OK());
    Serial.print("240 Enabled        : "); Serial.println(adc.read240Enabled());
    Serial.println();


    /* ----- Voltage Information ----- */
    Serial.print("Cell Voltage Max   : "); Serial.println(acu_data.high_voltage);
    Serial.print("Cell Voltage Min   : "); Serial.println(acu_data.low_voltage);
    Serial.print("Cell Voltage Avg   : "); Serial.println(acu_data.average_voltage);
    Serial.print("Cell Voltage Delta : "); Serial.println(acu_data.high_voltage - acu_data.low_voltage);
    Serial.print("Pack Voltage       : "); Serial.println(acu_data.pack_voltage);
    Serial.println();


    /* ----- Temperature Information ----- */
    Serial.print("Max Cell Temp      : "); Serial.println(acu_data.max_cell_temp);
    Serial.print("Min Cell Temp      : "); Serial.println(acu_data.min_cell_temp);
    Serial.print("Max Board Temp     : "); Serial.println(acu_data.max_board_temp);
    Serial.println();


    /* ----- Charge Current Information ----- */
    Serial.print("Charger Current    : "); Serial.println(charger_data.output_current_low);
    Serial.print("Calc Charge Current: "); Serial.println(MainChargeSystemInstance::instance().get_charge_current());
    Serial.println();


    /* ----- Charge Information ----- */
    Serial.print("CP PWM             : "); Serial.print(level2_data.control_pwm); Serial.print(" ("); Serial.print(level2_data.control_pwm_duty_cycle); Serial.println("%)");
    Serial.print("CP Voltage Sense   : "); Serial.println(adc.readControlPilot());
    Serial.print("PP Voltage Sense   : "); Serial.println(adc.readProximityPilot());
    Serial.println();


    /* ----- SHDN Information ----- */
    Serial.print("SHDN_A : "); Serial.println(adc.isShutdownAHigh() ? "HIGH" : "LOW");
    Serial.print("SHDN_B : "); Serial.println(adc.isShutdownBHigh() ? "HIGH" : "LOW");
    Serial.print("SHDN_C : "); Serial.println(adc.isShutdownCHigh() ? "HIGH" : "LOW");
    Serial.print("SHDN_D : "); Serial.println(adc.isShutdownDHigh() ? "HIGH" : "LOW");
    Serial.print("SHDN_E : "); Serial.println(adc.isShutdownEHigh() ? "HIGH" : "LOW");
    Serial.print("SHDN_F : "); Serial.println(adc.isShutdownFHigh() ? "HIGH" : "LOW");
    Serial.print("SHDN_G : "); Serial.println(adc.isShutdownGHigh() ? "HIGH" : "LOW");
    Serial.println();


    /* ----- Rotary Encoder ----- */
    Serial.print("Rotary Encoder     : "); Serial.println(RotaryEncoderInterfaceInstance::instance().getValue(), 2);
    Serial.println();


    /* ----- ACU Detailed Cell Voltages ----- */
    Serial.println("Cell Voltages:");
    for (int c = 0; c < default_acu_params::NUM_CELLS; c++)
    {
        Serial.print("C"); Serial.print(c); Serial.print(": ");
        if (acu_data.cell_voltages[c])
        {
            Serial.print(*acu_data.cell_voltages[c], 3);
        }
        else
        {
            Serial.print("--.-");
        }
        Serial.print((c % 12 == 11) ? "\n" : "\t");
    }
    Serial.println();


    /* ----- ACU Detailed Cell Temps ----- */
    Serial.println("Cell Temps:");
    for (int c = 0; c < default_acu_params::NUM_CELL_TEMPS; c++)
    {
        Serial.print("CT"); Serial.print(c); Serial.print(": ");
        if (acu_data.cell_temps[c])
        {
            Serial.print(*acu_data.cell_temps[c], 2);
        }
        else
        {
            Serial.print("--.-");
        }
        Serial.print((c % 12 == 11) ? "\n" : "\t");
    }
    Serial.println();


    /* ----- ACU Detailed Board Temps ----- */
    Serial.println("Board Temps:");
    for (int c = 0; c < default_acu_params::NUM_BOARD_TEMPS; c++)
    {
        Serial.print("BT"); Serial.print(c); Serial.print(": ");
        if (acu_data.board_temps[c])
        {
            Serial.print(*acu_data.board_temps[c], 2);
        }
        else
        {
            Serial.print("--.-");
        }
        Serial.print("\t");
    }
    Serial.println();
    Serial.println();

    return HT_TASK::TaskResponse::YIELD;
}
