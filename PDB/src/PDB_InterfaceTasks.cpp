#include "PDB_InterfaceTasks.hpp"


void initializeAllPins()
{
    // Outputs: write LOW *before* making the pin an output, so nothing glitches on at boot
    const uint32_t OUTPUTS_START_LOW[] = {
        PDBPins::EN_CAMERAS_MCU, PDBPins::EN_ORIN_MCU, PDBPins::EN_INV_COOL_MCU,     // Load switches
        PDBPins::EN_MOTOR_COOL_MCU, PDBPins::EN_LIDAR_MCU, PDBPins::EN_DTI_MCU,
        PDBPins::LIDAR_EN_MCU, PDBPins::EN_12V_DTI_MCU,                              // Bucks
        PDBPins::DSMS_EN_MCU, PDBPins::PDB_FAN_EN_MCU,                               // Other enables (no driver yet)
        PDBPins::EEPROM_WC_N_MCU,                                                    // EEPROM write-control, active-low
    };
    for (uint32_t pin : OUTPUTS_START_LOW)
    {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }

    // Inputs: open-drain status lines, pulled up on the board
    const uint32_t INPUTS[] = {
        PDBPins::FLT_CAMERAS_MCU, PDBPins::FLT_ORIN_MCU, PDBPins::FLT_INV_COOL_MCU,  // Load switch faults
        PDBPins::FLT_MOTOR_COOL_MCU, PDBPins::FLT_LIDAR_MCU, PDBPins::FLT_DTI_MCU,
        PDBPins::PG_18V_ORIN_MCU, PDBPins::PG_24V_MAIN_MCU, PDBPins::PG_12V_DTI_MCU, // Power good
        PDBPins::PG_12V_MAIN_MCU, PDBPins::PG_5V_MAIN_MCU, PDBPins::PG_3V3_MAIN_MCU,
        PDBPins::TEMP_48_ALERT, PDBPins::TEMP_49_ALERT, PDBPins::TEMP_4A_ALERT,      // Temp sensor alerts
        PDBPins::TEMP_4B_ALERT, PDBPins::TEMP_4C_ALERT, PDBPins::TEMP_4D_ALERT,
        PDBPins::TEMP_4E_ALERT, PDBPins::TEMP_4F_ALERT,
    };
    for (uint32_t pin : INPUTS)
    {
        pinMode(pin, INPUT);
    }
}

void initializeAllInterfaces()
{
    /**
     * @brief I2C1: temp sensors (ADT75 x8) and hotswap (LM5066)
    */
    const STM32I2CConfig_s I2C_CONFIG = {
        .instance            = I2C1,
        .scl_pin             = PDBPins::I2C1_SCL,
        .sda_pin             = PDBPins::I2C1_SDA,
        .kernel_clock_source = RCC_I2C123CLKSOURCE_D2PCLK1,
        .timing              = 0x307075B1,       // From CubeMX for the D2PCLK1 kernel clock
        .analog_filter       = true,
        .digital_filter      = 0,
    };

    /**
     * @brief FDCAN1, classic CAN at 500 kbit/s: 80 MHz (PLL1Q) / 10 / (1 + 13 + 2)
    */
    const STM32CANConfig_s CAN_CONFIG = {
        .instance                = FDCAN1,
        .rx_pin                  = PDBPins::FDCAN1_RX,
        .tx_pin                  = PDBPins::FDCAN1_TX,
        .kernel_clock_source     = RCC_FDCANCLKSOURCE_PLL,
        .nominal_prescaler       = 10,
        .nominal_sync_jump_width = 2,
        .nominal_time_seg1       = 13,
        .nominal_time_seg2       = 2,
        .auto_retransmission     = false,
        .rx_fifo_elements        = 8,
        .tx_fifo_elements        = 8,
    };

    // Init I2C1
    STM32I2CInterfaceInstance::create();
    STM32I2CInterfaceInstance::instance().init(I2C_CONFIG);

    // Init CAN interface
    VCRInterfaceInstance::create();
    CANInterfacesInstance::create(VCRInterfaceInstance::instance());

    /**
     * @brief Buck converters. Power tree:
     *
     *   24V ─┬─► LM61495   MAIN_12V ──► MAXM17536 MAIN_5V ──► TPS62085 MAIN_3V3 ──► MCU
     *        ├─► LM61495   ORIN_18V   (always on)
     *        ├─► LM61495   DTI_12V    (EN: MCU)
     *        └─► LTC3115   LIDAR_24V  (EN: MCU)
     *
     * @note The MCU's own supply chain (MAIN_12V -> MAIN_5V -> MAIN_3V3) has no EN pin, so firmware can never
     *       turn off its own power. Only DTI_12V and LIDAR_24V can be disabled / restarted, and the MCU stays
     *       powered while they are off.
     * @note restart_wait_duration_ms only matters for DTI_12V and LIDAR_24V. Their parts have no output discharge,
     *       so the rail decays through the load: measure the decay on a scope with the load connected and size it from that.
    */
    const BuckConverterDefinition_s LIDAR_24V = {
        .id     = BUCK_LIDAR_24V,
        .part   = BuckConverterPart_e::LTC3115,
        .en     = PDBPins::LIDAR_EN_MCU,
        .pg     = buck_converter_interface_default_params::NOT_CONNECTED,     // LTC3115 has no PG output
        .timing_params = buck_converter_default_timings::LTC3115,
    };

    const BuckConverterDefinition_s ORIN_18V = {
        .id     = BUCK_ORIN_18V,
        .part   = BuckConverterPart_e::LM61495,
        .en     = buck_converter_interface_default_params::NOT_CONNECTED,     // always on
        .pg     = PDBPins::PG_18V_ORIN_MCU,
        .timing_params = buck_converter_default_timings::LM61495,
    };

    const BuckConverterDefinition_s DTI_12V = {
        .id     = BUCK_DTI_12V,
        .part   = BuckConverterPart_e::LM61495,
        .en     = PDBPins::EN_12V_DTI_MCU,
        .pg     = PDBPins::PG_12V_DTI_MCU,
        .timing_params = buck_converter_default_timings::LM61495,
    };

    // MAIN_12V: LM61495 (24V -> 12V). MAIN_5V: MAXM17536 (12V -> 5V).
    const BuckConverterDefinition_s MAIN_12V = {
        .id     = BUCK_MAIN_12V,
        .part   = BuckConverterPart_e::LM61495,
        .en     = buck_converter_interface_default_params::NOT_CONNECTED,     // always on
        .pg     = PDBPins::PG_12V_MAIN_MCU,
        .timing_params = buck_converter_default_timings::LM61495,
    };

    const BuckConverterDefinition_s MAIN_5V = {
        .id     = BUCK_MAIN_5V,
        .part   = BuckConverterPart_e::MAXM17536,
        .en     = buck_converter_interface_default_params::NOT_CONNECTED,     // always on
        .pg     = PDBPins::PG_5V_MAIN_MCU,
        .timing_params = {
            .startup_timeout_duration_ms = maxm17536_params::computeStartupTimeoutMs(),
            .pg_debounce_duration_ms     = maxm17536_params::PG_DEBOUNCE_MS,
            .restart_wait_duration_ms     = maxm17536_params::RESTART_OFF_MS,
        },
    };

    const BuckConverterDefinition_s MAIN_3V3 = {
        .id     = BUCK_MAIN_3V3,
        .part   = BuckConverterPart_e::TPS62085,
        .en     = buck_converter_interface_default_params::NOT_CONNECTED,     // always on
        .pg     = PDBPins::PG_3V3_MAIN_MCU,
        .timing_params = buck_converter_default_timings::TPS62085,
    };

    const std::array<BuckConverterDefinition_s, buck_converter_interface_default_params::NUM_BUCKS> ALL_BUCKS =
    {
        LIDAR_24V, ORIN_18V, DTI_12V, MAIN_12V, MAIN_5V, MAIN_3V3
    };


    BuckConverterInterfaceInstance::create(ALL_BUCKS);
    BuckConverterInterfaceInstance::instance().init();

    I2C_HandleTypeDef *hi2c = STM32I2CInterfaceInstance::instance().getHandle();

    TempSensorInterfaceInstance::create(hi2c);
    TempSensorInterfaceInstance::instance().initAllSensors();

    /**
     * @brief LM5066 hotswap on I2C1. SMBA = PDBPins::HOTSWAP_SMBA_MCU (PB5), PGD = PDBPins::PG_24V_MAIN_MCU (PB11).
     * @note Hardware fields (slew rate, limits, thresholds) are still TODO in HotSwapInterface.hpp; left at 0.
    */
    const HotSwapConfig_s HOTSWAP_CONFIG = {
        .SMBA_PIN = PDBPins::HOTSWAP_SMBA_MCU,
        .PGD_PIN  = PDBPins::PG_24V_MAIN_MCU,
    };
    HotSwapInterfaceInstance::create(HOTSWAP_CONFIG);
    HotSwapInterfaceInstance::instance().initHotswap(hi2c);

    /**
     * @brief TPS2663 load switches. Pins from Pins.h (FLT / EN / IMON), resistors from the schematic
     *        (Aazam's table): I_OL = 18 / R_ILIM[kOhm], IMON scale from R_IMON.
     * @note startup_ignore_fault_ms covers turn-on delay + dVdT ramp (~56 ms at 100 nF, 24 V; see sampleFault()).
    */
    const std::array<LoadSwitchParams_s, NUM_LOAD_SWITCHES> LOAD_SWITCH_PARAMS = {{
        /* LDSW_CAMERAS    */ {.id = LDSW_CAMERAS, .fault_pin = PDBPins::FLT_CAMERAS_MCU,  .enable_pin = PDBPins::EN_CAMERAS_MCU,  .imon_pin = PDBPins::IMON_CAMERAS_MCU, .imon_resistor_ohms = 100000, .ilim_resistor_ohms = 20000, .startup_ignore_fault_ms = loadswitch_default_params::STARTUP_IGNORE_FAULT_MS},
        /* LDSW_ORIN       */ {.id = LDSW_ORIN, .fault_pin = PDBPins::FLT_ORIN_MCU,  .enable_pin = PDBPins::EN_ORIN_MCU,  .imon_pin = PDBPins::IMON_ORIN_MCU, .imon_resistor_ohms = 31600,  .ilim_resistor_ohms = 4870,  .startup_ignore_fault_ms = loadswitch_default_params::STARTUP_IGNORE_FAULT_MS},
        /* LDSW_INV_COOL   */ {.id = LDSW_INV_COOL, .fault_pin = PDBPins::FLT_INV_COOL_MCU,  .enable_pin = PDBPins::EN_INV_COOL_MCU,  .imon_pin = PDBPins::IMON_INV_COOL_MCU, .imon_resistor_ohms = 22100,  .ilim_resistor_ohms = 5100,  .startup_ignore_fault_ms = loadswitch_default_params::STARTUP_IGNORE_FAULT_MS},
        /* LDSW_MOTOR_COOL */ {.id = LDSW_MOTOR_COOL, .fault_pin = PDBPins::FLT_MOTOR_COOL_MCU, .enable_pin = PDBPins::EN_MOTOR_COOL_MCU, .imon_pin = PDBPins::IMON_MOTOR_COOL_MCU, .imon_resistor_ohms = 22100,  .ilim_resistor_ohms = 5100,  .startup_ignore_fault_ms = loadswitch_default_params::STARTUP_IGNORE_FAULT_MS},
        /* LDSW_LIDAR      */ {.id = LDSW_LIDAR, .fault_pin = PDBPins::FLT_LIDAR_MCU, .enable_pin = PDBPins::EN_LIDAR_MCU, .imon_pin = PDBPins::IMON_LIDAR_MCU, .imon_resistor_ohms = 84500,  .ilim_resistor_ohms = 14700, .startup_ignore_fault_ms = loadswitch_default_params::STARTUP_IGNORE_FAULT_MS},
        /* LDSW_DTI        */ {.id = LDSW_DTI, .fault_pin = PDBPins::FLT_DTI_MCU, .enable_pin = PDBPins::EN_DTI_MCU, .imon_pin = PDBPins::IMON_DTI_MCU, .imon_resistor_ohms = 24900,  .ilim_resistor_ohms = 3900,  .startup_ignore_fault_ms = loadswitch_default_params::STARTUP_IGNORE_FAULT_MS},
    }};
    LoadSwitchInterfaceInstance::create(LOAD_SWITCH_PARAMS);
    LoadSwitchInterfaceInstance::instance().init();     // SHDN driven low: every switch starts off

    // CAN last: frames can arrive as soon as it starts, and the callback routes them to CANInterfaces
    STM32CANInterfaceInstance::create();
    STM32CANInterfaceInstance::instance().setReceiveCallback(PDBCAN::onReceive);
    STM32CANInterfaceInstance::instance().init(CAN_CONFIG);
}

/**
 * @return Bitmask of active faults: bit 0 = overtemp, bit 1 = buck fault, bit 2 = hotswap fault
*/
uint8_t checkFaults() {
    uint8_t faults = 0;
    faults |= TempSensorInterfaceInstance::instance().anyOvertemp()  ? (1U << 0) : 0;
    faults |= BuckConverterInterfaceInstance::instance().anyFault()  ? (1U << 1) : 0;

    return faults;
}

HT_TASK::TaskResponse updateBuckConvertersTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    BuckConverterInterfaceInstance::instance().updateAllBuckConverters();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse updateHotSwapTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    HotSwapInterfaceInstance::instance().update();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse readTempSensorsTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    TempSensorInterfaceInstance::instance().readAllTempSensors();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse sampleLoadSwitchesTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    LoadSwitchInterfaceInstance::instance().sampleAll();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse debugPrintsTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    static const String BUCK_NAMES[NUM_BUCK_IDS] = {"LIDAR_24V", "ORIN_18V", "DTI_12V", "MAIN_12V", "MAIN_5V", "MAIN_3V3"};
    static const String BUCK_STATES[] = {"DISABLED", "STARTING", "ACTIVE", "FAULT", "RESTARTING"};
    static const String TEMP_SENSOR_NAMES[temp_sensor_default_params::NUM_SENSORS] = {"BUCK_LIDAR", "BUCK_ORIN", "BUCK_12V", "BUCK_DTI", "BUCK_5V", "BUCK_3V3", "MCU", "HOTSWAP"};
    static const String LOAD_SWITCH_NAMES[NUM_LOAD_SWITCHES] = {"CAMERAS", "ORIN", "INV_COOL", "MOTOR_COOL", "LIDAR", "DTI"};

    Serial.print("\n\nPDB Debug, time: ");
    Serial.println(millis());

    /* Buck Converters */
    Serial.println("\nBuck Converters:");
    Serial.println("\t\tState\t\tEN\tPG\tFaults\tRecoveries");
    for (uint8_t i = 0; i < NUM_BUCK_IDS; i++)
    {
        const BuckConverterStatus_s *status = BuckConverterInterfaceInstance::instance().getStatus(i);
        Serial.print(BUCK_NAMES[i]); Serial.print(":\t");
        Serial.print(BUCK_STATES[static_cast<uint8_t>(status->current_state)]); Serial.print("\t");
        Serial.print(status->is_enabled); Serial.print("\t");
        if (status->has_pg_pin) { Serial.print(status->is_pg_high); } else { Serial.print("-"); }   // "-" = no PG pin (LTC3115)
        Serial.print("\t");
        Serial.print(status->fault_count); Serial.print("\t");
        Serial.println(status->recovery_count);
    }

    /* Temp Sensors */
    Serial.println("\nTemp Sensors:");
    Serial.println("\t\tTemp [C]\tOnline\tOvertemp");
    for (uint8_t i = 0; i < temp_sensor_default_params::NUM_SENSORS; i++)
    {
        Serial.print(TEMP_SENSOR_NAMES[i]); Serial.print(":\t");
        Serial.print(TempSensorInterfaceInstance::instance().getTemp(i), 1); Serial.print("\t\t");
        Serial.print(TempSensorInterfaceInstance::instance().isOnline(i)); Serial.print("\t");
        Serial.println(TempSensorInterfaceInstance::instance().isOvertemp(i));
    }

    /* Hotswap */
    Serial.println("\nHotswap:");
    Serial.print("online: ");            Serial.println(HotSwapInterfaceInstance::instance().getStatus().is_online);
    Serial.print("power_good: ");        Serial.println(HotSwapInterfaceInstance::instance().isPowerGood());
    Serial.print("Vin [V]: ");           Serial.println(HotSwapInterfaceInstance::instance().getData().Vin, 2);
    Serial.print("Vout [V]: ");          Serial.println(HotSwapInterfaceInstance::instance().getData().Vout, 2);
    Serial.print("Iin [A]: ");           Serial.println(HotSwapInterfaceInstance::instance().getData().Iin, 2);
    Serial.print("Pin [W]: ");           Serial.println(HotSwapInterfaceInstance::instance().getData().Pin, 1);
    Serial.print("temp [C]: ");          Serial.println(HotSwapInterfaceInstance::instance().getData().temp, 1);
    Serial.print("diagnostic_word: 0x"); Serial.println(HotSwapInterfaceInstance::instance().getDiagnosticWord(), HEX);

    /* Load Switches */
    Serial.println("\nLoad Switches:");
    Serial.println("\t\tEN\tFLT\tI [mA]\tLimit [mA]");
    for (uint8_t i = 0; i < NUM_LOAD_SWITCHES; i++)
    {
        const LoadSwitchInterface &load_switches = LoadSwitchInterfaceInstance::instance();
        Serial.print(LOAD_SWITCH_NAMES[i]); Serial.print(":\t");
        Serial.print(load_switches.isLoadSwitchEnabled(i)); Serial.print("\t");
        Serial.print(load_switches.isLoadSwitchFaulted(i)); Serial.print("\t");
        Serial.print(load_switches.getIMONCurrent(i)); Serial.print("\t");
        Serial.println(load_switches.current_limit_mA(i));
    }

    return HT_TASK::TaskResponse::YIELD;
}

void enableAllRails()
{
    /// WARNING: This is only for testing, do not normally just enable all bucks
    for (uint8_t i = 0; i < NUM_BUCK_IDS; i++)
    {
        BuckConverterInterfaceInstance::instance().enable(i);
    }

    LoadSwitchInterfaceInstance::instance().enableAll();
}