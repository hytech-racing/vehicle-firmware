#ifndef DASHBOARD_INTERFACE_H
#define DASHBOARD_INTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include <Wire.h>
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include "FlexCAN_T4.h"

/* Local Interface Includes */
#include "SystemTimeInterface.h"

/* Local System Includes */
#include "MCP23017Interface.h"


struct DashboardGPIOs_s
{
    uint8_t BRIGHTNESS_CONTROL_PIN;
    uint8_t PRESET_BUTTON;
    uint8_t MC_CYCLE_BUTTON;
    uint8_t START_BUTTON;
    uint8_t DATA_BUTTON;
    uint8_t BUTTON_2;
};

class DashboardInterface
{
public:

    DashboardInterface(DashboardGPIOs_s gpio,
                    uint8_t io_expander_addr,
                    TwoWire &i2c_bus
    ) : _dashboard_gpios(gpio),
        _i2c_bus(i2c_bus),
        _io_expander(_i2c_bus,
                    IOExpanderParams_s {
                        .i2c_address = io_expander_addr,
                        .port_a = { .directions = 0b00000000, .pullups = 0xFF, .inverted = 0b00000000 },
                        .port_b = { .directions = 0b01111111, .pullups = 0b00000000, .inverted = 0xFF },
                    }
        )
    {};

    /**
     * @brief Initializes GPIO pins and IO expander.
    */
    void init();

    /**
     * @brief Syncs stored outputs with last read outputs.
    */
    void syncDashboardStoredState();

    void receiveACUOKCANMsg(const CAN_message_t &can_msg);

    void setDialState(ControllerMode_e mode);

    void readIOExpander();

    DashInputState_s get_dashboard_outputs();

    DashInputState_s get_dashboard_stored_state();

    bool isIMDOk() const { return _imd_ok; };

    bool isBMSOk() const { return _bms_ok; };

private:

    bool _bms_ok = true;
    bool _imd_ok = true;
    DashboardGPIOs_s _dashboard_gpios;
    DashInputState_s _dashboard_outputs; // curr state, what the buttons are doing right now
    DashInputState_s _dashboard_stored_state; // previous state, what the buttons were doing last tick
    TwoWire &_i2c_bus;
    MCP23017Interface _io_expander; // must be declared after _i2c_bus (init order follows declaration order)
    unsigned long _dash_created_millis;

};

using DashboardInterfaceInstance = etl::singleton<DashboardInterface>;

#endif /* DASHBOARD_INTERFACE_H */