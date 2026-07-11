#ifndef DASHBOARD_INTERFACE_H
#define DASHBOARD_INTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include <MCP23017.h>
#include <Wire.h>
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include "FlexCAN_T4.h"

/* Local Interface Includes */
#include "SystemTimeInterface.h"

/* Local System Includes */
#include "IOExpanderUtilities.h"


// Struct representing dashboard gpios
struct DashboardGPIOs_s
{
    // GPIO
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
    DashboardInterface(DashboardGPIOs_s gpios,
                    uint8_t io_expander_addr,
                    TwoWire &i2c_bus
    ) : _dashboard_gpios(gpios),
        _io_expander(MCP23017(io_expander_addr, i2c_bus)),
        _i2c_bus(i2c_bus)
    {};

    /**
     * @brief Initializes GPIO pins and IO expander.
     */
    void init();

    /**
     * @brief Syncs stored outputs with last read outputs.
     */
    void sync_dashboard_stored_state();

    bool bms_ok = true;
    bool imd_ok = true;
    void receive_ACU_OK(const CAN_message_t &can_msg);

    void set_dial_state(ControllerMode_e mode);

    void read_ioexpander();

    DashInputState_s get_dashboard_outputs();

    DashInputState_s get_dashboard_stored_state();

private:

    DashboardGPIOs_s _dashboard_gpios;
    DashInputState_s _dashboard_outputs; // curr state, what the buttons are doing right now
    DashInputState_s _dashboard_stored_state; // previous state, what the buttons were doing last tick
    MCP23017 _io_expander;
    TwoWire &_i2c_bus;
    unsigned long _dash_created_millis;

    void _init_ioexpander();

};

using DashboardInterfaceInstance = etl::singleton<DashboardInterface>;

#endif /* DASHBOARD_INTERFACE_H */