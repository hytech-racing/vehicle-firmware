#ifndef VCFINTERFACE_H
#define VCFINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "CANInterface.h"


struct VCFCANInterfaceData_s
{
    StampedPedalsSystemData_s stamped_pedals;
    DashInputState_s dash_input_state;
};


class VCFInterface
{
public:

    VCFInterface() = delete;

    VCFInterface(unsigned long init_millis, unsigned long max_heartbeat_interval_ms) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.stamped_pedals.last_recv_millis = 0;
        _curr_data.stamped_pedals.heartbeat_ok = false; // start out false
    };

    void receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis);

    void receive_dashboard_message(const CAN_message_t& msg, unsigned long curr_millis);

    bool is_brake_pressed() { return _curr_data.stamped_pedals.pedals_data.brake_is_pressed; }

    bool is_mech_brake_pressed() { return _curr_data.stamped_pedals.pedals_data.mech_brake_is_active; }

    bool is_drivetrain_reset_pressed() { return _curr_data.dash_input_state.mc_reset_btn_is_pressed; }

    bool is_recalibrate_pedals_button_pressed() { return _curr_data.dash_input_state.preset_btn_is_pressed; }

    int get_control_mode() { return _control_mode; }

    VCFCANInterfaceData_s get_curr_data() {return _curr_data;}

private:

    VCFCANInterfaceData_s _curr_data;
    int _control_mode = 0;
    unsigned long _max_heartbeat_interval_ms;
    bool _first_received_message_heartbeat_init = false;

};

using VCFInterfaceInstance = etl::singleton<VCFInterface>;

#endif // __VCFINTERFACE_H__


