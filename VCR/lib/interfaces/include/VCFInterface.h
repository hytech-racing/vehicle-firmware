#ifndef VCFINTERFACE_H
#define VCFINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include "hytech.h"
#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "SystemTimeInterface.h"


struct VCFCANInterfaceData_s
{
    StampedPedalsSystemData_s stamped_pedals;
    StampedSteeringSystemData_s stamped_steering;
    DashInputState_s dash_input_state;
    FrontLoadCellData_s front_loadcell_data;
    FrontSusPotData_s front_suspot_data;
};

class VCFInterface
{
public:

    VCFInterface() = delete;

    VCFInterface(unsigned long init_millis,
                unsigned long max_heartbeat_interval_ms
    ) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.stamped_pedals.last_recv_millis = 0;
        _curr_data.stamped_steering.last_recv_millis = 0;
        _curr_data.stamped_pedals.heartbeat_ok = false;
        _curr_data.stamped_steering.heartbeat_ok = false;
        _is_pedals_heartbeat_init = false;
        _is_steering_heartbeat_init = false;
    };

    /// NOTE: Start button is RTD
    bool isStartButtonPressed() { return _curr_data.dash_input_state.start_btn_is_pressed; }

    bool isBrakePressed() {return _curr_data.stamped_pedals.pedals_data.brake_is_pressed; }

    bool isRecalibratePedalsButtonPressed() {return _curr_data.dash_input_state.preset_btn_is_pressed; }

    bool isRecalibrateSteeringButtonPressed() {return _curr_data.dash_input_state.data_btn_is_pressed; }

    bool isPedalsHeartbeatNotOk() {return !_curr_data.stamped_pedals.heartbeat_ok; }

    bool isSteeringHeartbeatNotOk() {return !_curr_data.stamped_steering.heartbeat_ok; }

    void resetPedalsHeartbeat();

    void resetSteeringHeartbeat();

    void receivePedalsCANMessage(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveSteeringCANMessage(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveDashboardCANMessage(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveFrontSuspensionCANMessage(const CAN_message_t &msg, unsigned long curr_millis);

    VCFCANInterfaceData_s getLatestData() const;

    /**
     * @brief 4 methods below set various fields insid the DASHBOARD_BUZZER_CONTROL_t messages
     * @note TODO: Rename this message
    */
    void sendBuzzerStartCANMessage();
    void send_recalibrate_pedals_message();
    void enqueueRecalibrateSteeringCANMessage();
    void enqueue_torque_mode_LED_message(TorqueLimit_e torque_mode);

    void enqueueVehicleStateCANMessage(VehicleState_e vehicle_state, DrivetrainState_e drivetrain_state, bool db_is_in_ctrl);

private:

    mutable VCFCANInterfaceData_s _curr_data;
    unsigned long _max_heartbeat_interval_ms;
    mutable bool _is_pedals_heartbeat_init;
    mutable bool _is_steering_heartbeat_init;

};

using VCFInterfaceInstance = etl::singleton<VCFInterface>;

#endif // __VCFINTERFACE_H__