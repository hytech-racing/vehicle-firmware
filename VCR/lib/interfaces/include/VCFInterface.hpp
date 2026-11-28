#ifndef VCF_INTERFACE_HPP
#define VCF_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "ht_can.h"
#include "shared_types.hpp"

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

    VCFInterface(unsigned long init_millis, unsigned long max_heartbeat_interval_ms)
        : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.stamped_pedals.last_recv_millis = 0;
        _curr_data.stamped_steering.last_recv_millis = 0;
        _curr_data.stamped_pedals.heartbeat_ok = false;
        _curr_data.stamped_steering.heartbeat_ok = false;
    };

    bool isStartButtonPressed() { return _curr_data.dash_input_state.start_btn_is_pressed; }

    bool isBrakePressed() { return _curr_data.stamped_pedals.pedals_data.brake_is_pressed; }

    bool isDrivetrainResetPressed() { return _curr_data.dash_input_state.mc_reset_btn_is_pressed; }

    bool isRecalibratePedalsButtonPressed() { return _curr_data.dash_input_state.preset_btn_is_pressed; }

    bool isRecalibrateSteeringButtonPressed() { return _curr_data.dash_input_state.data_btn_is_pressed; }

    bool isPedalsHeartbeatNotOk() { return !_curr_data.stamped_pedals.heartbeat_ok; }

    bool isSteeringHeartbeatNotOk() { return !_curr_data.stamped_steering.heartbeat_ok; }

    void resetPedalsHeartbeat();

    void resetSteeringHeartbeat();

    void receivePedalsMsg(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveSteeringMsg(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveDashboardMsg(const CAN_message_t& msg, unsigned long curr_millis);

    void receiveFrontSuspensionMsg(const CAN_message_t& msg, unsigned long curr_millis);

    VCFCANInterfaceData_s getLatestData() const;

    void sendBuzzerStartMsg();

    void sendRecalibratePedalsMsg();

    void sendRecalibrateSteeringMsg();

    void enqueueTorqueModeLEDMsg(TorqueLimit_e torque_mode);

    void enqueueVehicleStateMsg(VehicleState_e vehicle_state, DrivetrainState_e drivetrain_state, bool db_is_in_ctrl);

  private:
    mutable VCFCANInterfaceData_s _curr_data;
    unsigned long _max_heartbeat_interval_ms;
    mutable bool _first_received_message_heartbeat_init = false;
};

using VCFInterfaceInstance = etl::singleton<VCFInterface>;

#endif // __VCFINTERFACE_H__
