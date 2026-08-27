#ifndef INVERTERINTERFACE_H
#define INVERTERINTERFACE_H

/* External Includes */
#include <algorithm>
#include "shared_types.h"
#include <FlexCAN_T4.h>
#include "CANInterface.h"
#include "hytech.h"

/* Local System Includes */
#include "DrivetrainSystem.h"

/**
 * GENERAL INFORMATION
 *
 *   - All messages are BIG ENDIAN
 *   - Standard ID (11 bit): Packet ID + Node ID
 *     Layout: [Packet ID: bits 10:5 (6 bits)] [Node ID: bits 4:0 (5 bits)]
 *   - For all limit booleans: 1 is active, 0 is inactive
 *
*/



namespace dti_default_params
{
    constexpr volt MINIMUM_HV_VOLTAGE = 200.0f;     // TODO: confirm real threshold against pack/HV config
    constexpr volt MAXIMUM_HV_VOLTAGE = 620.f;      // TODO: confirm real threshold against pack/HV config
    constexpr float LAMBDA_PM_WB = 0.0507f;         // Wb, DTI F-MOT
    constexpr float LD_HENRIES = 243e-6f;           // H, DTI F-MOT
    constexpr float LQ_HENRIES = 0.000371;          // H, DTI F-MOT
    constexpr uint8_t NUM_POLE_PAIRS = 4;           // TODO: confirm against motor datasheet
}


/* ---------- Packet + Node IDs (Status and Command) ---------- */
namespace dti_status_packet_ids
{
    // Status (inverter -> VCR)
    constexpr uint8_t STATUS_GENERAL_CONTROL        = 0x1F;   // Control mode, Target Iq, Motor position, isMotorStill
    constexpr uint8_t STATUS_GENERAL_ELEC           = 0x20;   // ERPM, Duty, Input Voltage
    constexpr uint8_t STATUS_AC_DC_CURRENT          = 0x21;   // AC Current, DC Current
    constexpr uint8_t STATUS_TEMP_AND_FAULT         = 0x22;   // Controller Temp, Motor Temp, Fault code
    constexpr uint8_t STATUS_FOC_CURRENTS           = 0x23;   // Id, Iq
    constexpr uint8_t STATUS_GENERAL_IO             = 0x24;   // Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version
    constexpr uint8_t STATUS_AC_CURRENT_LIMITS      = 0x25;   // Configured max AC current, available max AC current, configured min AC current, available min AC current
    constexpr uint8_t STATUS_DC_CURRENT_LIMITS      = 0x26;   // Configured max DC current, available max DC current, configured min DC current, available min DC current
}

namespace dti_command_packet_ids
{
    // Commands (VCR -> inverter)
    constexpr uint8_t SET_AC_CURRENT                    = 0x01;
    constexpr uint8_t SET_BRAKE_CURRENT                 = 0x02;
    constexpr uint8_t SET_ERPM                          = 0x03;   // unused in current-control design, kept for completeness
    constexpr uint8_t SET_MOTOR_POSITION                = 0x04;
    constexpr uint8_t SET_RELATIVE_AC_CURRENT           = 0x05;
    constexpr uint8_t SET_RELATIVE_AC_BRAKE_CURRENT     = 0x06;
    constexpr uint8_t SET_DIGITAL_OUTPUT                = 0x07;
    constexpr uint8_t SET_MAX_AC_CURRENT                = 0x08;
    constexpr uint8_t SET_MAX_AC_BRAKE_CURRENT          = 0x09;
    constexpr uint8_t SET_MAX_DC_CURRENT                = 0x0A;
    constexpr uint8_t SET_MAX_DC_BRAKE_CURRENT          = 0x0B;
    constexpr uint8_t SET_DRIVE_ENABLE                  = 0x0C;
}

namespace dti_node_ids
{
    constexpr uint8_t FL = 1;
    constexpr uint8_t FR = 2;
    constexpr uint8_t RL = 3;
    constexpr uint8_t RR = 4;
}

/**
 * @brief Fixed protocol scale factors from the DTI CAN manual. Identical for every inverter.
 */
struct DTIScales_s
{
    const uint8_t iq_scale;                               // 10
    const uint8_t motor_position_scale;                   // 10
    const uint8_t duty_cycle_scale;                       // 10
    const uint8_t active_ac_current_scale;                // 10
    const uint8_t active_dc_current_scale;                // 10
    const uint8_t controller_temp_scale;                  // 10
    const uint8_t motor_temp_scale;                       // 10
    const uint8_t foc_iq_scale;                           // 100
    const uint8_t foc_id_scale;                           // 100
    const uint8_t max_ac_current_scale;                   // 10
    const uint8_t available_max_ac_current_scale;         // 10
    const uint8_t min_ac_current_scale;                   // 10
    const uint8_t available_min_ac_current_scale;         // 10
    const uint8_t max_dc_current_scale;                   // 10
    const uint8_t available_max_dc_current_scale;         // 10
    const uint8_t min_dc_current_scale;                   // 10
    const uint8_t available_min_dc_current_scale;         // 10
};

struct DTIParams_s
{
    float minimum_hv_voltage;
    float maximum_hv_voltage;
    float lambda_pm_wb;
    float ld_henries;
    float lq_henries;
    uint8_t num_pole_pairs;
    DTIScales_s scales;
};

/* ---------- Enums (declared before structs that reference them) ---------- */

enum class DTIControlMode_e : uint8_t
{
    NONE_0             = 0,   // NOT USED per manual
    MODE_SPEED         = 1,
    MODE_CURRENT       = 2,
    MODE_CURRENT_BRAKE = 3,
    MODE_POSITION      = 4,
    NONE_5             = 5,   // NOT USED
    NONE_6             = 6,   // NOT USED
    MODE_NONE          = 7,
};

enum class DTIFaultCode_e : uint8_t
{
    NO_FAULTS            = 0x00,
    OVERVOLTAGE          = 0x01,
    UNDERVOLTAGE         = 0x02,
    DRV_ERROR            = 0x03,   // transistor/transistor drive error
    ABS_OVERCURRENT      = 0x04,   // AC current higher than set absolute maximum
    CONTROLLER_OVERTEMP  = 0x05,
    MOTOR_OVERTEMP       = 0x06,
    SENSOR_WIRE_FAULT    = 0x07,   // sensor differential signal fault
    SENSOR_GENERAL_FAULT = 0x08,   // error processing sensor signals
    CAN_COMMAND_ERROR    = 0x09,   // received message had a parameter out of boundaries
    ANALOG_INPUT_ERROR   = 0x0A,   // redundant output out of range
};


/* ---------- Status messages (inverter -> VCR) ---------- */

struct StatusGeneralControlMsg_s    // 0x1F
{
    DTIControlMode_e control_mode;
    float target_iq_apk;            // +/- 850 peak amperes (operational range)
    float motor_position_deg;
    bool is_motor_stationary;       // 1: still/stationary, 0: rotating
};

struct StatusGeneralElecMsg_s   // 0x20
{
    int32_t erpm;               // Electrical RPM. ERPM = Motor RPM * number of the motor pole pairs. +/- 120 000 (operational range)
    float duty_cycle_percent;   // Sign = motoring(+) vs regen(-)
    float input_voltage;        // 0 - 1000 V (operational range)
};

struct StatusActiveCurrentMsg_s     // 0x21
{
    float active_ac_current_apk;    // sign = motoring(+) vs regen(-), +/- 850 A_pk
    float active_dc_current_amp;    // sign = motoring(+) vs regen(-), +/- 850 A
};


struct StatusTempAndFaultMsg_s       // 0x22
{
    float controller_temp_c;        // -55 to +200 (operational range)
    float motor_temp_c;             // -55 to +200 (operational range)
    DTIFaultCode_e fault_code;
};

struct StatusFOCCurrentsMsg_s   // 0x23 — measured, not currently consumed downstream
{
    float iq_apk;               // +/- 850 A_pk (operational range)
    float id_apk;               // +/- 850 A_pk (operational range)
};

struct StatusGeneralIOMsg_s                  // 0x24
{
    float throttle_signal_percent;
    float brake_signal_percent;
    bool is_digital_input_1_active;
    bool is_digital_input_2_active;
    bool is_digital_input_3_active;
    bool is_digital_input_4_active;
    bool is_digital_output_1_active;
    bool is_digital_output_2_active;
    bool is_digital_output_3_active;
    bool is_digital_output_4_active;
    bool is_drive_enabled;
    bool is_capacitor_temp_limit_active;    // Inverter can limit the output power to not to overheat the internal capacitors. (only valid HW version 3.6 or newer)
    bool is_dc_current_limit_active;
    bool is_drive_enable_limit_active;      // NOTE: NOT an indicator of drive enable state
    bool is_igbt_accel_temp_limit_active;
    bool is_igbt_temp_limit_active;
    bool is_input_voltage_limit_active;
    bool is_motor_accel_temp_limit_active;
    bool is_motor_temp_limit_active;
    bool is_rpm_min_limit_active;
    bool is_rpm_max_limit_active;
    bool is_power_limit_active;
    float can_map_version;                  // e.g. 25 -> "2.5"
};

struct StatusACConfigCurrentMsg_s           // 0x25
{
    float max_ac_current_apk;               // 0 to +850 A_pk
    float available_max_ac_current_apk;     // affected by igbt/motor temp limiting etc.
    float min_ac_current_apk;               // 0 to -850 A_pk
    float available_min_ac_current_apk;     // affected by igbt/motor temp limiting etc.
};

struct StatusDCConfigCurrentMsg_s           // 0x26
{
    float max_dc_current_amp;               // 0 to +850 A. Configured w/CAN Tool
    float available_max_dc_current_amp;     // affected by igbt/motor temp limiting etc.
    float min_dc_current_amp;               // 0 to -850 A. Configured w/CAN Tool
    float available_min_dc_current_amp;     // affected by igbt/motor temp limiting etc.
};


/* ---------- Command messages (VCR -> inverter) ---------- */

struct SetACCurrentMsg_s                // 0x01
{
    float target_ac_current_apk;
    // +/- 850 A_pk (operational range). Sets target motor AC current.
    // Receiving this message auto-switches the inverter to current control mode.
};

struct SetBrakeCurrentMsg_s             // 0x02
{
    float target_brake_current_apk;     // 0 to +850 A_pk (operational range), positive-only
};

struct SetERPMMsg_s     // 0x03
{
    float target_erpm;  // +/- 100,000 (operational range)
};

struct SetMotorPositionMsg_s    // 0x04
{
    float target_motor_position_deg;
};

struct SetRelativeACCurrentMsg_s    // 0x05
{
    float target_ac_current_percent;
    // This command sets a relative AC current to the minimum and maximum limits set by configuration.
    // NOTE!!! Achieves the same function as the “Set AC current” command.
    // You do not need to know the motor limit parameters for this command
};

struct SetRelativeACBrakeCurrentMsg_s   // 0x06
{
    float target_ac_brake_current_percent;
};
struct SetDigitalOutputsMsg_s   // 0x07
{
    bool is_digital_output_1_active;
    bool is_digital_output_2_active;
    bool is_digital_output_3_active;
    bool is_digital_output_4_active;
};

struct SetMaxACCurrentMsg_s   // 0x08
{
    float max_ac_current_apk;
};

struct SetMaxACBrakeCurrentMsg_s   // 0x09
{
    float max_ac_brake_current_limit_apk;
};

struct SetMaxDCCurrentMsg_s   // 0x0A
{
    float max_dc_current_limit_amp;
};

struct SetMaxDCBrakeCurrentMsg_s   // 0x0B
{
    float max_dc_brake_current_amp;
};

struct SetDriveEnableMsg_s   // 0x0C
{
    bool is_drive_enabled;
};

struct DTIStatusMessages_s
{
    StatusGeneralControlMsg_s general_control_msg;
    StatusGeneralElecMsg_s general_elec_msg;
    StatusActiveCurrentMsg_s active_current_msg;
    StatusTempAndFaultMsg_s temp_and_fault_msg;
    StatusFOCCurrentsMsg_s foc_current_msg;
    StatusGeneralIOMsg_s general_io_msg;
    StatusACConfigCurrentMsg_s ac_config_current_msg;
    StatusDCConfigCurrentMsg_s dc_config_current_msg;
};
struct DTISetMessages_s
{
    SetACCurrentMsg_s ac_current_msg;
    SetBrakeCurrentMsg_s brake_current_msg;
    SetERPMMsg_s erpm_msg;
    SetMotorPositionMsg_s motor_position_msg;
    SetRelativeACCurrentMsg_s relative_ac_current_msg;
    SetRelativeACBrakeCurrentMsg_s relative_ac_brake_current_msg;
    SetDigitalOutputsMsg_s digital_output_msg;
    SetMaxACCurrentMsg_s max_ac_current_msg;
    SetMaxACBrakeCurrentMsg_s max_ac_brake_current_msg;
    SetMaxDCCurrentMsg_s max_dc_current_msg;
    SetMaxDCBrakeCurrentMsg_s max_dc_brake_current_msg;
    SetDriveEnableMsg_s drive_enable_msg;
};

class InverterInterface
{
public:

    InverterInterface() = delete;

    InverterInterface(uint8_t node_id,
                    DTIScales_s scales
    ) : _node_id(node_id),
        _dti_params {
            .minimum_hv_voltage = dti_default_params::MINIMUM_HV_VOLTAGE,
            .maximum_hv_voltage = dti_default_params::MAXIMUM_HV_VOLTAGE,
            .lambda_pm_wb = dti_default_params::LAMBDA_PM_WB,
            .ld_henries = dti_default_params::LD_HENRIES,
            .lq_henries = dti_default_params::LQ_HENRIES,
            .num_pole_pairs = dti_default_params::NUM_POLE_PAIRS,
            .scales = scales
          }
    {}

    /* ---------- Receiving Callbacks ---------- */
    void receive_GENERAL_CONTROL(const CAN_message_t& can_msg, unsigned long curr_millis);       // 0x1F
    void receive_GENERAL_ELEC(const CAN_message_t& can_msg, unsigned long curr_millis);          // 0x20
    void receive_ACTIVE_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis);        // 0x21
    void receive_TEMP_AND_FAULT(const CAN_message_t& can_msg, unsigned long curr_millis);        // 0x22
    void receive_FOC_CURRENTS(const CAN_message_t& can_msg, unsigned long curr_millis);          // 0x23
    void receive_GENERAL_IO(const CAN_message_t& can_msg, unsigned long curr_millis);            // 0x24
    void receive_AC_CONFIG_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis);     // 0x25
    void receive_DC_CONFIG_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis);     // 0x26

    /* ---------- Sending ---------- */
    /*
     * Each sender is self-validating (clamps to its own operational range) since these are called
     * directly without a routing wrapper for now.
    */
    void send_AC_CURRENT(float target_ac_current_apk);
    void send_BRAKE_CURRENT(float target_brake_current_apk);
    void send_ERPM(float target_erpm);
    void send_MOTOR_POSITION(float target_motor_position_deg);
    void send_REL_AC_CURRENT(float target_ac_current_percent);
    void send_REL_AC_BRAKE_CURRENT(float target_rel_ac_brake_current_percent);

    // TODO: unimplemented — digital output usage/wiring not yet determined.
    // Signature likely needs to take the 4 output bools once that's known.
    void send_DIGITAL_OUTPUTS();

    void send_MAX_AC_CURRENT(float target_max_ac_current_apk);
    void send_MAX_AC_BRAKE_CURRENT(float target_max_ac_brake_current_apk);
    void send_MAX_DC_CURRENT(float target_max_dc_current_apk);
    void send_MAX_DC_BRAKE_CURRENT(float target_max_dc_brake_current_apk);
    void send_DRIVE_ENABLE();

    /* ---------- InverterFuncts_s-facing API ---------- */
    void set_torque(float torque_nm);
    void set_idle();
    void request_enable(bool enable);
    void request_error_reset();
    InverterStatus_s get_status() const;
    MotorMechanics_s get_motor_mechanics() const;

    /* ---------- DTI-specific diagnostics ---------- */
    DTIFaultCode_e get_fault_code() const;
    const StatusGeneralIOMsg_s& get_io_status() const;
    const StatusGeneralControlMsg_s& get_control_status() const;
    const DTIStatusMessages_s& get_all_inverter_data() const;

    uint8_t get_node_id() const { return _node_id; }

private:

    uint8_t _node_id;
    DTIParams_s _dti_params;

    bool _enable_requested = false;

    DTIStatusMessages_s _feedback_data = {};
    DTISetMessages_s _set_commands = {};

    bool _connected = false;
    unsigned long _last_recv_millis = 0;

    /// @brief Packs this inverter's node ID with the given packet ID into a
    /// standard 11-bit CAN ID. Layout: [Packet ID: bits 10:5] [Node ID: bits 4:0]
    uint16_t _pack_dti_can_id(uint8_t packet_id) const;

    float _torque_to_current_apk(float torque_nm) const;

};

#endif // INVERTERINTERFACE_H
