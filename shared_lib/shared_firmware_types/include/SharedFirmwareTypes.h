#ifndef __SHAREDFIRMWARETYPES_H__
#define __SHAREDFIRMWARETYPES_H__
#include <stdint.h>

#include <utility>
#include <array>
#include <stddef.h>

using speed_rpm = float;
using torque_nm = float;
using volt = float;
using celsius = float;
using time_ms = uint32_t;

/**
 * AnalogSensorStatus_e gets packaged along with the AnalogConversion_s struct as
 * the output of an AnalogChannel.
 */
enum class AnalogSensorStatus_e
{
    ANALOG_SENSOR_GOOD = 0,
    ANALOG_SENSOR_CLAMPED = 1,
};

/**
 * The AnalogConversion_s is the output struct for an AnalogChannel. It includes
 * the original analog value (for debugging purposes), the converted value according
 * to the configured scale, offset, and clamp, and the status (good or clamped).
 */
struct AnalogConversion_s
{
    int raw;
    float conversion;
    AnalogSensorStatus_e status;
};

/**
 * The AnalogConversionPacket_s is the output of an AnalogMultiSensor, which includes
 * each channel's output packet (an AnalogConversion_s). This is templated to account
 * for multi-sensors with different numbers of channels (2, 4, 8-channel ADCs).
 */
template <int N>
struct AnalogConversionPacket_s
{
    std::array <AnalogConversion_s, N> conversions;
};

/**
 * Generic data vector type that can be used for tires, load cells, or anything that has to do with
 * the four corners of the car.
 */
template <typename T>
struct veh_vec
{
public:
    T FL = {};
    T FR = {};
    T RL = {};
    T RR = {};

public:
    veh_vec() {};
    veh_vec(T _FL, T _FR, T _RL, T _RR)
    {
        FL = _FL;
        FR = _FR;
        RL = _RL;
        RR = _RR;
    }

    /// @brief copy values to array in FL, FR, RL, RR order
    void copy_to_arr(T (&arr_out)[4])
    {
        arr_out[0] = FL;
        arr_out[1] = FR;
        arr_out[2] = RL;
        arr_out[3] = RR;
    }

    std::array<T, 4> as_array()
    {
        return {FL, FR, RL, RR};
    }
    
};

struct TimestampedData_s
{
    unsigned long last_recv_millis = 0;
    bool recvd = false; // flag saying that this message has been received at least once 
};

template <typename T>
struct StampedVehVec : TimestampedData_s
{
    veh_vec<T> veh_vec_data;
};

struct FWVersionInfo
{
    std::array<char, 9> fw_version_hash = {"deadbeef"};
    bool project_on_main_or_master = false;
    bool project_is_dirty = false;
};

template <typename T>
struct xyz_vec
{
    T x;
    T y;
    T z;
};

template <typename T>
struct xy_vec
{
    T x;
    T y;
};

/**
 * Nested struct of analog pedals data (stored as int from 0-4095).
 */
struct PedalSensorData_s
{
    uint32_t accel_1;
    uint32_t accel_2;
    uint32_t brake_1;
    uint32_t brake_2;
};

struct FrontLoadCellData_s
{
    uint32_t FL_loadcell_analog;
    bool valid_FL_sample;
    uint32_t FR_loadcell_analog;
    bool valid_FR_sample;
};

/**
 * Since suspension potentiometers are only used for validation, it's OK to keep them in units of "analog".
 */
struct FrontSusPotData_s
{
    uint32_t FL_sus_pot_analog;
    uint32_t FR_sus_pot_analog;
};

/**
 * Non digitally-filtered steering data.
 */
struct SteeringSensorData_s
{
    // Analog steering sensor data, in degrees.
    float analog_steering_degrees;
    // Digital steering sensor data, unconverted (0-4095)
    float digital_steering_analog;
};

enum class SteeringEncoderStatus_e
{
    NOMINAL = 0,
    ERROR = 1,
};

struct EncoderErrorFlags_s
{
    bool dataInvalid              = false;
    bool operatingLimit           = false;
    bool noData                   = false;
};

struct SteeringEncoderReading_s
{
    float angle = 0.0f;
    int rawValue = 0;
    SteeringEncoderStatus_e status = SteeringEncoderStatus_e::NOMINAL;
    EncoderErrorFlags_s errors;
};

/**
 * Enum for the modes on the dial, corresponds directly to dial index position.
 */
enum class ControllerMode_e
{
    MODE_0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_5,
};

struct DashInputState_s
{
    bool btn_dim_read_is_pressed : 1;
    bool brightness_ctrl_btn_is_pressed : 1;
    bool preset_btn_is_pressed : 1;
    bool mc_reset_btn_is_pressed : 1; // Resets the motor controller errors
    bool start_btn_is_pressed : 1; // The start button is the READY_TO_DRIVE button
    bool data_btn_is_pressed : 1;
    bool BUTTON_2 : 1;
    ControllerMode_e dial_state;
};

/**
 * Copied from HT08 MCU, but with clearer names.
 */
struct PedalsSystemData_s
{
    bool accel_is_implausible : 1; // Checks if either accel pedal is out of range OR they disagree by more than 10%
    bool brake_is_implausible : 1; // Checks if brake sensor is out of range.
    bool brake_is_pressed : 1; // True if brake pedal is pressed beyond the specified activationPercentage.
    bool accel_is_pressed : 1; // True if the accel pedal is pressed beyond the specified activationPercentage.
    bool mech_brake_is_active : 1; // True if the brake pedal is pressed beyond mechanical_activation_percentage.
    bool brake_and_accel_pressed_implausibility_high : 1; // If accel is pressed at all while mech_brake_is_active.
    bool implausibility_has_exceeded_max_duration : 1; // True if implausibility lasts more than 100ms
    float accel_percent;
    float brake_percent;
    float regen_percent; // When brake pedal is 0%, regen_percent is 0.0. When brakes are at mechanical_activation_percentage,
                         // regen_percent is at 1.0. For instance, if mech activation percentage was 60%, then when brake
                         // travel is at 40%, regen_percent would be 0.667. Beyond that, regen_percent is clamped to 1.0.
};

struct SteeringSystemData_s
{
    uint32_t analog_raw;
    uint32_t digital_raw;

    float analog_steering_angle; //in degrees
    float digital_steering_angle; //in degrees
    float output_steering_angle; // represents the better output of the two sensors or some combination of the values

    float analog_steering_velocity_deg_s; //in degrees per second
    float digital_steering_velocity_deg_s;

    bool digital_oor_implausibility;
    bool analog_oor_implausibility;
    bool sensor_disagreement_implausibility;
    bool dtheta_exceeded_analog;
    bool dtheta_exceeded_digital;
    bool both_sensors_fail;
    bool interface_sensor_error;
};

struct RearLoadCellData_s
{
    uint32_t RL_loadcell_analog;
    bool valid_RL_sample;
    uint32_t RR_loadcell_analog;
    bool valid_RR_sample;
};

/**
 * Since suspension potentiometers are only used for validation, it's OK to keep them in units of "analog".
 */
struct RearSusPotData_s
{
    uint32_t RL_sus_pot_analog;
    uint32_t RR_sus_pot_analog;
};

/**
 * Creating Thermistor Data structure to account for storage of analog readings and actual temperature in degrees celsius
*/
struct ThermistorData_s
{
    uint32_t thermistor_analog;
    float thermistor_degrees_C;
};

struct VCRThermistorData_s
{
    ThermistorData_s thermistor_0;
    ThermistorData_s thermistor_1;
    ThermistorData_s thermistor_2;
    ThermistorData_s thermistor_3;
};

struct FlowmeterData_s
{
  float flowmeter_gallons_per_min;
};

/**
 * Directly copied from HT08 MCU SharedDataTypes.h.
 */
struct VectorNavData_s
{
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    float uncompLinear_accel[3]; // 3D uncompensated linear acceleration
    float yaw;
    float pitch;
    float roll;
    double latitude;
    double longitude;
    double ecef_coords[3]; // x,y,z
    uint64_t gps_time;     // gps time
    uint8_t vn_status;     // status
    xyz_vec<float> angular_rates;
};

struct CurrentSensorData_s
{
    float twentyfour_volt_sensor; // Senses the 24V power line
    float current_sensor_unfiltered;
    float current_refererence_unfiltered;

    bool bspd_brake_high_sense;
    bool bspd_current_high_sense;
};

/**
 * The signals beginning with a letter prefix are according to this page (https://wiki.hytechracing.org/books/ht09-design/page/shutdown-circuit-order),
 * and are on the shutdown line. Signals without the letter prefix are the inputs to those shutdown relays that determine whether or not they close
 * when the latch button is pressed. VCR has FOUR relays. Since each shutdown "letter" is a node between shutdown components, that means VCR has five
 * nodes to probe and four relay inputs to probe (total of 9 booleans).
 */
struct ShutdownSensingData_s
{   

    // Shutdown inputs
    bool bspd_is_ok : 1;
    bool imd_is_ok : 1;
    bool bms_is_ok : 1;
    bool vcr_sw_is_ok : 1;

    // Shutdown relays
    bool shdn_h_is_ok : 1;
    bool shdn_i_is_ok : 1;
    bool shdn_j_is_ok : 1;
    bool shdn_k_is_ok : 1;
    bool shdn_l_is_ok : 1;

    // Ethernet links
    bool acu_is_linked : 1;
    bool teensy_is_linked : 1;
    bool vcf_is_linked : 1;
    bool ubiquiti_is_linked : 1;
    bool db_is_linked : 1;

};

/**
 * The 'link' lights from the ethernet switch to indicate whether or not each item is connected.
 */
struct VCREthernetLinkData_s
{
    bool acu_link : 1;
    bool drivebrain_link : 1;
    bool vcf_link : 1;
    bool teensy_link : 1;
    bool debug_link : 1;
    bool ubiquiti_link : 1;
    bool vn_link: 1;
};

/**
 * The 'link' data from the VCF ethernet switch to indicate whether or not each item is connected.
 */
struct VCFEthernetLinkData_s
{
    bool poe_1_link : 1;
    bool poe_2_link : 1;
    bool mag_3_link : 1;
    bool vcr_link : 1;
    bool teensy_link : 1;
    bool dash_link : 1;
};

/**
 * A collection of all the data InverterInterface.tpp used to send individually over CAN. Now,
 * using ethernet, we can bundle all of these values together for better organization.
 */
struct InverterData_s
{
    bool system_ready : 1;
    bool error : 1;
    bool warning : 1;
    bool quit_dc_on : 1;
    bool dc_on : 1;
    bool quit_inverter_on : 1;
    bool inverter_on : 1;
    bool derating_on : 1;
    int speed_rpm;
    int actual_motor_torque;
    int commanded_torque;
    int motor_temp;
    int inverter_temp;
    int diagnostic_number;
    int igbt_temp;
    int dc_bus_voltage;
    int actual_power;
    int feedback_torque;
};

/**
 * Forwarded directly from CAN with no additional transformations.
 */
struct EnergyMeterData_s
{
    float em_current; // Current, in amps, from the EM
    float em_voltage; // Voltage, in volts, from the EM.
};

/// @brief Defines modes of torque limit to be processed in torque limit map for exact values.
enum class TorqueLimit_e
{
    TCMUX_FULL_TORQUE = 0,
    TCMUX_MID_TORQUE = 1,
    TCMUX_LOW_TORQUE = 2,
    TCMUX_NUM_TORQUE_LIMITS = 3,
};

/// @brief Defines errors for TC Mux to use to maintain system safety
enum class TorqueControllerMuxError_e
{
    NO_ERROR = 0,
    ERROR_SPEED_DIFF_TOO_HIGH = 1,
    ERROR_TORQUE_DIFF_TOO_HIGH = 2,
    ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS =3,
    ERROR_CONTROLLER_NULL_POINTER =4
};

/// @brief packages TC Mux indicators: errors, mode, torque limit, bypass
struct TorqueControllerMuxStatus_s
{
    TorqueControllerMuxError_e active_error;
    ControllerMode_e active_controller_mode;
    ControllerMode_e prev_controller_mode;
    TorqueLimit_e active_torque_limit_enum;
    float active_torque_limit_value;
    bool output_is_bypassing_limits;
};



/// @brief Stores setpoints for a command to the Drivetrain, containing speed setpoints and torque limits for each motor. These setpoints are defined in the torque controllers cycled by the TC Muxer. 
/// The Speeds unit is rpm and are the targeted speeds for each wheel of the car.
/// The torques unit is nm and is the max torque requested from the inverter to reach such speeds.
struct DrivetrainCommand_s
{
    veh_vec<speed_rpm> desired_speeds;
    veh_vec<torque_nm> torque_limits;
};

struct StampedDrivetrainCommand_s
{
    StampedVehVec<speed_rpm> desired_speeds;
    StampedVehVec<torque_nm> torque_limits;

    DrivetrainCommand_s get_command()
    {
        return {.desired_speeds= desired_speeds.veh_vec_data, 
                .torque_limits = torque_limits.veh_vec_data};
    }
};

struct DrivebrainMessageLatencyInfo_s {
    bool timing_failure; 
    unsigned long worst_latency_millis; 

    bool speed_setpoint_msg_too_latent; 
    bool torque_limit_message_too_latent;
    bool not_all_messages_received; 
    bool latency_diff_too_high;
};

struct DrivetrainTorqueCommand_s
{
    veh_vec<torque_nm> torque_limits;
    veh_vec<torque_nm> torque_setpoints;
};

struct StampedDrivetrainTorqueCommand_s
{
    StampedVehVec<speed_rpm> torque_limits;
    StampedVehVec<torque_nm> torque_setpoints;

    DrivetrainTorqueCommand_s get_command()
    {
        return {.torque_limits= torque_limits.veh_vec_data, 
                .torque_setpoints = torque_setpoints.veh_vec_data};
    }
};

struct DrivetrainDynamicReport_s
{
    uint16_t measuredInverterFLPackVoltage;
    veh_vec<speed_rpm> measuredSpeeds;
    veh_vec<torque_nm> measuredTorques;
    veh_vec<float> measuredTorqueCurrents;
    veh_vec<float> measuredMagnetizingCurrents;
};

/**
 * Output data for the ACU Heartbeat. This struct is different from ACUCoreData and
 * ACUAllData because those contain information that is processed and sent by the
 * ACU. All values are calculated FROM ACUAllData.
 */
struct ACUHeartbeatData_s
{
    bool heartbeat_ok = false;
    unsigned long last_heartbeat_time = 0;
};

enum class ACUState_e
{
    STARTUP = 0, 
    ACTIVE = 1, 
    CHARGING = 2, 
    FAULTED = 3,
    WELDED = 4,
    WELDCHECK = 5 
};

/**
 * Minimum data that the ACU must send for the car to run. Detailed temps/voltages are not minimum-viable. The
 * ACUAllData message contains an instance of the data in this ACUCoreData struct.
 */
struct ACUCoreData_s
{
    volt pack_voltage;
    volt min_cell_voltage;
    volt avg_cell_voltage;
    volt max_cell_voltage;
    celsius max_cell_temp;
    celsius min_cell_temp;
    celsius max_board_temp;
    volt max_measured_pack_out_voltage;
    volt max_measured_ts_out_voltage;
    volt max_measured_glv;
    volt min_measured_pack_out_voltage;
    volt min_measured_ts_out_voltage;
    volt min_measured_glv;
    volt min_shdn_out_voltage;
    volt hv_plus_out_voltage;
    volt main_ok_voltage;
    volt precharge_ok_voltage;
    volt main_under_threshold_voltage;
    volt precharge_under_threshold_voltage;
    float tractive_system_current;
    ACUState_e acu_sm_state;

    bool high_side_contactor_welded;
    bool low_side_contactor_welded;
};

struct StampedACUCoreData_s : TimestampedData_s
{
    ACUCoreData_s acu_data;
};

/**
 * ACUAllData contains the detailed, unprocessed data from ACU sensors.
 */
template<size_t num_cells, size_t num_cell_temps, size_t num_chips>
struct ACUAllData_s
{
    ACUCoreData_s core_data;
    size_t max_consecutive_invalid_packet_count;
    size_t max_cell_voltage_id;
    size_t min_cell_voltage_id;
    size_t max_cell_temp_id;
    size_t max_board_temp_id;
    volt measured_tractive_system_voltage; 
    volt measured_pack_voltage;
    volt measured_shdn_voltage;
    float measured_bspd_current;
    FWVersionInfo fw_version_info;
    float valid_packet_rate;
    float SoC;
    float SoH;
    float lifetime_ah_throughput;
    float SoE_percentage;
    float V1;
    double remaining_pack_wh;    
    std::array<size_t, num_chips> consecutive_invalid_packet_counts;
    std::array<volt, num_cells> cell_voltages;
    std::array<celsius, num_cell_temps> cell_temps; 
    std::array<celsius, num_chips> board_temps;
    bool shutdown_has_gone_low;
};

using ACUAllDataType_s = ACUAllData_s<126, 48, 12>;

/**
 * Timestamped pedals data. Extends TimestampedData_s to include a received timestamp, in milliseconds.
 */
struct StampedPedalsSystemData_s : TimestampedData_s
{
    PedalsSystemData_s pedals_data;
    bool heartbeat_ok = false;
};

/**
 * Timestamped steering data. Extends TimestampedData_s to include a received timestamp in milliseconds.
 */
struct StampedSteeringSystemData_s : TimestampedData_s
{
    SteeringSystemData_s steering_data;
    bool heartbeat_ok = false;
};

/**
 * Struct containing the VCR systems' data. These are generally the outputs of VCR systems.
 */
struct VCFSystemData_s
{
    PedalsSystemData_s pedals_system_data;
    bool buzzer_is_active = false;
};

/**
 * Struct containing the VCF interfaces' data. An instance of this will be passed into the
 * evaluation of the VCF systems.
 */
struct VCFInterfaceData_s
{
    PedalSensorData_s pedal_sensor_data;
    FrontLoadCellData_s front_loadcell_data;
    FrontSusPotData_s front_suspot_data;
    SteeringSensorData_s steering_data;
    DashInputState_s dash_input_state; // Direct button signals from the dashboard IOExpander
    CurrentSensorData_s current_sensor_data;
    VCFEthernetLinkData_s vcf_ethernet_link_data;
};

/**
 * All system AND interface data in VCF. VCF systems will place data in some of the nested structs, while
 * systems will place data in some of the other structs.
 */
struct VCFData_s
{
    VCFSystemData_s system_data;
    VCFInterfaceData_s interface_data;
};

/**
 * Determines whether VCR has receied a message from VCF recently.
 */
struct VCFHeartbeatData_s
{
    bool heartbeat_ok = false;
    unsigned long last_heartbeat_time = 0;
};

enum class VehicleState_e {
    TRACTIVE_SYSTEM_NOT_ACTIVE = 0,
    TRACTIVE_SYSTEM_ACTIVE = 1,
    WANTING_READY_TO_DRIVE = 2,
    READY_TO_DRIVE = 3,
    WANTING_RECALIBRATE_PEDALS = 4,
    RECALIBRATING_PEDALS = 5,
    WANTING_RECALIBRATE_STEERING = 6,
    RECALIBRATING_STEERING = 7
};

enum class DrivetrainState_e
{
    NOT_CONNECTED = 0,
    NOT_ENABLED_NO_HV_PRESENT = 1,
    NOT_ENABLED_HV_PRESENT = 2,
    INVERTERS_READY = 3,
    INVERTERS_HV_ENABLED = 4,
    ENABLED_DRIVE_MODE = 5,
    ERROR = 6, 
    CLEARING_ERRORS = 7
};

struct DrivebrainControllerStatus_s 
{
    bool drivebrain_is_in_control = false;
    bool drivebrain_controller_timing_failure = false;
};
/**
 * Struct containing the location data for the Loss of communication of VCR with other other ECUs.
 */
struct VCRLOCData {
    bool acu_loc = false;
    bool vcf_loc = false;
};
/**
 * Struct containing the VCR systems' data. These are generally the outputs of VCR systems.
 */
struct VCRSystemData_s
{
    ACUHeartbeatData_s acu_heartbeat_data = {};
    DrivetrainDynamicReport_s drivetrain_data = {};
    TorqueControllerMuxStatus_s tc_mux_status = {};
    VCFHeartbeatData_s vcf_heartbeat_data = {};
    VehicleState_e vehicle_state_machine_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    DrivetrainState_e drivetrain_state_machine_state = DrivetrainState_e::NOT_CONNECTED;
    bool buzzer_is_active = false;
    DrivebrainControllerStatus_s db_cntrl_status = {};
    VCRLOCData vcr_loc_data = {};
    DrivebrainMessageLatencyInfo_s aux_latency_info; 
    DrivebrainMessageLatencyInfo_s telem_latency_info;
};

/**
 * Struct containing the VCR interfaces' data. An instance of this will be passed into the
 * evaluation of the VCR systems.
 */
struct VCRInterfaceData_s
{
    VCREthernetLinkData_s ethernet_is_linked = {};
    ShutdownSensingData_s shutdown_sensing_data = {};
    RearLoadCellData_s rear_loadcell_data = {};
    RearSusPotData_s rear_suspot_data = {};
    FrontLoadCellData_s front_loadcell_data = {};
    FrontSusPotData_s front_suspot_data = {};
    VCRThermistorData_s thermistor_data = {};
    FlowmeterData_s flowmeter_data = {};
    CurrentSensorData_s current_sensor_data = {};
    StampedPedalsSystemData_s recvd_pedals_data = {};
    veh_vec<InverterData_s> inverter_data = {};
    DashInputState_s dash_input_state = {};
    StampedACUCoreData_s stamped_acu_core_data = {};
    ACUAllDataType_s acu_all_data = {};
    StampedDrivetrainCommand_s latest_drivebrain_telem_command = {};
    StampedDrivetrainCommand_s latest_drivebrain_auxillary_command = {};
};

struct VCRData_s
{
    VCRSystemData_s system_data;
    VCRInterfaceData_s interface_data;
    FWVersionInfo fw_version_info;
};

#endif // __SHAREDFIRMWARETYPES_H__
