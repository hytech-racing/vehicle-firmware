#ifndef LOADCELLVECTORINGCONTROLLER
#define LOADCELLVECTORINGCONTROLLER

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"


namespace loadcell_vectoring_tc_default_params
{
    /**
     * @param REGEN_BIAS is a fraction (0.0 to 1.0)
     *
     *   0.0 = FWD (all torque to front)
     *   0.5 = AWD (balanced)
     *   1.0 = RWD (all torque to rear)
    */
    constexpr float REGEN_BIAS = 0.15f;    // front-biased under regen/braking, more torque to fronts

    constexpr size_t MAX_LOADCELL_ERROR_COUNT = 25;
    constexpr DrivetrainControlMode_e LOADCELL_CONTROLLER_MODE = DrivetrainControlMode_e::TORQUE;

    // TODO: Where deez values from
    constexpr float FL_LOADCELL_SCALE = 0.138796f;
    constexpr float FR_LOADCELL_SCALE = 0.135748f;
    constexpr float RL_LOADCELL_SCALE = 0.143619f;
    constexpr float RR_LOADCELL_SCALE = 0.137349f;

    constexpr float FL_LOADCELL_OFFSET = -33.7501f;
    constexpr float FR_LOADCELL_OFFSET = -29.665f;
    constexpr float RL_LOADCELL_OFFSET = -33.9947f;
    constexpr float RR_LOADCELL_OFFSET = -25.7212f;
}

struct LoadcellVectoringOffsets_s
{
    float fl_loadcell_offset;
    float fr_loadcell_offset;
    float rl_loadcell_offset;
    float rr_loadcell_offset;
};

struct LoadcellVectoringTCScales_s
{
    float fl_loadcell_scale;
    float fr_loadcell_scale;
    float rl_loadcell_scale;
    float rr_loadcell_scale;
};

struct LoadcellVectoringTCParams_s
{
    LoadcellVectoringOffsets_s offsets;
    LoadcellVectoringTCScales_s scales;
    size_t max_loadcell_error_count;
    DrivetrainControlMode_e control_mode;
    speed_rpm motor_max_rpm;
    torque_nm motor_max_torque_nm;
    torque_nm motor_max_regen_torque_nm;
    float regen_bias;
};

class LoadCellVectoringTorqueController
{
public:

    /**
     * @brief This is our Torque Controller (TC) which uses load-cell vectoring
     *        and corresponds to Mode 1. Accel torque is distributed per-wheel
     *        by measured normal force (load-cell-based vectoring). Regen
     *        torque uses a fixed front/rear bias only (no vectoring).
    */
    explicit LoadCellVectoringTorqueController(LoadcellVectoringTCParams_s params)
        : _params(params)
    {}

    /// @brief Default constructor — uses loadcell_vectoring_tc_default_params.
    LoadCellVectoringTorqueController()
        : _params {
            .offsets = {
                .fl_loadcell_offset = loadcell_vectoring_tc_default_params::FL_LOADCELL_OFFSET,
                .fr_loadcell_offset = loadcell_vectoring_tc_default_params::FR_LOADCELL_OFFSET,
                .rl_loadcell_offset = loadcell_vectoring_tc_default_params::RL_LOADCELL_OFFSET,
                .rr_loadcell_offset = loadcell_vectoring_tc_default_params::RR_LOADCELL_OFFSET
            },
            .scales = {
                .fl_loadcell_scale = loadcell_vectoring_tc_default_params::FL_LOADCELL_SCALE,
                .fr_loadcell_scale = loadcell_vectoring_tc_default_params::FR_LOADCELL_SCALE,
                .rl_loadcell_scale = loadcell_vectoring_tc_default_params::RL_LOADCELL_SCALE,
                .rr_loadcell_scale = loadcell_vectoring_tc_default_params::RR_LOADCELL_SCALE
            },
            .max_loadcell_error_count = loadcell_vectoring_tc_default_params::MAX_LOADCELL_ERROR_COUNT,
            .control_mode = loadcell_vectoring_tc_default_params::LOADCELL_CONTROLLER_MODE,
            .motor_max_rpm = dti_motor_params::MOTOR_MAX_RPM,
            .motor_max_torque_nm = dti_motor_params::MOTOR_MAX_TORQUE_NM,
            .motor_max_regen_torque_nm = dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM,
            .regen_bias = loadcell_vectoring_tc_default_params::REGEN_BIAS
        }
    {}

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, unsigned long curr_millis);

private:

    LoadcellVectoringTCParams_s _params;
    veh_vec<size_t> _loadcell_error_counts = {};

};

#endif