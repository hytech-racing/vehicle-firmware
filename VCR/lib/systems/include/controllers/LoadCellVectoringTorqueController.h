#ifndef LOADCELLVECTORINGCONTROLLER
#define LOADCELLVECTORINGCONTROLLER

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"


namespace loadcell_vectoring_tc_default_params
{
    constexpr float FRONT_REGEN_LIMIT = 13.0f;
    constexpr float REAR_REGEN_LIMIT = 3.5f;
    constexpr size_t MAX_LOADCELL_ERROR_COUNT = 25;
    constexpr DrivetrainControlMode_e LOADCELL_CONTROLLER_MODE = DrivetrainControlMode_e::TORQUE;

    /**
     *  @param REAR_TORQUE_SCALE 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = 50/50, 2 = RWD
     *  @param REAR_REGEN_TORQUE_SCALE same as rear_torque_scale but applies to regen torque split.
     *                                 0 = all regen torque on fronts
     *                                 1 = 50/50
     *                                 2 = all regen torque on rears
    */
    constexpr float FRONT_TORQUE_SCALE = 1.0;
    constexpr float REAR_TORQUE_SCALE = 1.0f;
    constexpr float FRONT_REGEN_TORQUE_SCALE = 1.7;
    constexpr float REAR_REGEN_TORQUE_SCALE = 0.3f;

    // Where did we get these values from?
    constexpr float FL_LOADCELL_SCALE = 0.138796f;
    constexpr float FR_LOADCELL_SCALE = 0.138796f;
    constexpr float RL_LOADCELL_SCALE = 0.143619f;
    constexpr float RR_LOADCELL_SCALE = 0.137349f;

    // Where did we get these values from?
    constexpr float FL_LOADCELL_OFFSET = -33.7501f;
    constexpr float FR_LOADCELL_OFFSET = -29.665f;
    constexpr float RL_LOADCELL_OFFSET = -33.9947f;
    constexpr float RR_LOADCELL_OFFSET = -25.7212f;
}

struct LoadcellVectoringOffsets_s
{
    const float fl_loadcell_offset;
    const float fr_loadcell_offset;
    const float rl_loadcell_offset;
    const float rr_loadcell_offset;
};

struct LoadcellVectoringTCScales_s
{
    float front_torque_scale;
    float front_regen_torque_scale;
    float rear_torque_scale;
    float rear_regen_torque_scale;

    const float fl_loadcell_scale;
    const float fr_loadcell_scale;
    const float rl_loadcell_scale;
    const float rr_loadcell_scale;
};

struct LoadcellVectoringTCParams_s
{
    LoadcellVectoringOffsets_s offsets;
    LoadcellVectoringTCScales_s scales;
    const float front_regen_limit;
    const float rear_regen_limit;
    const float max_loadcell_error_count;
    DrivetrainControlMode_e control_mode;
};


class LoadCellVectoringTorqueController
{
public:

    /**
     * @brief This is our Torque Controller (TC) which uses loadcell vectoring and corresponds to Mode 1.
     *        This TC has tunable F/R torque balance, as well as accel/regen torque balance (tuned independently)
    */
    explicit LoadCellVectoringTorqueController(LoadcellVectoringTCParams_s params)
        : _params(params)
    {};

    /**
     * @brief Default Constructor
    */
    LoadCellVectoringTorqueController()
        : _params {
            .offsets = {
                .fl_loadcell_offset = loadcell_vectoring_tc_default_params::FL_LOADCELL_OFFSET,
                .fr_loadcell_offset = loadcell_vectoring_tc_default_params::FR_LOADCELL_OFFSET,
                .rl_loadcell_offset = loadcell_vectoring_tc_default_params::RL_LOADCELL_OFFSET,
                .rr_loadcell_offset = loadcell_vectoring_tc_default_params::RR_LOADCELL_OFFSET
            },
            .scales = {
                .front_torque_scale = loadcell_vectoring_tc_default_params::FRONT_TORQUE_SCALE,
                .front_regen_torque_scale = loadcell_vectoring_tc_default_params::FRONT_REGEN_TORQUE_SCALE,
                .rear_torque_scale = loadcell_vectoring_tc_default_params::REAR_TORQUE_SCALE,
                .rear_regen_torque_scale = loadcell_vectoring_tc_default_params::REAR_REGEN_TORQUE_SCALE,
                .fl_loadcell_scale = loadcell_vectoring_tc_default_params::FL_LOADCELL_SCALE,
                .fr_loadcell_scale = loadcell_vectoring_tc_default_params::FR_LOADCELL_SCALE,
                .rl_loadcell_scale = loadcell_vectoring_tc_default_params::RL_LOADCELL_SCALE,
                .rr_loadcell_scale = loadcell_vectoring_tc_default_params::RR_LOADCELL_SCALE
            },
            .front_regen_limit = loadcell_vectoring_tc_default_params::FRONT_REGEN_LIMIT,
            .rear_regen_limit = loadcell_vectoring_tc_default_params::REAR_REGEN_LIMIT,
            .max_loadcell_error_count = loadcell_vectoring_tc_default_params::MAX_LOADCELL_ERROR_COUNT,
            .control_mode = loadcell_vectoring_tc_default_params::LOADCELL_CONTROLLER_MODE
        }
    {};

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, unsigned long curr_millis);

private:

    LoadcellVectoringTCParams_s _params;
    veh_vec<size_t> _loadcell_error_counts = {};

};

#endif