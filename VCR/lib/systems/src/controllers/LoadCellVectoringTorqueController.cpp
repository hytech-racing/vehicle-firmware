#include "controllers/LoadCellVectoringTorqueController.h"


DrivetrainCommand_s LoadCellVectoringTorqueController::evaluate(const VCRData_s &vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = {.torque_setpoints = {0.0f, 0.0f, 0.0f, 0.0f}};

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;
    const FrontLoadCellData_s &front_loadcell_data = vcr_data.interface_data.front_loadcell_data;
    const RearLoadCellData_s &rear_loadcell_data = vcr_data.interface_data.rear_loadcell_data;

    veh_vec<float> loadcell_data(static_cast<float>(front_loadcell_data.FL_loadcell_analog),
                                  static_cast<float>(front_loadcell_data.FR_loadcell_analog),
                                  static_cast<float>(rear_loadcell_data.RL_loadcell_analog),
                                  static_cast<float>(rear_loadcell_data.RR_loadcell_analog));

    // What are the unit and why did we not use the scale and offset?
    // veh_vec<float> loadcell_data(
    //     static_cast<float>(front_loadcell_data.FL_loadcell_analog) * _params.scales.fl_loadcell_scale + _params.offsets.fl_loadcell_offset,
    //     static_cast<float>(front_loadcell_data.FR_loadcell_analog) * _params.scales.fr_loadcell_scale + _params.offsets.fr_loadcell_offset,
    //     static_cast<float>(rear_loadcell_data.RL_loadcell_analog) * _params.scales.rl_loadcell_scale + _params.offsets.rl_loadcell_offset,
    //     static_cast<float>(rear_loadcell_data.RR_loadcell_analog) * _params.scales.rr_loadcell_scale + _params.offsets.rr_loadcell_offset
    // );

    // Track consecutive invalid samples per corner to detect a faulted/disconnected load cell
    _loadcell_error_counts.FL = front_loadcell_data.valid_FL_sample ? 0 : _loadcell_error_counts.FL + 1;
    _loadcell_error_counts.FR = front_loadcell_data.valid_FR_sample ? 0 : _loadcell_error_counts.FR + 1;
    _loadcell_error_counts.RL = rear_loadcell_data.valid_RL_sample ? 0 : _loadcell_error_counts.RL + 1;
    _loadcell_error_counts.RR = rear_loadcell_data.valid_RR_sample ? 0 : _loadcell_error_counts.RR + 1;

    bool are_all_load_cells_ok =
        (_loadcell_error_counts.FL < _params.max_loadcell_error_count) &&
        (_loadcell_error_counts.FR < _params.max_loadcell_error_count) &&
        (_loadcell_error_counts.RL < _params.max_loadcell_error_count) &&
        (_loadcell_error_counts.RR < _params.max_loadcell_error_count);

    if (!are_all_load_cells_ok)
    {
        // Too many consecutive invalid samples -> return zero torque
        return out;
    }

    float sum_normal_force = loadcell_data.FL + loadcell_data.FR + loadcell_data.RL + loadcell_data.RR;

    // TODO: guard against sum_normal_force being ~0 or negative or just unreasonable

    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;
    float torque_request = 0.0f;

    if (accel_request >= 0.0f)
    {
        /**
         * Positive torque request distributed to each corner; fraction of total measured normal force (load-cell-based vectoring).
         * We sum the normal force, so we also want to sum the torque request of all 4 corners before distributing based off load
         * which is why we multiply by 4.
        */
        torque_request = accel_request * dti_motor_params::MOTOR_MAX_TORQUE_NM * 4.0f;   // Why *4?

        out.torque_setpoints.FL = torque_request * _params.scales.front_torque_scale * loadcell_data.FL / sum_normal_force;
        out.torque_setpoints.FR = torque_request * _params.scales.front_torque_scale * loadcell_data.FR / sum_normal_force;
        out.torque_setpoints.RL = torque_request * _params.scales.rear_torque_scale * loadcell_data.RL / sum_normal_force;
        out.torque_setpoints.RR = torque_request * _params.scales.rear_torque_scale * loadcell_data.RR / sum_normal_force;
    }
    else
    {
        /**
         * Regen request. No load-cell vectoring applied to regen. We only use a fixed front/rear split.
         * accel_request is negative here, and DTI_REGEN_TORQUE is a positive constant, so torque_request comes
         * out negative. This sign matches the sign convention InverterInterface uses to route to
         * SET_BRAKE_CURRENT (negative torque) vs  SET_AC_CURRENT (non-negative).
        */
        torque_request = dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM * accel_request;

        out.torque_setpoints.FL = std::max(-_params.front_regen_limit, std::min(0.0f, torque_request * _params.scales.front_regen_torque_scale));
        out.torque_setpoints.FR = std::max(-_params.front_regen_limit, std::min(0.0f, torque_request * _params.scales.front_regen_torque_scale));
        out.torque_setpoints.RL = std::max(-_params.rear_regen_limit, std::min(0.0f, torque_request * _params.scales.rear_regen_torque_scale));
        out.torque_setpoints.RR = std::max(-_params.rear_regen_limit, std::min(0.0f, torque_request * _params.scales.rear_regen_torque_scale));
    }

    return out;
}