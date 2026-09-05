#include "controllers/LoadCellVectoringTorqueController.h"


DrivetrainCommand_s LoadCellVectoringTorqueController::evaluate(const VCRData_s &vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = { .control_mode = _params.control_mode,
                                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                                .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}};

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;
    const FrontLoadCellData_s &front_loadcell_data = vcr_data.interface_data.front_loadcell_data;
    const RearLoadCellData_s &rear_loadcell_data = vcr_data.interface_data.rear_loadcell_data;

    // what are the units???
    veh_vec<float> loadcell_data(
        static_cast<float>(front_loadcell_data.FL_loadcell_analog) * _params.scales.fl_loadcell_scale + _params.offsets.fl_loadcell_offset,
        static_cast<float>(front_loadcell_data.FR_loadcell_analog) * _params.scales.fr_loadcell_scale + _params.offsets.fr_loadcell_offset,
        static_cast<float>(rear_loadcell_data.RL_loadcell_analog) * _params.scales.rl_loadcell_scale + _params.offsets.rl_loadcell_offset,
        static_cast<float>(rear_loadcell_data.RR_loadcell_analog) * _params.scales.rr_loadcell_scale + _params.offsets.rr_loadcell_offset
    );

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
        // Too many consecutive invalid samples -> return zero command
        return out;
    }

    float sum_normal_force = loadcell_data.FL + loadcell_data.FR + loadcell_data.RL + loadcell_data.RR;

    // TODO: Add someguard against sum_normal_force being ~0, negative, or otherwise
    // unreasonable

    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;

    if (_params.control_mode == DrivetrainControlMode_e::TORQUE)
    {
        float torque_request = 0.0f;

        if (accel_request >= 0.0f)
        {
            /**
             * @note Positive torque request distributed to each corner by fraction of total measured
             *       normal force (load-cell-based vectoring). The four load-cell fractions sum to 1,
             *       so torque_request is scaled to a 4-motor budget (*4) before distributing, so each
             *       wheel can reach up to one full motor's torque at even weight distribution.
            */
            torque_request = accel_request * dti_motor_params::MOTOR_MAX_TORQUE_NM * 4.0f;

            out.desired_torques.FL = torque_request * _params.scales.front_torque_scale * loadcell_data.FL / sum_normal_force;
            out.desired_torques.FR = torque_request * _params.scales.front_torque_scale * loadcell_data.FR / sum_normal_force;
            out.desired_torques.RL = torque_request * _params.scales.rear_torque_scale * loadcell_data.RL / sum_normal_force;
            out.desired_torques.RR = torque_request * _params.scales.rear_torque_scale * loadcell_data.RR / sum_normal_force;
        }
        else
        {
            /**
             * @note Regen request. No load-cell vectoring applied to regen. We will just use fixed front/rear split only.
             *       accel_request is negative here and MOTOR_MAX_REGEN_TORQUE_NM is positive, so torque_request comes out
             *       negative naturally. Sign just signals regen/braking, matching how InverterInterface
             *       routes negative torque to SET_BRAKE_CURRENT.
            */
            torque_request = dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM * accel_request;

            out.desired_torques.FL = std::max(-_params.front_regen_limit, std::min(0.0f, torque_request * _params.scales.front_regen_torque_scale));
            out.desired_torques.FR = std::max(-_params.front_regen_limit, std::min(0.0f, torque_request * _params.scales.front_regen_torque_scale));
            out.desired_torques.RL = std::max(-_params.rear_regen_limit, std::min(0.0f, torque_request * _params.scales.rear_regen_torque_scale));
            out.desired_torques.RR = std::max(-_params.rear_regen_limit, std::min(0.0f, torque_request * _params.scales.rear_regen_torque_scale));
        }
    }
    else   // SPEED CONTROL
    {
        float speed_request = accel_request * dti_motor_params::MOTOR_MAX_RPM;

        out.desired_speeds.FL = speed_request;
        out.desired_speeds.FR = speed_request;
        out.desired_speeds.RL = speed_request;
        out.desired_speeds.RR = speed_request;
    }

    return out;
}