#include "controllers/LoadCellVectoringTorqueController.h"


DrivetrainCommand_s LoadCellVectoringTorqueController::evaluate(const VCRData_s &vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = { .control_mode = _params.control_mode,
                                .desired_torques = {0.0f, 0.0f, 0.0f, 0.0f},
                                .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}};

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;
    const FrontLoadCellData_s &front_loadcell_data = vcr_data.interface_data.front_loadcell_data;
    const RearLoadCellData_s &rear_loadcell_data = vcr_data.interface_data.rear_loadcell_data;

    /// TODO: What deez units
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

    /// TODO: guard against sum_normal_force being ~0, negative, or otherwise unreasonable

    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;

    if (_params.control_mode == DrivetrainControlMode_e::TORQUE)
    {
        if (accel_request >= 0.0f)
        {
            /**
             * @note Total torque budget across all 4 motors (motor_max_torque_nm is a single
             *       motor's rating, which is why we do *4). Distributed to each corner purely
             *       by fraction of total measured normal force (load-cell-based vectoring) — no
             *       fixed front/rear bias is applied here. Since the four load-cell fractions
             *       always sum to 1, the total delivered torque always equals total_torque_request
             *       exactly, regardless of actual weight distribution.
            */
            float total_torque_request = accel_request * _params.motor_max_torque_nm * 4.0f;

            out.desired_torques.FL = total_torque_request * loadcell_data.FL / sum_normal_force;
            out.desired_torques.FR = total_torque_request * loadcell_data.FR / sum_normal_force;
            out.desired_torques.RL = total_torque_request * loadcell_data.RL / sum_normal_force;
            out.desired_torques.RR = total_torque_request * loadcell_data.RR / sum_normal_force;
        }
        else
        {
            /**
             * @note Regen request. No load-cell vectoring applied to regen, just a fixed front/rear bias
             *       only, via regen_bias. accel_request is negative here and MOTOR_MAX_REGEN_TORQUE_NM
             *       is positive, so total_torque_request comes out negative naturally, matching the
             *       sign convention InverterInterface uses to route to SET_BRAKE_CURRENT vs SET_AC_CURRENT.
            */
            float total_torque_request = dti_motor_params::MOTOR_MAX_REGEN_TORQUE_NM * accel_request;

            float rear_torque_fraction = _params.regen_bias;
            float front_torque_fraction = 1.0f - rear_torque_fraction;

            float rears_torque_share = total_torque_request * rear_torque_fraction;
            float fronts_torque_share = total_torque_request * front_torque_fraction;

            out.desired_torques.FL = std::max(-_params.front_regen_limit, std::min(0.0f, fronts_torque_share / 2.0f));
            out.desired_torques.FR = std::max(-_params.front_regen_limit, std::min(0.0f, fronts_torque_share / 2.0f));
            out.desired_torques.RL = std::max(-_params.rear_regen_limit, std::min(0.0f, rears_torque_share / 2.0f));
            out.desired_torques.RR = std::max(-_params.rear_regen_limit, std::min(0.0f, rears_torque_share / 2.0f));
        }
    }
    else   // SPEED
    {
        // Same bias/split logic as TORQUE mode, just producing a speed target instead of a torque target.
        if (accel_request >= 0.0f)
        {
            float total_speed_request = accel_request * _params.motor_max_rpm * 4.0f;

            out.desired_speeds.FL = total_speed_request * loadcell_data.FL / sum_normal_force;
            out.desired_speeds.FR = total_speed_request * loadcell_data.FR / sum_normal_force;
            out.desired_speeds.RL = total_speed_request * loadcell_data.RL / sum_normal_force;
            out.desired_speeds.RR = total_speed_request * loadcell_data.RR / sum_normal_force;
        }
        else
        {
            /**
             * @note SET_ERPM's sign represents spin direction, not regen/braking
             *       Reverse rotation is what we want I beleive
            */
            float total_speed_request = accel_request * _params.motor_max_rpm;

            float rear_speed_fraction = _params.regen_bias;
            float front_speed_fraction = 1.0f - rear_speed_fraction;

            float rears_speed_share = total_speed_request * rear_speed_fraction;
            float fronts_speed_share = total_speed_request * front_speed_fraction;

            out.desired_speeds.FL = fronts_speed_share / 2.0f;
            out.desired_speeds.FR = fronts_speed_share / 2.0f;
            out.desired_speeds.RL = rears_speed_share / 2.0f;
            out.desired_speeds.RR = rears_speed_share / 2.0f;
        }
    }

    return out;
}