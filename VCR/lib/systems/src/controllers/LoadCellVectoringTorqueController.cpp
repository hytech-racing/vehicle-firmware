#include "controllers/LoadCellVectoringTorqueController.hpp"


DrivetrainCommand_s LoadCellVectoringTorqueController::evaluate(const VCRData_s& vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // speed rpms
        {0.0f, 0.0f, 0.0f, 0.0f}  // torques
    };

    const PedalsSystemData_s& pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;
    const FrontLoadCellData_s& front_lc_data = vcr_data.interface_data.front_loadcell_data;
    const RearLoadCellData_s& rear_lc_data = vcr_data.interface_data.rear_loadcell_data;

    veh_vec<float> load_cell_data(
        static_cast<float>(front_lc_data.fl_loadcell_analog),
        static_cast<float>(front_lc_data.fr_loadcell_analog),
        static_cast<float>(rear_lc_data.rl_loadcell_analog),
        static_cast<float>(rear_lc_data.rr_loadcell_analog));

    // Do sanity checks on raw data - FIX
    _load_cell_error_counts.fl = front_lc_data.valid_fl_sample ? 0 : _load_cell_error_counts.fl + 1;
    _load_cell_error_counts.fr = front_lc_data.valid_fr_sample ? 0 : _load_cell_error_counts.fr + 1;
    _load_cell_error_counts.rl = rear_lc_data.valid_rl_sample ? 0 : _load_cell_error_counts.rl + 1;
    _load_cell_error_counts.rr = rear_lc_data.valid_rr_sample ? 0 : _load_cell_error_counts.rr + 1;

    if (_load_cell_error_counts.fl < _max_error_count
        && _load_cell_error_counts.fr < _max_error_count
        && _load_cell_error_counts.rl < _max_error_count
        && _load_cell_error_counts.rr < _max_error_count)
    {
        // Calculate total normal force
        float sum_normal_force = load_cell_data.fl + load_cell_data.fr + load_cell_data.rl + load_cell_data.rr;

        // Both pedals are not pressed and no implausibility has been detected
        // accel_request goes between 1.0 and -1.0
        float accel_request = pedals_data.accel_percent - std::min(1.0f, (_brake_percent_scale * pedals_data.brake_percent));
        float torque_request = 0;

        if (accel_request >= 0.0)
        {
            // Positive torque request
            torque_request = accel_request * PhysicalParameters::AMK_MAX_TORQUE * 4;

            out.desired_speeds.fl = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.fr = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.rl = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.rr = PhysicalParameters::AMK_MAX_RPM;

            out.torque_limits.fl = torque_request * _front_torque_scale * load_cell_data.fl / sum_normal_force;
            out.torque_limits.fr = torque_request * _front_torque_scale * load_cell_data.fr / sum_normal_force;
            out.torque_limits.rl = torque_request * _rear_torque_scale * load_cell_data.rl / sum_normal_force;
            out.torque_limits.rr = torque_request * _rear_torque_scale * load_cell_data.rr / sum_normal_force;
        }
        else
        {
            // Negative torque request
            // No load cell vectoring on regen

            // RPM_TO_METERS_PER_SECOND

            torque_request = PhysicalParameters::MAX_REGEN_TORQUE * accel_request * -1.0F;

            out.desired_speeds = {0.0F, 0.0F, 0.0F, 0.0F};

            out.torque_limits.fl = std::min(_front_regen_limit, std::max(0.0f, torque_request * _front_regen_torque_scale));
            out.torque_limits.fr = std::min(_front_regen_limit, std::max(0.0f, torque_request * _front_regen_torque_scale));
            out.torque_limits.rl = std::min(_rear_regen_limit, std::max(0.0f, torque_request * _rear_regen_torque_scale));
            out.torque_limits.rr = std::min(_rear_regen_limit, std::max(0.0f, torque_request * _rear_regen_torque_scale));
        }
    }

    return out;
}
