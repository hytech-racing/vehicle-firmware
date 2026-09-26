#ifndef PHYSICAL_PARAMETERS
#define PHYSICAL_PARAMETERS


namespace dti_motor_params
{
    constexpr float MOTOR_MAX_RPM                   = 20000.0f;
    constexpr torque_nm MOTOR_MAX_TORQUE_NM         = 31.6f;
    constexpr torque_nm MOTOR_MAX_REGEN_TORQUE_NM   = 10.0f;
    constexpr watt MOTOR_MAX_POWER_WATTS            = 35000.0f;
    constexpr float MOTOR_GEARBOX_RATIO             = 11.83f;
    constexpr meters WHEEL_DIAMETER_METERS          = 0.4064f;
}

namespace physical_motor_scales
{
    constexpr float RPM_TO_METERS_PER_SECOND    = dti_motor_params::WHEEL_DIAMETER_METERS * 3.1415f / dti_motor_params::MOTOR_GEARBOX_RATIO / 60.0f;
    constexpr float RPM_TO_KILOMETERS_PER_HOUR  = RPM_TO_METERS_PER_SECOND * 3600.0f / 1000.0f;
    constexpr float METERS_PER_SECOND_TO_RPM    = 1.0f / RPM_TO_METERS_PER_SECOND;
    constexpr float RPM_TO_RAD_PER_SECOND       = 2 * 3.1415f / 60.0f;
    constexpr float RAD_PER_SECOND_TO_RPM       = 1 / RPM_TO_RAD_PER_SECOND;
}

#endif /* PHYSICALPARAMETERS */