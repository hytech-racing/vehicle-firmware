#ifndef PHYSICALPARAMETERS
#define PHYSICALPARAMETERS

namespace PhysicalParameters
{
    const float AMK_MAX_RPM = 16000;
    const float AMK_MAX_TORQUE = 21.0f;
    const float MAX_REGEN_TORQUE = 10.0f;
}

constexpr const float GEARBOX_RATIO               = 11.83f;
constexpr const float WHEEL_DIAMETER              = 0.4064f; // meters
constexpr const float RPM_TO_METERS_PER_SECOND    = WHEEL_DIAMETER * 3.1415f / GEARBOX_RATIO / 60.0f;
constexpr const float RPM_TO_KILOMETERS_PER_HOUR  = RPM_TO_METERS_PER_SECOND * 3600.0f / 1000.0f;
constexpr const float METERS_PER_SECOND_TO_RPM    = 1.0f / RPM_TO_METERS_PER_SECOND;

const float RPM_TO_RAD_PER_SECOND = 2 * 3.1415f / 60.0f;
const float RAD_PER_SECOND_TO_RPM = 1 / RPM_TO_RAD_PER_SECOND;

#endif /* PHYSICALPARAMETERS */