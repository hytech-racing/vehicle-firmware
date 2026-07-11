#ifndef SIMPLECONTROLLER
#define SIMPLECONTROLLER

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Controller Includes */
#include "PhysicalParameters.h"


/// @param rear_torque_scale 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = Balanced, 2 = RWD
/// @param rear_regen_torque_scale same as rear_torque_scale but applies to regen torque split. 0 = All regen torque on the front, 1 = 50/50, 2 = all regen torque on the rear

struct TorqueControllerSimpleParams_s
{
public:

    float rear_torque_scale = {};
    float rear_regen_torque_scale = {};
    speed_rpm amk_max_rpm = {};
    torque_nm amk_max_torque = {};
    torque_nm amk_max_regen_torque = {};

    TorqueControllerSimpleParams_s() : rear_torque_scale(1.3f), //more torque to rears
                                    rear_regen_torque_scale(0.3f), //more regen to fronts
                                    amk_max_rpm(_amk_max_rpm_default),
                                    amk_max_torque(_amk_max_torque),
                                    amk_max_regen_torque(_amk_max_regen_torque)
    {};

    TorqueControllerSimpleParams_s(float rts,
                                float rrts, float max_rpm,
                                float max_torq,
                                float max_reg_torq
    ) : rear_torque_scale(rts),
        rear_regen_torque_scale(rrts),
        amk_max_rpm(max_rpm),
        amk_max_torque(max_torq),
        amk_max_regen_torque(max_reg_torq)
    {};

private:

    static constexpr speed_rpm _amk_max_rpm_default = 20000.0f;
    // static constexpr speed_rpm _amk_max_rpm_default = METERS_PER_SECOND_TO_RPM * 20;
    static constexpr torque_nm _amk_max_torque = 21.0f;
    static constexpr torque_nm _amk_max_regen_torque = 15.0f;

};

class TorqueControllerSimple
{
public:

    /// @brief simple TC with tunable F/R torque balance. Accel torque balance can be tuned independently of regen torque balance
    TorqueControllerSimple(TorqueControllerSimpleParams_s params = TorqueControllerSimpleParams_s()) : _params(params) {};

    /// @brief calculates torque output based off max torque and simple torque scaling
    DrivetrainCommand_s evaluate(const VCRData_s &state, unsigned long curr_millis);

private:

    TorqueControllerSimpleParams_s _params;

};

#endif