#include <math.h>
#include "PedalsSystem.hpp"


void PedalsSystem::evaluatePedals(PedalSensorData_s pedals_data, unsigned long curr_millis)
{
    int accel_1 = static_cast<int>(pedals_data.accel_1);
    int accel_2 = static_cast<int>(pedals_data.accel_2);
    int brake_1 = static_cast<int>(pedals_data.brake_1);
    int brake_2 = static_cast<int>(pedals_data.brake_2);

    float accel1_scaled = _pedalsScaler(accel_1, static_cast<int>(_accel_params.max_pedal_1), static_cast<int>(_accel_params.min_pedal_1));
    float accel2_scaled = _pedalsScaler(accel_2, static_cast<int>(_accel_params.max_pedal_2), static_cast<int>(_accel_params.min_pedal_2));
    float brake1_scaled = _pedalsScaler(brake_1, static_cast<int>(_brake_params.max_pedal_1), static_cast<int>(_brake_params.min_pedal_1));
    float brake2_scaled = _pedalsScaler(brake_2, static_cast<int>(_brake_params.max_pedal_2), static_cast<int>(_brake_params.min_pedal_2));


    // std::c_systemData << "accel1_scaled " << accel1_scaled << std::endl;
    // std::c_systemData << "accel2_scaled " << accel2_scaled << std::endl;
    // std::c_systemData << "brake1_scaled " << brake1_scaled << std::endl;
    // std::c_systemData << "brake2_scaled " << brake2_scaled << std::endl;

    // FSAE Rules T.4.2.4
    _system_data.brake_is_implausible = _evaluatePedalImplausibilities(brake1_scaled,
                                                                        brake2_scaled,
                                                                        brake_1,
                                                                        brake_2,
                                                                        _brake_params,
                                                                        IMPLAUSIBILITY_PERCENT
    );
    _system_data.accel_is_implausible = _evaluatePedalImplausibilities(accel1_scaled,
                                                                        accel2_scaled,
                                                                        accel_1,
                                                                        accel_2,
                                                                        _accel_params,
                                                                        IMPLAUSIBILITY_PERCENT
    );

    float accel_percent = _pedalPercentage(accel1_scaled, accel2_scaled, _accel_params);
    _system_data.accel_percent = std::max(accel_percent, 0.0f);
    float brake_percent = _pedalPercentage(brake1_scaled, brake2_scaled, _brake_params);
    _system_data.brake_percent = std::max(brake_percent, 0.0f);

    bool accel_pressed = accel_percent > _accel_params.activation_percentage;
    bool brake_pressed = brake_percent > _brake_params.activation_percentage;
    bool mech_brake_pressed = brake_percent >= _brake_params.mechanical_activation_percentage;

    _system_data.accel_is_pressed = accel_pressed;
    _system_data.brake_is_pressed = brake_pressed;
    _system_data.mech_brake_is_active = mech_brake_pressed;
    _system_data.brake_and_accel_pressed_implausibility_high = accel_pressed && _system_data.brake_is_pressed;

    bool accel_pedal_oor = (_evaluatePedalOOR(accel_1,
                                            static_cast<int>(_accel_params.min_sensor_pedal_1),
                                            static_cast<int>(_accel_params.max_sensor_pedal_1)
                            ) ||
                            _evaluatePedalOOR(accel_2,
                                            static_cast<int>(_accel_params.min_sensor_pedal_2),
                                            static_cast<int>(_accel_params.max_sensor_pedal_2)
                            )
    );
    bool brake_pedal_oor = (_evaluatePedalOOR(brake_1,
                                            static_cast<int>(_brake_params.min_sensor_pedal_1),
                                            static_cast<int>(_brake_params.max_sensor_pedal_1)
                            ) ||
                            _evaluatePedalOOR(brake_2,
                                            static_cast<int>(_brake_params.min_sensor_pedal_2),
                                            static_cast<int>(_brake_params.max_sensor_pedal_2)
                            )
    );
    bool implausibility = (_system_data.accel_is_implausible ||
                        _system_data.brake_and_accel_pressed_implausibility_high ||
                        _system_data.brake_is_implausible ||
                        brake_pedal_oor ||
                        accel_pedal_oor
    );

    // std::c_systemData << "implaus " << implausibility <<std::endl;
    // std::c_systemData << "accel_pedal_oor " << accel_pedal_oor <<std::endl;
    // std::c_systemData << "brake_pedal_oor " << brake_pedal_oor <<std::endl;
    // std::c_systemData << "accel_1 " << accel_1<<std::endl;
    // std::c_systemData << "accel_2 " << accel_2<<std::endl;
    // std::c_systemData << "brake_1 " << brake_1<<std::endl;
    // std::c_systemData << "brake_2 " << brake_2<<std::endl;
    if (implausibility && (_implausibility_start_time == 0))
    {
        _implausibility_start_time = curr_millis;
    }
    else if ((!implausibility) && ((_system_data.accel_percent <= ACCELERATION_PERCENT_LIMIT)))
    {
        _implausibility_start_time = 0;
    }

    if (implausibility)
    {
        _implausibility_occured = true;
    }
    else if (_implausibility_occured && _system_data.accel_percent <= ACCELERATION_PERCENT_LIMIT)
    {
        _implausibility_occured = false;
    }

    _system_data.mech_brake_is_active = _system_data.brake_percent >= _brake_params.mechanical_activation_percentage;
    _system_data.implausibility_has_exceeded_max_duration = _maxDurationOfImplausibilityExceeded(curr_millis);

    // std::c_systemData << "implaus "<< _implaus_occured <<std::endl;
    _system_data.accel_percent = (_implausibility_occured) ? 0 : _system_data.accel_percent;
    // we dont care if brake is implaus, as long as it isnt oor (likely errored)
    _system_data.brake_percent = (brake_pedal_oor) ? 0 : _system_data.brake_percent;

    return;
}

void PedalsSystem::recalibrateMinMax(const PedalSensorData_s &curr_values)
{

    // If pedal is near 0% travel and is closer to the observed max, then this sensor is a negative coefficient.
    bool accel_1_flipped = std::abs((int) curr_values.accel_1 - (int) max_observed_accel_1) < std::abs((int) curr_values.accel_1 - (int) min_observed_accel_1);
    bool accel_2_flipped = std::abs((int) curr_values.accel_2 - (int) max_observed_accel_2) < std::abs((int) curr_values.accel_2 - (int) min_observed_accel_2);
    bool brake_1_flipped = std::abs((int) curr_values.brake_1 - (int) max_observed_brake_1) < std::abs((int) curr_values.brake_1 - (int) min_observed_brake_1);
    bool brake_2_flipped = std::abs((int) curr_values.brake_2 - (int) max_observed_brake_2) < std::abs((int) curr_values.brake_2 - (int) min_observed_brake_2);

    _accel_params.min_pedal_1 = accel_1_flipped ? max_observed_accel_1 : min_observed_accel_1;
    _accel_params.max_pedal_1 = accel_1_flipped ? min_observed_accel_1 : max_observed_accel_1;
    _accel_params.min_pedal_2 = accel_2_flipped ? max_observed_accel_2 : min_observed_accel_2;
    _accel_params.max_pedal_2 = accel_2_flipped ? min_observed_accel_2 : max_observed_accel_2;
    _brake_params.min_pedal_1 = brake_1_flipped ? max_observed_brake_1 : min_observed_brake_1;
    _brake_params.max_pedal_1 = brake_1_flipped ? min_observed_brake_1 : max_observed_brake_1;
    _brake_params.min_pedal_2 = brake_2_flipped ? max_observed_brake_2 : min_observed_brake_2;
    _brake_params.max_pedal_2 = brake_2_flipped ? min_observed_brake_2 : max_observed_brake_2;
}

void PedalsSystem::updateObservedPedalLimits(const PedalSensorData_s &curr_values)
{
    min_observed_accel_1 = std::min(min_observed_accel_1, curr_values.accel_1);
    max_observed_accel_1 = std::max(max_observed_accel_1, curr_values.accel_1);
    min_observed_accel_2 = std::min(min_observed_accel_2, curr_values.accel_2);
    max_observed_accel_2 = std::max(max_observed_accel_2, curr_values.accel_2);
    min_observed_brake_1 = std::min(min_observed_brake_1, curr_values.brake_1);
    max_observed_brake_1 = std::max(max_observed_brake_1, curr_values.brake_1);
    min_observed_brake_2 = std::min(min_observed_brake_2, curr_values.brake_2);
    max_observed_brake_2 = std::max(max_observed_brake_2, curr_values.brake_2);
}

float PedalsSystem::_pedalPercentage(float scaled_pedal_1, float scaled_pedal_2, const PedalsParams& params)
{
    const float divider = 2.0;
    float percent = (static_cast<float>(scaled_pedal_1) + static_cast<float>(scaled_pedal_2)) / divider;
    return _removeDeadzone(percent, params.deadzone_margin);
}

float PedalsSystem::_pedalsScaler(int pedal_val, int max_pedal, int min_pedal)
{
    if (max_pedal > min_pedal)
    {
        return ::fabs(static_cast<float>(pedal_val - min_pedal))/::fabs(static_cast<float>(max_pedal - min_pedal));
    }

    return ::fabs(static_cast<float>(min_pedal - pedal_val))/::fabs(static_cast<float>(max_pedal - min_pedal));
}


bool PedalsSystem::_maxDurationOfImplausibilityExceeded(unsigned long curr_millis)
{

    if (_implausibility_start_time != 0)
    {
        return ((curr_millis - _implausibility_start_time) > IMPLAUSIBILITY_DURATION);
    }
    else
    {
        return false;
    }
}

bool PedalsSystem::_evaluatePedalImplausibilities(float pedal_1_scaled,float pedal_2_scaled, int pedal_data1_analog, int pedal_data2_analog, const PedalsParams &params, float max_percent_diff)
{
    bool pedal1_min_max_implaus = _evaluateMinMaxPedalImplausibilities(pedal_data1_analog,
                                                                    static_cast<int>(params.min_pedal_1),
                                                                    static_cast<int>(params.max_pedal_1),
                                                                    params.implausibility_margin
    );
    bool pedal2_min_max_implaus = _evaluateMinMaxPedalImplausibilities(pedal_data2_analog,
                                                                    static_cast<int>(params.min_pedal_2),
                                                                    static_cast<int>(params.max_pedal_2),
                                                                    params.implausibility_margin
    );

    bool sens_not_within_req_percent = ((::fabs(pedal_1_scaled - pedal_2_scaled)) > max_percent_diff); // DIVIDE BY 100
    return pedal1_min_max_implaus || pedal2_min_max_implaus || sens_not_within_req_percent;
}

bool PedalsSystem::_evaluatePedalOOR(int pedal_data, int min, int max)
{
    return (pedal_data <= min || pedal_data >= max);
}

bool PedalsSystem::_evaluateMinMaxPedalImplausibilities(int pedal_data, int min, int max, float implaus_margin_scale)
{
    bool pedal_swapped = false;
    float pedal_margin = static_cast<float>(::abs(max-min)) * implaus_margin_scale;
    if (min > max)
    {
        pedal_swapped = true;
    }

    // FSAE EV.5.5
    // FSAE T.4.2.10
    float float_pedal_data = static_cast<float>(pedal_data);
    float min_float = static_cast<float>(min);
    float max_float = static_cast<float>(max);
    bool pedal_less_than_min = pedal_swapped ? (float_pedal_data > (min_float+pedal_margin)) : (float_pedal_data < (min_float-pedal_margin));
    bool pedal_greater_than_max = pedal_swapped ? (float_pedal_data < (max_float-pedal_margin)) : (float_pedal_data > (max_float+pedal_margin));
    return pedal_less_than_min || pedal_greater_than_max;
}

float PedalsSystem::_removeDeadzone(float conversion_input, float deadzone)
{
    // Your conversion input is basically pedal data over abs(max-min) to get it betwen 0-1. Then deadzone is removed from it.
    const float onner = 1.0;
    float range = onner - (deadzone * 2);
    // e.g. vals from 0 to 1, deadzone is .05, range is .9
    // subtract deadzone to be -.05 to .95 & clamp at 0
    float _system_data = std::max(conversion_input - deadzone, 0.0f); // max(1010 - 0.03, 0)
    // values now are 0 to .95
    // divide by range of values to scale up (.9)
    _system_data /= range; // 1074.43617
    // values are now 0 to 1.0555...
    // clamp at 0 to 1
    _system_data = std::min(_system_data, 1.0f); // min(_systemData, 1)

    return _system_data;
}