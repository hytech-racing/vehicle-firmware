#ifndef INVERTER_INTERFACE_HPP
#define INVERTER_INTERFACE_HPP

/* Standard Library */
#include <stdint.h>

/* External Includes */
#include "FlexCAN_T4.h"
#include "shared_types.hpp"

#include <CANInterface.h>
#include <ht_can.h>

/* Local System Includes */
#include "DrivetrainSystem.hpp"


struct InverterParams_s
{
    float minimum_hv_voltage;
};

/**
 * Struct containing id info for this specific inverter interface
 */
struct InverterCANIds_s
{
    uint32_t inv_control_word_id;
    uint32_t inv_control_input_id;
    uint32_t inv_control_parameter_id;

    uint32_t inv_temps_id;
    uint32_t inv_status_id;
    uint32_t inv_dynamics_id;
};

/**
 * Inverter interface
 */
class InverterInterface
{
  public:
    InverterInterface(uint32_t inv_control_word_id, uint32_t inv_control_input_id, uint32_t inv_control_params_id, InverterParams_s inverter_params)
        : _inverter_params(inverter_params)
    {
        _inverter_ids.inv_control_word_id = inv_control_word_id;
        _inverter_ids.inv_control_parameter_id = inv_control_params_id;
        _inverter_ids.inv_control_input_id = inv_control_input_id;
    }

    /* receiving callbacks */
    void receiveInvStatus(const CAN_message_t& can_msg, unsigned long curr_millis);

    void receiveInvTemps(const CAN_message_t& can_msg, unsigned long curr_millis);

    void receiveInvDynamics(const CAN_message_t& can_msg, unsigned long curr_millis);

    void receiveInvPower(const CAN_message_t& can_msg, unsigned long curr_millis);

    void receiveInvFeedback(const CAN_message_t& can_msg, unsigned long curr_millis);

    /* Sending */
    void sendInvSetpointCommand();

    void sendInvControlWord();

    void sendInvControlParams();

    /* Inverter Functs */
    void setSpeed(float desired_rpm, float torque_limit_nm);

    void setIdle();

    void setInverterControlWord(InverterControlWord_s control_word);

    /* Getters */
    InverterStatus_s getStatus() const;

    InverterTemps_s getTemps() const;

    InverterPower_s getPower() const;

    MotorMechanics_s getMotorMechanics() const;

    InverterControlFeedback_s getControlParams() const;

    InverterFeedbackData_s getAllInverterData() const;

  private:
    InverterCANIds_s _inverter_ids;
    InverterControlInput_s _inverter_control_inputs;
    InverterControlWord_s _inverter_control_word;
    InverterControlParams_s _inverter_control_params;
    InverterFeedbackData_s _feedback_data;
    InverterParams_s _inverter_params;
};


#endif // __INVERTERINTERFACE_H__
