#include "VCF_SystemTasks.h"


void initialize_all_systems()
{

    /* Neopixel Controller */
    NeopixelControllerInstance::create(VCFSystems::NEOPIXEL_COUNT, VCFSystems::NEOPIXEL_CONTROL_PIN);
    NeopixelControllerInstance::instance().init_neopixels();

    /* Pedals System */
    PedalsParams accel_params = {
        .min_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::ACCEL_1_MIN_ADDR),
        .min_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::ACCEL_2_MIN_ADDR),
        .max_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::ACCEL_1_MAX_ADDR),
        .max_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::ACCEL_2_MAX_ADDR),
        .activation_percentage = VCFSystems::ACCEL_ACTIVATION_PERCENTAGE,
        .min_sensor_pedal_1 = VCFSystems::ACCEL_MIN_SENSOR_PEDAL_1,
        .min_sensor_pedal_2 = VCFSystems::ACCEL_MIN_SENSOR_PEDAL_2,
        .max_sensor_pedal_1 = VCFSystems::ACCEL_MAX_SENSOR_PEDAL_1,
        .max_sensor_pedal_2 = VCFSystems::ACCEL_MAX_SENSOR_PEDAL_2,
        .deadzone_margin = VCFSystems::ACCEL_DEADZONE_MARGIN,
        .implausibility_margin = IMPLAUSIBILITY_PERCENT,
        .mechanical_activation_percentage = VCFSystems::ACCEL_MECHANICAL_ACTIVATION_PERCENTAGE
    };

    PedalsParams brake_params = {
        .min_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::BRAKE_1_MIN_ADDR),
        .min_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::BRAKE_2_MIN_ADDR),
        .max_pedal_1 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::BRAKE_1_MAX_ADDR),
        .max_pedal_2 = EEPROMUtilities::read_eeprom_32bit(VCFSystems::BRAKE_2_MAX_ADDR),
        .activation_percentage = VCFSystems::BRAKE_ACTIVATION_PERCENTAGE,
        .min_sensor_pedal_1 = VCFSystems::BRAKE_MIN_SENSOR_PEDAL_1,
        .min_sensor_pedal_2 = VCFSystems::BRAKE_MIN_SENSOR_PEDAL_2,
        .max_sensor_pedal_1 = VCFSystems::BRAKE_MAX_SENSOR_PEDAL_1,
        .max_sensor_pedal_2 = VCFSystems::BRAKE_MAX_SENSOR_PEDAL_2,
        .deadzone_margin = VCFSystems::BRAKE_DEADZONE_MARGIN,
        .implausibility_margin = IMPLAUSIBILITY_PERCENT,
        .mechanical_activation_percentage = VCFSystems::BRAKE_MECHANICAL_ACTIVATION_PERCENTAGE
    };
    PedalsSystemInstance::create(accel_params, brake_params); // pass in the two different params

    /* Steering System */
    SteeringParams_s steering_params = {
        .min_steering_signal_analog = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_ANALOG_ADDR),
        .max_steering_signal_analog = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_ANALOG_ADDR),
        .min_steering_signal_digital = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_DIGITAL_ADDR),
        .max_steering_signal_digital = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_DIGITAL_ADDR),
        .analog_min_with_margins = EEPROMUtilities::read_eeprom_32bit(VCFSystems::ANALOG_MIN_WITH_MARGINS_ADDR), // NOLINT this is prev saved value so it is ok
        .analog_max_with_margins = EEPROMUtilities::read_eeprom_32bit(VCFSystems::ANALOG_MAX_WITH_MARGINS_ADDR), // NOLINT this is prev saved value so it is ok
        .digital_min_with_margins = EEPROMUtilities::read_eeprom_32bit(VCFSystems::DIGITAL_MIN_WITH_MARGINS_ADDR), // NOLINT this is prev saved value so it is ok
        .digital_max_with_margins = EEPROMUtilities::read_eeprom_32bit(VCFSystems::DIGITAL_MAX_WITH_MARGINS_ADDR), // NOLINT this is prev saved value so it is ok
        .deg_per_count_analog = VCFSystems::DEG_PER_COUNT_ANALOG,
        .deg_per_count_digital = VCFSystems::DEG_PER_COUNT_DIGITAL,
        .analog_tolerance = VCFSystems::ANALOG_TOLERANCE,
        .digital_tolerance = VCFSystems::DIGITAL_TOLERANCE,
        .max_dtheta_threshold = VCFSystems::MAX_DTHETA_THRESHOLD,
        .error_between_sensors_tolerance = VCFSystems::ERROR_BETWEEN_SENSORS_TOLERANCE
    };
    steering_params.span_signal_analog = steering_params.max_steering_signal_analog - steering_params.min_steering_signal_analog;
    steering_params.analog_midpoint = (steering_params.max_steering_signal_analog + steering_params.min_steering_signal_analog) / 2;
    steering_params.span_signal_digital = steering_params.max_steering_signal_digital - steering_params.min_steering_signal_digital;
    steering_params.digital_midpoint = (steering_params.min_steering_signal_digital + steering_params.max_steering_signal_digital) / 2;
    SteeringSystemInstance::create(steering_params); // NOLINT thinks steering params is not initialized


}

HT_TASK::TaskResponse update_pedals_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    // Observed pedal values (ONLY USED FOR RECALIBRATION)
    // WARNING: These are the true min/max observed values, NOT the "value at min travel" and "value at max travel"
    //          that are defined in the PedalsParam struct.
    PedalsSystemInstance::instance().update_observed_pedal_limits(PedalsSystemInstance::instance().get_pedals_sensor_data());

    if (VCRInterfaceInstance::instance().is_in_pedals_calibration_state())
    {
        // PedalsSystemInstance::instance().recalibrate_min_max(VCFData_sInstance::instance().interface_data.pedal_sensor_data);
        PedalsSystemInstance::instance().recalibrate_min_max(PedalsSystemInstance::instance().get_pedals_sensor_data());
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_1_MIN_ADDR, PedalsSystemInstance::instance().get_accel_params().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_1_MAX_ADDR, PedalsSystemInstance::instance().get_accel_params().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_2_MIN_ADDR, PedalsSystemInstance::instance().get_accel_params().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_2_MAX_ADDR, PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_1_MIN_ADDR, PedalsSystemInstance::instance().get_brake_params().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_1_MAX_ADDR, PedalsSystemInstance::instance().get_brake_params().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_2_MIN_ADDR, PedalsSystemInstance::instance().get_brake_params().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_2_MAX_ADDR, PedalsSystemInstance::instance().get_brake_params().max_pedal_2);
    }

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_pedals_data(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info)
{
    PEDALS_SYSTEM_DATA_t pedals_data = {};

    pedals_data.accel_implausible = PedalsSystemInstance::instance().get_pedals_system_data().accel_is_implausible;
    pedals_data.brake_implausible = PedalsSystemInstance::instance().get_pedals_system_data().brake_is_implausible;
    pedals_data.brake_accel_implausibility = PedalsSystemInstance::instance().get_pedals_system_data().brake_and_accel_pressed_implausibility_high;

    pedals_data.accel_pedal_active = PedalsSystemInstance::instance().get_pedals_system_data().accel_is_pressed;
    pedals_data.brake_pedal_active = PedalsSystemInstance::instance().get_pedals_system_data().brake_is_pressed;
    pedals_data.mechanical_brake_active = PedalsSystemInstance::instance().get_pedals_system_data().mech_brake_is_active;
    pedals_data.implaus_exceeded_max_duration = PedalsSystemInstance::instance().get_pedals_system_data().implausibility_has_exceeded_max_duration;


    pedals_data.accel_pedal_ro = HYTECH_accel_pedal_ro_toS(PedalsSystemInstance::instance().get_pedals_system_data().accel_percent);
    pedals_data.brake_pedal_ro = HYTECH_brake_pedal_ro_toS(PedalsSystemInstance::instance().get_pedals_system_data().brake_percent);

    CAN_util::enqueue_msg(&pedals_data, &Pack_PEDALS_SYSTEM_DATA_hytech, VCFCANInterfaceInstance::instance().telem_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_steering_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    const uint32_t analog_raw = SteeringSystemInstance::instance().get_steering_system_data().analog_raw; // NOLINT thinks this is not initialized
    const uint32_t digital_raw = SteeringSystemInstance::instance().get_steering_system_data().digital_raw; // NOLINT thinks this is not initialized

    SteeringSystemInstance::instance().update_observed_steering_limits(analog_raw, digital_raw);


     if (VCRInterfaceInstance::instance().is_in_steering_calibration_state()) {

        SteeringSystemInstance::instance().recalibrate_steering_digital();
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_ANALOG_ADDR, SteeringSystemInstance::instance().get_steering_params().min_steering_signal_analog);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_ANALOG_ADDR, SteeringSystemInstance::instance().get_steering_params().max_steering_signal_analog);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_DIGITAL_ADDR, SteeringSystemInstance::instance().get_steering_params().min_steering_signal_digital);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_DIGITAL_ADDR, SteeringSystemInstance::instance().get_steering_params().max_steering_signal_digital);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ANALOG_MIN_WITH_MARGINS_ADDR, SteeringSystemInstance::instance().get_steering_params().analog_min_with_margins);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ANALOG_MAX_WITH_MARGINS_ADDR, SteeringSystemInstance::instance().get_steering_params().analog_max_with_margins);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::DIGITAL_MIN_WITH_MARGINS_ADDR, SteeringSystemInstance::instance().get_steering_params().digital_min_with_margins);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::DIGITAL_MAX_WITH_MARGINS_ADDR, SteeringSystemInstance::instance().get_steering_params().digital_max_with_margins);
    }

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_steering_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    STEERING_DATA_t msg_out;
    SteeringSystemData_s steering_system_data = SteeringSystemInstance::instance().get_steering_system_data();

    msg_out.steering_analog_oor = steering_system_data.analog_oor_implausibility;
    msg_out.steering_both_sensors_fail = steering_system_data.both_sensors_fail;
    msg_out.steering_digital_oor = steering_system_data.digital_oor_implausibility;
    msg_out.steering_dtheta_exceeded_analog = steering_system_data.dtheta_exceeded_analog;
    msg_out.steering_dtheta_exceeded_digital = steering_system_data.dtheta_exceeded_digital;
    msg_out.steering_interface_sensor_error = steering_system_data.interface_sensor_error;
    msg_out.steering_output_steering_angle_ro = HYTECH_steering_output_steering_angle_ro_toS(steering_system_data.output_steering_angle);
    msg_out.steering_sensor_disagreement = steering_system_data.sensor_disagreement_implausibility;
    msg_out.steering_analog_raw = steering_system_data.analog_raw;
    msg_out.steering_digital_raw = steering_system_data.digital_raw;

    CAN_util::enqueue_msg(&msg_out, &Pack_STEERING_DATA_hytech, VCFCANInterfaceInstance::instance().telem_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    NeopixelControllerInstance::instance().refresh_neopixels(PedalsSystemInstance::instance().get_pedals_system_data(), CANInterfacesInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

