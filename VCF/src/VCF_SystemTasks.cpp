#include "VCF_SystemTasks.hpp"


void initialize_all_systems()
{
    /* Neopixel Controller */
    NeopixelControllerInstance::create(VCFSystems::NEOPIXEL_COUNT, VCFSystems::NEOPIXEL_CONTROL_PIN);
    NeopixelControllerInstance::instance().init();

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
    SteeringSystemParams_s steering_params = {
        .analog = {
            .deg_per_count = VCFSystems::DEG_PER_COUNT_ANALOG,
            .oor_tolerance_fraction = VCFSystems::ANALOG_TOLERANCE,
            .calibration_snapshot = {
                .min_raw = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_ANALOG_ADDR),
                .max_raw = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_ANALOG_ADDR),
            },
            .oor_bounds = {
                .min_valid_raw = static_cast<int32_t>(EEPROMUtilities::read_eeprom_32bit(VCFSystems::ANALOG_MIN_WITH_MARGINS_ADDR)),
                .max_valid_raw = static_cast<int32_t>(EEPROMUtilities::read_eeprom_32bit(VCFSystems::ANALOG_MAX_WITH_MARGINS_ADDR)),
            },
        },
        .digital = {
            .deg_per_count = VCFSystems::DEG_PER_COUNT_DIGITAL,
            .oor_tolerance_fraction = VCFSystems::DIGITAL_TOLERANCE,
            .calibration_snapshot = {
                .min_raw = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_DIGITAL_ADDR),
                .max_raw = EEPROMUtilities::read_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_DIGITAL_ADDR),
            },
            .oor_bounds = {
                .min_valid_raw = static_cast<int32_t>(EEPROMUtilities::read_eeprom_32bit(VCFSystems::DIGITAL_MIN_WITH_MARGINS_ADDR)),
                .max_valid_raw = static_cast<int32_t>(EEPROMUtilities::read_eeprom_32bit(VCFSystems::DIGITAL_MAX_WITH_MARGINS_ADDR)),
            },
        },
        .max_steering_rate_deg_per_s = VCFSystems::MAX_DTHETA_THRESHOLD,
        .max_sensor_disagreement_deg = VCFSystems::ERROR_BETWEEN_SENSORS_TOLERANCE,
    };

    // Derived calibration values (not stored in EEPROM)
    SteeringSensorCalibration_s &analog_calibration = steering_params.analog.calibration_snapshot;
    analog_calibration.span_raw   = analog_calibration.max_raw - analog_calibration.min_raw;
    analog_calibration.center_raw = analog_calibration.min_raw + (analog_calibration.span_raw / 2);
    steering_params.analog.oor_tolerance_raw = static_cast<float>(analog_calibration.span_raw) * steering_params.analog.oor_tolerance_fraction;

    SteeringSensorCalibration_s &digital_calibration = steering_params.digital.calibration_snapshot;
    digital_calibration.span_raw   = digital_calibration.max_raw - digital_calibration.min_raw;
    digital_calibration.center_raw = digital_calibration.min_raw + (digital_calibration.span_raw / 2);
    steering_params.digital.oor_tolerance_raw = static_cast<float>(digital_calibration.span_raw) * steering_params.digital.oor_tolerance_fraction;

    SteeringSystemInstance::create(steering_params);
}

HT_TASK::TaskResponse update_pedals_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    // Observed pedal values (ONLY USED FOR RECALIBRATION)
    // WARNING: These are the true min/max observed values, NOT the "value at min travel" and "value at max travel"
    //          that are defined in the PedalsParam struct.
    PedalsSystemInstance::instance().updateObservedPedalLimits(PedalsSystemInstance::instance().getPedalsSensorData());

    if (VCRInterfaceInstance::instance().arePedalsCalibrating())
    {
        // PedalsSystemInstance::instance().recalibrate_min_max(VCFData_sInstance::instance().interface_data.pedal_sensor_data);
        PedalsSystemInstance::instance().recalibrateMinMax(PedalsSystemInstance::instance().getPedalsSensorData());
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_1_MIN_ADDR, PedalsSystemInstance::instance().getAccelParams().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_1_MAX_ADDR, PedalsSystemInstance::instance().getAccelParams().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_2_MIN_ADDR, PedalsSystemInstance::instance().getAccelParams().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ACCEL_2_MAX_ADDR, PedalsSystemInstance::instance().getAccelParams().max_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_1_MIN_ADDR, PedalsSystemInstance::instance().getBrakeParams().min_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_1_MAX_ADDR, PedalsSystemInstance::instance().getBrakeParams().max_pedal_1);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_2_MIN_ADDR, PedalsSystemInstance::instance().getBrakeParams().min_pedal_2);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::BRAKE_2_MAX_ADDR, PedalsSystemInstance::instance().getBrakeParams().max_pedal_2);
    }

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_pedals_data(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info)
{
    PEDALS_SYSTEM_DATA_t pedals_data = {};

    pedals_data.accel_implausible = PedalsSystemInstance::instance().getPedalsSystemData().accel_is_implausible;
    pedals_data.brake_implausible = PedalsSystemInstance::instance().getPedalsSystemData().brake_is_implausible;
    pedals_data.brake_accel_implausibility = PedalsSystemInstance::instance().getPedalsSystemData().brake_and_accel_pressed_implausibility_high;

    pedals_data.accel_pedal_active = PedalsSystemInstance::instance().getPedalsSystemData().accel_is_pressed;
    pedals_data.brake_pedal_active = PedalsSystemInstance::instance().getPedalsSystemData().brake_is_pressed;
    pedals_data.mechanical_brake_active = PedalsSystemInstance::instance().getPedalsSystemData().mech_brake_is_active;
    pedals_data.implaus_exceeded_max_duration = PedalsSystemInstance::instance().getPedalsSystemData().implausibility_has_exceeded_max_duration;


    pedals_data.accel_pedal_ro = HYTECH_accel_pedal_ro_toS(PedalsSystemInstance::instance().getPedalsSystemData().accel_percent);
    pedals_data.brake_pedal_ro = HYTECH_brake_pedal_ro_toS(PedalsSystemInstance::instance().getPedalsSystemData().brake_percent);

    CAN_util::enqueue_msg(&pedals_data, &Pack_PEDALS_SYSTEM_DATA_hytech, VCFCANInterfaceInstance::instance().telem_can_tx_buffer);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_steering_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    const uint32_t analog_raw = SteeringSystemInstance::instance().getSteeringSystemData().analog_raw; // NOLINT thinks this is not initialized
    const uint32_t digital_raw = SteeringSystemInstance::instance().getSteeringSystemData().digital_raw; // NOLINT thinks this is not initialized

    SteeringSystemInstance::instance().updateObservedExtremes(analog_raw, digital_raw);


    if (VCRInterfaceInstance::instance().isSteeringCalibrating())
    {
        SteeringSystem &steering_system = SteeringSystemInstance::instance();
        steering_system.recalibrateSteering();

        const SteeringSystemParams_s &steering_params = steering_system.getSteeringParams();

        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_ANALOG_ADDR, steering_params.analog.calibration_snapshot.min_raw);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_ANALOG_ADDR, steering_params.analog.calibration_snapshot.max_raw);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MIN_STEERING_SIGNAL_DIGITAL_ADDR, steering_params.digital.calibration_snapshot.min_raw);
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::MAX_STEERING_SIGNAL_DIGITAL_ADDR, steering_params.digital.calibration_snapshot.max_raw);

        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ANALOG_MIN_WITH_MARGINS_ADDR, static_cast<uint32_t>(steering_params.analog.oor_bounds.min_valid_raw));
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::ANALOG_MAX_WITH_MARGINS_ADDR, static_cast<uint32_t>(steering_params.analog.oor_bounds.max_valid_raw));
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::DIGITAL_MIN_WITH_MARGINS_ADDR, static_cast<uint32_t>(steering_params.digital.oor_bounds.min_valid_raw));
        EEPROMUtilities::write_eeprom_32bit(VCFSystems::DIGITAL_MAX_WITH_MARGINS_ADDR, static_cast<uint32_t>(steering_params.digital.oor_bounds.max_valid_raw));
    }

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_steering_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    STEERING_DATA_t msg_out;
    SteeringSystemData_s steering_system_data = SteeringSystemInstance::instance().getSteeringSystemData();

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
    NeopixelControllerInstance::instance().refreshNeopixels(PedalsSystemInstance::instance().getPedalsSystemData(), CANInterfacesInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}