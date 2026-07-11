#ifdef ARDUINO
#include <Arduino.h>
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EEPROMUtilities.h"

/* Local includes */

#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include <stdio.h>

//accel params and brake params

const PedalsParams accel_params = {
    .min_pedal_1 = 1790,
    .min_pedal_2 = 1690,
    .max_pedal_1 = 2830,
    .max_pedal_2 = 670,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.05
};

const PedalsParams brake_params = {
    .min_pedal_1 = 1100,
    .min_pedal_2 = 2500, //needs to be calibrated again i think
    .max_pedal_1 = 2500,
    .max_pedal_2 = 1100,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.65
};

int count = 0;

void setup(){
    PedalsSystemInstance::create(accel_params, brake_params); //pass in the two different params
    ADCInterfaceInstance::create(
    ADCPinout_s {
        VCFInterfaceConstants::ADC0_CS,
        VCFInterfaceConstants::ADC1_CS
    },
    ADCChannels_s {
        VCFInterfaceConstants::PEDAL_REF_2V5_CHANNEL,
        VCFInterfaceConstants::STEERING_1_CHANNEL,
        VCFInterfaceConstants::STEERING_2_CHANNEL,
        VCFInterfaceConstants::ACCEL_1_CHANNEL,
        VCFInterfaceConstants::ACCEL_2_CHANNEL,
        VCFInterfaceConstants::BRAKE_1_CHANNEL,
        VCFInterfaceConstants::BRAKE_2_CHANNEL,

        VCFInterfaceConstants::SHDN_H_CHANNEL,
        VCFInterfaceConstants::SHDN_D_CHANNEL,
        VCFInterfaceConstants::FL_LOADCELL_CHANNEL,
        VCFInterfaceConstants::FR_LOADCELL_CHANNEL,
        VCFInterfaceConstants::FR_SUS_POT_CHANNEL,
        VCFInterfaceConstants::FL_SUS_POT_CHANNEL,
        VCFInterfaceConstants::BRAKE_PRESSURE_FRONT_CHANNEL,
        VCFInterfaceConstants::BRAKE_PRESSURE_REAR_CHANNEL
    },
    ADCScales_s { 
        VCFInterfaceConstants::PEDAL_REF_2V5_SCALE,
        VCFInterfaceConstants::STEERING_1_SCALE,
        VCFInterfaceConstants::STEERING_2_SCALE,
        VCFInterfaceConstants::ACCEL_1_SCALE,
        VCFInterfaceConstants::ACCEL_2_SCALE,
        VCFInterfaceConstants::BRAKE_1_SCALE,
        VCFInterfaceConstants::BRAKE_2_SCALE,

        VCFInterfaceConstants::SHDN_H_SCALE,
        VCFInterfaceConstants::SHDN_D_SCALE,
        VCFInterfaceConstants::FL_LOADCELL_SCALE,
        VCFInterfaceConstants::FR_LOADCELL_SCALE,
        VCFInterfaceConstants::FR_SUS_POT_SCALE,
        VCFInterfaceConstants::FL_SUS_POT_SCALE,
        VCFInterfaceConstants::BRAKE_PRESSURE_FRONT_SCALE,
        VCFInterfaceConstants::BRAKE_PRESSURE_REAR_SCALE
    }, 
    ADCOffsets_s {
        VCFInterfaceConstants::PEDAL_REF_2V5_OFFSET,
        VCFInterfaceConstants::STEERING_1_OFFSET,
        VCFInterfaceConstants::STEERING_2_OFFSET,
        VCFInterfaceConstants::ACCEL_1_OFFSET,
        VCFInterfaceConstants::ACCEL_2_OFFSET,
        VCFInterfaceConstants::BRAKE_1_OFFSET,
        VCFInterfaceConstants::BRAKE_2_OFFSET,

        VCFInterfaceConstants::SHDN_H_OFFSET,
        VCFInterfaceConstants::SHDN_D_OFFSET,
        VCFInterfaceConstants::FL_LOADCELL_OFFSET,
        VCFInterfaceConstants::FR_LOADCELL_OFFSET,
        VCFInterfaceConstants::FR_SUS_POT_OFFSET,
        VCFInterfaceConstants::FL_SUS_POT_OFFSET,
        VCFInterfaceConstants::BRAKE_PRESSURE_FRONT_OFFSET,
        VCFInterfaceConstants::BRAKE_PRESSURE_REAR_OFFSET
    });
    const int begin_time = 115200;
    Serial.begin(begin_time);
    SPI.begin();
    // init_adc_task();
    //observed values from 3/14 to init eeprom
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_1_MIN_ADDR, 2007); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_1_MAX_ADDR, 2569); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_2_MIN_ADDR, 1744); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_2_MAX_ADDR, 1170); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_2_MIN_ADDR, 2844); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_2_MAX_ADDR, 1379); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_1_MIN_ADDR, 899); //NOLINT manual pedals cal
    EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_1_MAX_ADDR, 2358); //NOLINT manual pedals cal
}

void loop(){
    // ADCInterfaceInstance::instance().adc0_tick();
    // PedalSensorData_s pedal_sensor_data = {};
    
    // pedal_sensor_data.accel_1 = ADCInterfaceInstance::instance().acceleration_1().raw;
    // pedal_sensor_data.accel_2 = ADCInterfaceInstance::instance().acceleration_2().raw;
    // pedal_sensor_data.brake_1 = ADCInterfaceInstance::instance().brake_1().raw;
    // pedal_sensor_data.brake_2 = ADCInterfaceInstance::instance().brake_2().raw;
    
    // // PedalsSystemInstance::instance().evaluate_pedals(pedal_sensor_data, millis());
    // // PedalsSystemInstance::instance().update_observed_pedal_limits(PedalsSystemInstance::instance().get_pedals_sensor_data());

    // Serial.print("Accel 1: ");
    // Serial.println(pedal_sensor_data.accel_1);
    // Serial.print("Accel 2: ");
    // Serial.println(pedal_sensor_data.accel_2);
    // Serial.print("Brake 1: ");
    // Serial.println(pedal_sensor_data.brake_1);
    // Serial.print("Brake 2: ");
    // Serial.println(pedal_sensor_data.brake_2);
    // Serial.print("Accel Implausible: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_implausible);
    // Serial.print("Brake Implausible: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_implausible);
    // Serial.print("Brake Pressed: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().brake_is_pressed);
    // Serial.print("Accel Pressed: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().accel_is_pressed);
    // Serial.print("Mech Brake Active: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().mech_brake_is_active);
    // Serial.print("Brake and Accel Implausibility High: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().brake_and_accel_pressed_implausibility_high);
    // Serial.print("Implausibility has exceeded max duration: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().implausibility_has_exceeded_max_duration);
    // Serial.print("Accel Percent: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().accel_percent);
    // Serial.print("Brake Percent: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().brake_percent);
    // Serial.print("Regen Percent: ");
    // Serial.println(PedalsSystemInstance::instance().get_pedals_system_data().regen_percent);    

    // if (count >= 100) {
    //     Serial.println("\nRecalibrating Pedals...\n");
    //     PedalsSystemInstance::instance().recalibrate_min_max(PedalsSystemInstance::instance().get_pedals_sensor_data());
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_1_MIN_ADDR, 1958);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_1_MAX_ADDR, 2505);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_2_MIN_ADDR, 1703);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::ACCEL_2_MAX_ADDR, 1143);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_1_MIN_ADDR, 2829);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_1_MAX_ADDR, 1950);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_2_MIN_ADDR, 837);
        // EEPROMUtilities::write_eeprom_32bit(VCFInterfaceConstants::BRAKE_2_MAX_ADDR, 1600);
    //     count = 0;
    // } else {
    //     count++;
    // }
    
    const int delay_time = 100;
    delay(delay_time);
}