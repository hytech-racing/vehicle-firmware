#include "Arduino.h"
#include "ADCInterface.h"
#include "VCF_Constants.h"

#include "Logger.h"

unsigned long const DELAY = 100;
unsigned long last = millis();

void setup()
{
    SPI.begin();    //for ADC
    Serial.begin(VCFTaskConstants::SERIAL_BAUDRATE);    //for serial monitor

    //create the ADC instance
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
}

void loop()
{
    if (millis() - DELAY > last) {
        ADCInterfaceInstance::instance().adc0_tick();
        Serial.print("\n===== ADC 0 =====\n");
        Serial.printf("2V5 Pedal Reference Raw:  %d\n", ADCInterfaceInstance::instance().pedal_reference().raw);
        Serial.printf("Steering 1 (CW) Raw:      %d\n", ADCInterfaceInstance::instance().get_steering_degrees_cw().raw);
        Serial.printf("Steering 2 (CCW) Raw:     %d\n", ADCInterfaceInstance::instance().get_steering_degrees_ccw().raw);
        Serial.printf("Acceleration 1 Raw:       %d\n", ADCInterfaceInstance::instance().acceleration_1().raw);
        Serial.printf("Acceleration 2 Raw:       %d\n", ADCInterfaceInstance::instance().acceleration_2().raw);
        Serial.printf("Brake 1 Raw:              %d\n", ADCInterfaceInstance::instance().brake_1().raw);
        Serial.printf("Brake 2 Raw:              %d\n", ADCInterfaceInstance::instance().brake_2().raw);

        ADCInterfaceInstance::instance().adc1_tick();
        Serial.printf("\n===== ADC 1 =====\n");
        Serial.printf("SHDN H Raw:                %d\n", ADCInterfaceInstance::instance().shdn_h().raw);
        Serial.printf("SHDN D Raw:                %d\n", ADCInterfaceInstance::instance().shdn_d().raw);
        Serial.printf("FL Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().FL_load_cell().raw);
        Serial.printf("FR Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().FR_load_cell().raw);
        Serial.printf("FR Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().FR_sus_pot().raw);
        Serial.printf("FL Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().FL_sus_pot().raw);
        Serial.printf("Front Brake Pressure Raw:  %d\n", ADCInterfaceInstance::instance().get_brake_pressure_front().raw);
        Serial.printf("Rear Brake Pressure Raw:   %d\n", ADCInterfaceInstance::instance().get_brake_pressure_rear().raw);

        last = millis();
    }
}