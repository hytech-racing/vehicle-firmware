#include "ADCInterface.h"
#include "VCF_Constants.h"

unsigned long const DELAY = 100;
unsigned long last = millis();

void setup()
{
    SPI.begin();    //for ADC
    Serial.begin(VCFInterfaces::SERIAL_BAUDRATE);    //for serial monitor

    //create the ADC instance
    ADCInterfaceInstance::create(
        ADCPinout_s
        {
            VCFInterfaces::ADC0_CS,
            VCFInterfaces::ADC1_CS
        },
        ADCChannels_s
        {
            VCFInterfaces::PEDAL_REF_2V5_CHANNEL,
            VCFInterfaces::STEERING_1_CHANNEL,
            VCFInterfaces::STEERING_2_CHANNEL,
            VCFInterfaces::ACCEL_1_CHANNEL,
            VCFInterfaces::ACCEL_2_CHANNEL,
            VCFInterfaces::BRAKE_1_CHANNEL,
            VCFInterfaces::BRAKE_2_CHANNEL,

            VCFInterfaces::SHDN_H_CHANNEL,
            VCFInterfaces::SHDN_D_CHANNEL,
            VCFInterfaces::FL_LOADCELL_CHANNEL,
            VCFInterfaces::FR_LOADCELL_CHANNEL,
            VCFInterfaces::FR_SUS_POT_CHANNEL,
            VCFInterfaces::FL_SUS_POT_CHANNEL,
            VCFInterfaces::BRAKE_PRESSURE_FRONT_CHANNEL,
            VCFInterfaces::BRAKE_PRESSURE_REAR_CHANNEL
        },
        ADCScales_s
        {
            VCFInterfaces::PEDAL_REF_2V5_SCALE,
            VCFInterfaces::STEERING_1_SCALE,
            VCFInterfaces::STEERING_2_SCALE,
            VCFInterfaces::ACCEL_1_SCALE,
            VCFInterfaces::ACCEL_2_SCALE,
            VCFInterfaces::BRAKE_1_SCALE,
            VCFInterfaces::BRAKE_2_SCALE,

            VCFInterfaces::SHDN_H_SCALE,
            VCFInterfaces::SHDN_D_SCALE,
            VCFInterfaces::FL_LOADCELL_SCALE,
            VCFInterfaces::FR_LOADCELL_SCALE,
            VCFInterfaces::FR_SUS_POT_SCALE,
            VCFInterfaces::FL_SUS_POT_SCALE,
            VCFInterfaces::BRAKE_PRESSURE_FRONT_SCALE,
            VCFInterfaces::BRAKE_PRESSURE_REAR_SCALE
        },
        ADCOffsets_s
        {
            VCFInterfaces::PEDAL_REF_2V5_OFFSET,
            VCFInterfaces::STEERING_1_OFFSET,
            VCFInterfaces::STEERING_2_OFFSET,
            VCFInterfaces::ACCEL_1_OFFSET,
            VCFInterfaces::ACCEL_2_OFFSET,
            VCFInterfaces::BRAKE_1_OFFSET,
            VCFInterfaces::BRAKE_2_OFFSET,

            VCFInterfaces::SHDN_H_OFFSET,
            VCFInterfaces::SHDN_D_OFFSET,
            VCFInterfaces::FL_LOADCELL_OFFSET,
            VCFInterfaces::FR_LOADCELL_OFFSET,
            VCFInterfaces::FR_SUS_POT_OFFSET,
            VCFInterfaces::FL_SUS_POT_OFFSET,
            VCFInterfaces::BRAKE_PRESSURE_FRONT_OFFSET,
            VCFInterfaces::BRAKE_PRESSURE_REAR_OFFSET
        }
    );
}

void loop()
{
    if (millis() - DELAY > last)
    {
        ADCInterfaceInstance::instance().tick_adc0();
        Serial.print("\n===== ADC 0 =====\n");
        Serial.printf("2V5 Pedal Reference Raw:  %d\n", ADCInterfaceInstance::instance().pedal_reference().raw);
        Serial.printf("Steering 1 (CW) Raw:      %d\n", ADCInterfaceInstance::instance().get_steering_degrees_cw().raw);
        Serial.printf("Steering 2 (CCW) Raw:     %d\n", ADCInterfaceInstance::instance().get_steering_degrees_ccw().raw);
        Serial.printf("Acceleration 1 Raw:       %d\n", ADCInterfaceInstance::instance().get_acceleration_1().raw);
        Serial.printf("Acceleration 2 Raw:       %d\n", ADCInterfaceInstance::instance().get_acceleration_2().raw);
        Serial.printf("Brake 1 Raw:              %d\n", ADCInterfaceInstance::instance().get_brake_1().raw);
        Serial.printf("Brake 2 Raw:              %d\n", ADCInterfaceInstance::instance().get_brake_2().raw);

        ADCInterfaceInstance::instance().tick_adc1();
        Serial.printf("\n===== ADC 1 =====\n");
        Serial.printf("SHDN H Raw:                %d\n", ADCInterfaceInstance::instance().shdn_h().raw);
        Serial.printf("SHDN D Raw:                %d\n", ADCInterfaceInstance::instance().shdn_d().raw);
        Serial.printf("FL Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().get_FL_load_cell().raw);
        Serial.printf("FR Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().get_FR_load_cell().raw);
        Serial.printf("FR Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().get_FR_sus_pot().raw);
        Serial.printf("FL Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().get_FL_sus_pot().raw);
        Serial.printf("Front Brake Pressure Raw:  %d\n", ADCInterfaceInstance::instance().get_brake_pressure_front().raw);
        Serial.printf("Rear Brake Pressure Raw:   %d\n", ADCInterfaceInstance::instance().get_brake_pressure_rear().raw);

        last = millis();
    }
}