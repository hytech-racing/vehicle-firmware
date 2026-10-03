#include "ADCInterface.hpp"
#include "VCF_Constants.hpp"

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
        ADCInterfaceInstance::instance().tickADC0();
        Serial.print("\n===== ADC 0 =====\n");
        Serial.printf("2V5 Pedal Reference Raw:  %d\n", ADCInterfaceInstance::instance().getPedalReference().raw);
        Serial.printf("Steering 1 (CW) Raw:      %d\n", ADCInterfaceInstance::instance().getSteeringDegreesCW().raw);
        Serial.printf("Steering 2 (CCW) Raw:     %d\n", ADCInterfaceInstance::instance().getSteeringDegreesCCW().raw);
        Serial.printf("Acceleration 1 Raw:       %d\n", ADCInterfaceInstance::instance().getAcceleration1().raw);
        Serial.printf("Acceleration 2 Raw:       %d\n", ADCInterfaceInstance::instance().getAcceleration2().raw);
        Serial.printf("Brake 1 Raw:              %d\n", ADCInterfaceInstance::instance().getBrake1().raw);
        Serial.printf("Brake 2 Raw:              %d\n", ADCInterfaceInstance::instance().getBrake2().raw);

        ADCInterfaceInstance::instance().tickADC1();
        Serial.printf("\n===== ADC 1 =====\n");
        Serial.printf("SHDN H Raw:                %d\n", ADCInterfaceInstance::instance().getShutdownH().raw);
        Serial.printf("SHDN D Raw:                %d\n", ADCInterfaceInstance::instance().getShutdownD().raw);
        Serial.printf("FL Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().getFLLoadcell().raw);
        Serial.printf("FR Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().getFRLoadcell().raw);
        Serial.printf("FR Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().getFRSuspot().raw);
        Serial.printf("FL Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().getFLSuspot().raw);
        Serial.printf("Front Brake Pressure Raw:  %d\n", ADCInterfaceInstance::instance().getBrakePressureFront().raw);
        Serial.printf("Rear Brake Pressure Raw:   %d\n", ADCInterfaceInstance::instance().getBrakePressureRear().raw);

        last = millis();
    }
}