#include "Arduino.h"
#include "ADCInterface.h"
#include "VCR_Constants.h"

#include "Logger.h"

unsigned long const DELAY = 100;
unsigned long last = millis();

void setup()
{
    SPI.begin();    //for ADC
    Serial.begin(115200);    //for serial monitor

    //create the ADC instance
     // Instantiate ADC interface
    ADCInterfaceInstance::create(
      ADCPinout_s {ADC0_CS, ADC1_CS},
      ADCChannels_s {
        GLV_SENSE_CHANNEL,
        CURRENT_SENSE_CHANNEL,
        REFERENCE_SENSE_CHANNEL,
        RL_LOADCELL_CHANNEL,
        RR_LOADCELL_CHANNEL,
        RL_SUS_POT_CHANNEL,
        RR_SUS_POT_CHANNEL,
        THERMISTOR_0_CHANNEL,
        THERMISTOR_1_CHANNEL,
        THERMISTOR_2_CHANNEL,
        THERMISTOR_3_CHANNEL,
        THERMISTOR_4_CHANNEL,
        THERMISTOR_5_CHANNEL,
        THERMISTOR_6_CHANNEL,
        THERMISTOR_7_CHANNEL
      },
      ADCScales_s {
        GLV_SENSE_SCALE,
        CURRENT_SENSE_SCALE,
        REFERENCE_SENSE_SCALE,
        RL_LOADCELL_SCALE,
        RR_LOADCELL_SCALE,
        RL_SUS_POT_SCALE,
        RR_SUS_POT_SCALE,
        THERMISTOR_0_SCALE,
        THERMISTOR_1_SCALE,
        THERMISTOR_2_SCALE,
        THERMISTOR_3_SCALE,
        THERMISTOR_4_SCALE,
        THERMISTOR_5_SCALE,
        THERMISTOR_6_SCALE,
        THERMISTOR_7_SCALE,
      },
      ADCOffsets_s {
        GLV_SENSE_OFFSET,
        CURRENT_SENSE_OFFSET,
        REFERENCE_SENSE_OFFSET,
        RL_LOADCELL_OFFSET,
        RR_LOADCELL_OFFSET,
        RL_SUS_POT_OFFSET,
        RR_SUS_POT_OFFSET,
        THERMISTOR_0_OFFSET,
        THERMISTOR_1_OFFSET,
        THERMISTOR_2_OFFSET,
        THERMISTOR_3_OFFSET,
        THERMISTOR_4_OFFSET,
        THERMISTOR_5_OFFSET,
        THERMISTOR_6_OFFSET,
        THERMISTOR_7_OFFSET,
      }
    );
}

void loop()
{
    if (millis() - DELAY > last) {
        ADCInterfaceInstance::instance().tick_adc0();
        Serial.print("\n===== ADC 0 =====\n");
        Serial.printf("GLV:  %d\n", ADCInterfaceInstance::instance().read_glv().raw);
        //Serial.printf("LoadCell?:      %d\n", ADCInterfaceInstance::instance().steering_degrees_cw().raw);
        // Serial.printf("Steering 2 (CCW) Raw:     %d\n", ADCInterfaceInstance::instance().steering_degrees_ccw().raw);
        // Serial.printf("Acceleration 1 Raw:       %d\n", ADCInterfaceInstance::instance().acceleration_1().raw);
        // Serial.printf("Acceleration 2 Raw:       %d\n", ADCInterfaceInstance::instance().acceleration_2().raw);
        // Serial.printf("Brake 1 Raw:              %d\n", ADCInterfaceInstance::instance().brake_1().raw);
        // Serial.printf("Brake 2 Raw:              %d\n", ADCInterfaceInstance::instance().brake_2().raw);

        // ADCInterfaceInstance::instance().tick_adc1();
        // Serial.printf("\n===== ADC 1 =====\n");
        // Serial.printf("SHDN H Raw:                %d\n", ADCInterfaceInstance::instance().shdn_h().raw);
        // Serial.printf("SHDN D Raw:                %d\n", ADCInterfaceInstance::instance().shdn_d().raw);
        // Serial.printf("FL Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().FL_load_cell().raw);
        // Serial.printf("FR Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().FR_load_cell().raw);
        // Serial.printf("FR Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().FR_sus_pot().raw);
        // Serial.printf("FL Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().FL_sus_pot().raw);
        // Serial.printf("Front Brake Pressure Raw:  %d\n", ADCInterfaceInstance::instance().brake_pressure_front().raw);
        // Serial.printf("Rear Brake Pressure Raw:   %d\n", ADCInterfaceInstance::instance().brake_pressure_rear().raw);

        last = millis();
    }
}