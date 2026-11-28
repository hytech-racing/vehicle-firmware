#include "ADCInterface.hpp"
#include "Arduino.h"
#include "Logger.h"
#include "VCR_Constants.hpp"

const unsigned long DELAY = 100;
unsigned long last = millis();

void setup()
{
    SPI.begin();          // for ADC
    Serial.begin(115200); // for serial monitor

    // create the ADC instance
    //  Instantiate ADC interface
    ADCInterfaceInstance::create(
        ADCPinout_s{VCRInterfaces::ADC0_CS, VCRInterfaces::ADC1_CS},
        ADCChannels_s{
            VCRInterfaces::GLV_SENSE_CHANNEL,
            VCRInterfaces::CURRENT_SENSE_CHANNEL,
            VCRInterfaces::REFERENCE_SENSE_CHANNEL,
            VCRInterfaces::RL_LOADCELL_CHANNEL,
            VCRInterfaces::RR_LOADCELL_CHANNEL,
            VCRInterfaces::RL_SUS_POT_CHANNEL,
            VCRInterfaces::RR_SUS_POT_CHANNEL,
            VCRInterfaces::THERMISTOR_0_CHANNEL,
            VCRInterfaces::THERMISTOR_1_CHANNEL,
            VCRInterfaces::THERMISTOR_2_CHANNEL,
            VCRInterfaces::THERMISTOR_3_CHANNEL,
            VCRInterfaces::THERMISTOR_4_CHANNEL,
            VCRInterfaces::THERMISTOR_5_CHANNEL,
            VCRInterfaces::THERMISTOR_6_CHANNEL,
            VCRInterfaces::THERMISTOR_7_CHANNEL},
        ADCScales_s{
            VCRInterfaces::GLV_SENSE_SCALE,
            VCRInterfaces::CURRENT_SENSE_SCALE,
            VCRInterfaces::REFERENCE_SENSE_SCALE,
            VCRInterfaces::RL_LOADCELL_SCALE,
            VCRInterfaces::RR_LOADCELL_SCALE,
            VCRInterfaces::RL_SUS_POT_SCALE,
            VCRInterfaces::RR_SUS_POT_SCALE,
            VCRInterfaces::THERMISTOR_0_SCALE,
            VCRInterfaces::THERMISTOR_1_SCALE,
            VCRInterfaces::THERMISTOR_2_SCALE,
            VCRInterfaces::THERMISTOR_3_SCALE,
            VCRInterfaces::THERMISTOR_4_SCALE,
            VCRInterfaces::THERMISTOR_5_SCALE,
            VCRInterfaces::THERMISTOR_6_SCALE,
            VCRInterfaces::THERMISTOR_7_SCALE,
        },
        ADCOffsets_s{
            VCRInterfaces::GLV_SENSE_OFFSET,
            VCRInterfaces::CURRENT_SENSE_OFFSET,
            VCRInterfaces::REFERENCE_SENSE_OFFSET,
            VCRInterfaces::RL_LOADCELL_OFFSET,
            VCRInterfaces::RR_LOADCELL_OFFSET,
            VCRInterfaces::RL_SUS_POT_OFFSET,
            VCRInterfaces::RR_SUS_POT_OFFSET,
            VCRInterfaces::THERMISTOR_0_OFFSET,
            VCRInterfaces::THERMISTOR_1_OFFSET,
            VCRInterfaces::THERMISTOR_2_OFFSET,
            VCRInterfaces::THERMISTOR_3_OFFSET,
            VCRInterfaces::THERMISTOR_4_OFFSET,
            VCRInterfaces::THERMISTOR_5_OFFSET,
            VCRInterfaces::THERMISTOR_6_OFFSET,
            VCRInterfaces::THERMISTOR_7_OFFSET,
        });
}

void loop()
{
    if (millis() - DELAY > last)
    {
        ADCInterfaceInstance::instance().tickADC0();
        Serial.print("\n===== ADC 0 =====\n");
        Serial.printf("GLV:                      %d\n", ADCInterfaceInstance::instance().getGlv().raw);
        Serial.printf("BSPD Current:             %d\n", ADCInterfaceInstance::instance().getBspdCurrent().raw);
        Serial.printf("BSPD Ref Current:             %d\n", ADCInterfaceInstance::instance().getBspdReferenceCurrent().raw);
        Serial.printf("RL Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().getRLLoadCell().raw);
        Serial.printf("RR Load Cell Raw:          %d\n", ADCInterfaceInstance::instance().getRRLoadCell().raw);
        Serial.printf("RR Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().getRRSusPot().raw);
        Serial.printf("RL Sus Pot Raw:            %d\n", ADCInterfaceInstance::instance().getRLSusPot().raw);

        ADCInterfaceInstance::instance().tickADC1();
        Serial.printf("\n===== ADC 1 =====\n");
        Serial.printf("Thermisistor 0:                %d\n", ADCInterfaceInstance::instance().getThermistor0().raw);
        Serial.printf("Thermisistor 1:                %d\n", ADCInterfaceInstance::instance().getThermistor1().raw);
        Serial.printf("Thermisistor 2:                %d\n", ADCInterfaceInstance::instance().getThermistor2().raw);
        Serial.printf("Thermisistor 3:                %d\n", ADCInterfaceInstance::instance().getThermistor3().raw);
        Serial.printf("Thermisistor 4:                %d\n", ADCInterfaceInstance::instance().getThermistor4().raw);
        Serial.printf("Thermisistor 5:                %d\n", ADCInterfaceInstance::instance().getThermistor5().raw);
        Serial.printf("Thermisistor 6:                %d\n", ADCInterfaceInstance::instance().getThermistor6().raw);
        Serial.printf("Thermisistor 7:                %d\n", ADCInterfaceInstance::instance().getThermistor7().raw);

        last = millis();
    }
}
