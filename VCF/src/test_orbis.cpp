#include <Arduino.h>
#include "SteeringEncoderInterface.h"
#include "Orbis_BR.h"

constexpr int SAMPLE_RATE = 5000;

// Globals
unsigned long lastPrintTime = 0;


void setup()
{
    Serial.begin(OrbisConstants::ORBIS_BR_DEFAULT_BAUD_RATE);

    while(!Serial);
    Serial.println("Serial Monitor connected.");        // Debug line

    OrbisBRInstance::create(&Serial2);


    Serial.println("Calling Self-Calib");                  // Debug line

    bool _isCalibrated = false;    // Assume sensor is not calibrated
    while (!_isCalibrated) //NOLINT
    {
        _isCalibrated = OrbisBRInstance::instance().performSelfCalibration();
    }

    OrbisBRInstance::instance().setEncoderOffset();

    /* --- Block For Debugging --- */
    // OrbisBRInstance::instance().sample();
    // Serial.print("\n\n");

    // SteeringEncoderReading_s result = OrbisBRInstance::instance().convert();
    //     if (result.errors.generalError)   { Serial.println("General error"); }
    //     if (result.errors.generalWarning) { Serial.println("General warning"); }
    //     if (result.errors.noData)         { Serial.println("No data"); }

    //     OrbisErrorFlags_s orbisErrors = OrbisBRInstance::instance().getOrbisErrors();
    //     if (orbisErrors.distFar)              { Serial.println("Readhead too far"); }
    //     if (orbisErrors.distNear)             { Serial.println("Readhead too close"); }
    //     if (orbisErrors.tempRange)            { Serial.println("Temperature out of range"); }
    //     if (orbisErrors.speedHigh)            { Serial.println("Speed too high"); }
    //     if (orbisErrors.counterError)         { Serial.println("Multiturn counter error"); }
    //     if (orbisErrors.calibrationTimeout)   { Serial.println("Calibration timed out"); }
    //     if (orbisErrors.calibrationParameter) { Serial.println("Calibration parameter error"); }
}

void loop()
{
    if (millis() - lastPrintTime >= SAMPLE_RATE)
    {
        lastPrintTime = millis();

        OrbisBRInstance::instance().sample();

        SteeringEncoderReading_s result = OrbisBRInstance::instance().getLastReading();
        if (result.errors.dataInvalid)   { Serial.println("General error"); }
        if (result.errors.operatingLimit) { Serial.println("General warning"); }
        if (result.errors.noData)         { Serial.println("No data"); }

        OrbisErrorFlags_s orbisErrors = OrbisBRInstance::instance().getOrbisDetailedErrors();
        if (orbisErrors.dist_far)              { Serial.println("Readhead too far"); }
        if (orbisErrors.dist_near)             { Serial.println("Readhead too close"); }
        if (orbisErrors.temp_out_of_range)     { Serial.println("Temperature out of range"); }
        if (orbisErrors.speed_high)            { Serial.println("Speed too high"); }
        if (orbisErrors.counter_error)         { Serial.println("Multiturn counter error"); }
        if (orbisErrors.calibration_timeout)   { Serial.println("Calibration timed out"); }
        if (orbisErrors.calibration_parameter) { Serial.println("Calibration parameter error"); }

    }
}
