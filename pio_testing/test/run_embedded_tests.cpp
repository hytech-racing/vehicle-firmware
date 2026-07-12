#include <Arduino.h>
#include <unity.h>

#include "MCP_ADC_test.h"
#include "ORBIS_Steering _test.h"

void setUp(void)
{

}

void tearDown(void)
{

}

int runUnityTests(void) 
{
    UNITY_BEGIN();
    /* TEST MCP_ADC */
    RUN_TEST(test_MCP_ADC_sample);
    /* Test steering encoder */
    RUN_TEST(test_steering_sample_and_convert);

    return UNITY_END();
}

void setup() 
{
    delay(500);

    runUnityTests();
}

void loop() {}