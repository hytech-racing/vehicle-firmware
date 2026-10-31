#include <Arduino.h>
#include "benchmark.h"

///benchmark config with default valuues
bench::Benchmark::Config config = {
    2000,//warmup iterations
    3000,//measurements
    100 //batch size
};

bench::Benchmark benchmark(config);
///this prevents optimizations that would optimize away code
volatile int resultInt = 0;
volatile float resultFloat = 0.0f;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    ///initialize DWT
    benchmark.begin();
    Serial.println();
    Serial.println("================================");
    Serial.println("     HyTech Racing Test Benchmark");
    Serial.println("================================");

    Serial.printf(
        "CPU frequency: %lu MHz\n",
        static_cast<unsigned long>(F_CPU_ACTUAL / 1000000)
    );

    ///TEST 1: Integer addition
    Serial.println("\nTEST 1: Integer addition");

    benchmark.warmup([]() {
        resultInt = 100 + 200;
        bench::do_not_optimize(resultInt);
    });

    benchmark.measure([]() {
        resultInt = 100 + 200;
        bench::do_not_optimize(resultInt);
    });

    benchmark.print_results(Serial);
    benchmark.reset();

    ///TEST 2: Floating-point multiplication
    Serial.println("\nTEST 2: Floating-point multiplication");

    benchmark.warmup([]() {
        resultFloat = 12.5f * 3.14f;
        bench::do_not_optimize(resultFloat);
    });

    benchmark.measure([]() {
        resultFloat = 12.5f * 3.14f;
        bench::do_not_optimize(resultFloat);
    });

    benchmark.print_results(Serial);
    benchmark.reset();

    ///TEST 3: Mathematical loop
    Serial.println("\nTEST 3: Mathematical loop");

    benchmark.warmup([]() {
        float x = 1.0f;
        for (int i = 0; i < 100; ++i)
        {
            x = x * 1.001f + 0.01f;
        }
        resultFloat = x;
        bench::do_not_optimize(resultFloat);
    });

    benchmark.measure([]() {
        float x = 1.0f;
        for (int i = 0; i < 100; ++i)
        {
            x = x * 1.001f + 0.01f;
        }
        resultFloat = x;
        bench::do_not_optimize(resultFloat);
    });

    benchmark.print_results(Serial);
    Serial.println("\nAll benchmarks completed.");
    Serial.flush();
}

void loop()
{
}
