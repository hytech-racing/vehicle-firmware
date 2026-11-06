#pragma once
#include <Arduino.h>
#include <imxrt.h>
#include <cstddef>
#include <cstdint>
#include <vector>

#if !defined(__IMXRT1062__)
#error "This benchmark currently supports Teensy 4.0 and 4.1 only"
#endif

namespace bench
{

/**
 * @brief Config parameters for the benchmark.
 */
struct BenchmarkConfig
{
    std::size_t warmup_iterations = 2000;
    std::size_t measurements = 3000;
    std::size_t batch_size = 100;
};

/**
 * @brief Prevents the compiler from optimizing away a variable.
 * @tparam T Type of the value.
 * @param value Value that must remain observable to the compiler.
 */
template <typename T>
inline void do_not_optimize(const T& value) noexcept
{
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "g"(value) : "memory");
#else
    (void)value;
#endif
}

/**
 * @brief Stats from benchmark measurements.
 */
struct Statistics
{
    double min_ns{};
    double avg_ns{};
    double percentile50_ns{};
    double percentile90_ns{};
    double percentile99_ns{};
    double percentile99_9_ns{};
    double max_ns{};
};

/**
 * @brief Provides cycle-wise time measurements.
 * 
 */
class Benchmark
{
public:

    Benchmark();
    explicit Benchmark(BenchmarkConfig config);

    /**
     * @brief Initializes and enables the ARM DWT cycle counter.
     */
    void begin();

    /**
     * @brief Executes a function repeatedly to warm up the cache, etc. before measurements.
     * 
     * @tparam Function callable type.
     * @param function Function to benchmark.
     */
    template <typename Function>
    void warmup(Function&& function);

    /**
     * @brief Measures the execution time of a function.
     *
     * @tparam Function callable type.
     * @param function Function to benchmark.
     */
    template <typename Function>
    void measure(Function&& function);

    /**
     * @brief Calculates stats.
     */
    [[nodiscard]] Statistics statistics() const;

    /**
     * @brief Returns the individual benchmark samples.
     */
    [[nodiscard]] const std::vector<double>& samples() const noexcept;

    /**
     * @brief Clears all previously collected measurements.
     */
    void reset();


    void print_results(Stream& output) const;

private:

    BenchmarkConfig _config;
    std::vector<double> _samples;
    bool _initialized = false;

    /**
     * @brief Reads the current cycle counter.
     */
    static uint32_t _read_cycle_count() noexcept;

    /**
     * @brief Converts cycles into ns.
     *
     * @param cycles # of cycles.
     * @return effective execution time in ns.
     */
    static double _cycle_to_ns(uint32_t cycles) noexcept;
};

/**
 * @brief Executes the configured # of warmup iterations.
 *
 * @tparam Function callable type.
 * @param function Function to benchmark.
 */
template <typename Function>
void Benchmark::warmup(Function&& function)
{
    for (std::size_t i = 0; i < _config.warmup_iterations; ++i)
    {
        function();
    }
}

/**
 * @brief Measures the average execution time per operation.
 *
 * @tparam Function callable type.
 * @param function Function to benchmark.
 */
template <typename Function>
void Benchmark::measure(Function&& function)
{
    if (!_initialized) {return;}

    _samples.clear();
    _samples.reserve(_config.measurements);

    for (std::size_t i = 0; i < _config.measurements; ++i)
    {
        const uint32_t start = _read_cycle_count();
        for (std::size_t j = 0; j < _config.batch_size; ++j)
        {
            function();
        }
        const uint32_t end = _read_cycle_count();
        const uint32_t elapsed_cycles = end - start;
        const double elapsed_ns = _cycle_to_ns(elapsed_cycles);
        const double per_operation_ns = elapsed_ns / static_cast<double>(_config.batch_size);
        _samples.push_back(per_operation_ns);
    }
}

}/// namespace bench
