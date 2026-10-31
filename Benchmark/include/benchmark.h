#pragma once
#include <Arduino.h>
#include <imxrt.h>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cstdint>

#if !defined(__IMXRT1062__)
#error "This benchmark currently supports Teensy 4.0 and 4.1 only"
#endif

namespace bench {
///prevent compiler from optimizing away a result
template <typename T>
inline void do_not_optimize(const T& value) noexcept
{
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "g"(value) : "memory");
#else
    (void)value;
#endif
}

struct Statistics
{
    double min_ns{};
    double avg_ns{};
    double p50_ns{};
    double p90_ns{};
    double p99_ns{};
    double p99_9_ns{};
    double max_ns{};
};

class Benchmark
{
public:
    struct Config
    {
        std::size_t warmup_iterations = 2000;
        std::size_t measurements = 3000;
        std::size_t batch_size = 100;
    };
    Benchmark();

    explicit Benchmark(Config config);
    void begin();

    ///warmup
    template <typename Function>
    void warmup(Function&& function);

    ///measure
    template <typename Function>
    void measure(Function&& function);

    ///stats
    [[nodiscard]]
    Statistics statistics() const;

    ///samples
    [[nodiscard]]
    const std::vector<double>& samples() const noexcept;

    ///reset measurements
    void reset();
    void print_results(Stream& output) const;

private:
    Config config_;
    std::vector<double> samples_;
    bool initialized_ = false;
    static uint32_t read_cycle_count() noexcept;
    static double cycle_to_ns(uint32_t cycles) noexcept;
};

///warmup
template <typename Function>
void Benchmark::warmup(Function&& function)
{
    for (std::size_t i = 0; i < config_.warmup_iterations; ++i)
    {
        function();
    }
}

///measurement
template <typename Function>
void Benchmark::measure(Function&& function)
{
    if (!initialized_)
    {
        return;
    }
    samples_.clear();
    samples_.reserve(config_.measurements);

    for (std::size_t i = 0; i < config_.measurements; ++i)
    {
        const uint32_t start = read_cycle_count();
        for (std::size_t j = 0; j < config_.batch_size; ++j)
        {
            function();
        }

        const uint32_t end = read_cycle_count();
        const uint32_t elapsed_cycles = end - start;
        const double elapsed_ns = cycle_to_ns(elapsed_cycles);
        const double per_operation_ns = elapsed_ns / static_cast<double>(config_.batch_size);
        samples_.push_back(per_operation_ns);
    }
}

}///namespace bench
