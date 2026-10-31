#include "benchmark.h"

namespace bench {
Benchmark::Benchmark() : Benchmark(Config{}) {}

Benchmark::Benchmark(Config config)
    : config_(config)
{
    if (config_.measurements == 0)
    {
        config_.measurements = 1;
    }

    if (config_.batch_size == 0)
    {
        config_.batch_size = 1;
    }

    samples_.reserve(config_.measurements);
}

void Benchmark::begin()
{
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    ARM_DWT_CYCCNT = 0;
    initialized_ = true;
}

uint32_t Benchmark::read_cycle_count() noexcept
{
    return ARM_DWT_CYCCNT;
}

double Benchmark::cycle_to_ns(uint32_t cycles) noexcept
{
#if defined(F_CPU_ACTUAL)
    constexpr double cpu_hz = static_cast<double>(F_CPU_ACTUAL);
#else
    constexpr double cpu_hz = static_cast<double>(F_CPU);
#endif
    return static_cast<double>(cycles) * (1000000000.0 / cpu_hz);
}

Statistics Benchmark::statistics() const
{
    if (samples_.empty())
    {
        return {};
    }

    auto sorted = samples_;
    std::sort(sorted.begin(), sorted.end());

    double sum = 0.0;
    for (const double sample : samples_)
    {
        sum += sample;
    }

    const double average = sum / static_cast<double>(samples_.size());

    ///percentile
    const auto percentile =
        [&sorted](double p) -> double
        {
            const std::size_t index = static_cast<std::size_t>(p * static_cast<double>(sorted.size() - 1));
            return sorted[index];
        };

    return Statistics{
        sorted.front(),
        average,
        percentile(0.50),
        percentile(0.90),
        percentile(0.99),
        percentile(0.999),
        sorted.back()
    };
}

const std::vector<double>&
Benchmark::samples() const noexcept
{
    return samples_;
}

void Benchmark::reset()
{
    samples_.clear();
}

void Benchmark::print_results(Stream& output) const
{
    if (samples_.empty())
    {
        output.println("No benchmark samples available right now");
        return;
    }

    const Statistics stats = statistics();

    output.println();
    output.println(
        "========== Hytech Racing Benchmarking =========="
    );

    output.printf("Measurements : %u\n", static_cast<unsigned>(samples_.size()));

    output.printf(
        "Min          : %.3f ns\n",
        stats.min_ns
    );

    output.printf(
        "Average      : %.3f ns\n",
        stats.avg_ns
    );

    output.printf(
        "p50          : %.3f ns\n",
        stats.p50_ns
    );

    output.printf(
        "p90          : %.3f ns\n",
        stats.p90_ns
    );

    output.printf(
        "p99          : %.3f ns\n",
        stats.p99_ns
    );

    output.printf(
        "p99.9        : %.3f ns\n",
        stats.p99_9_ns
    );

    output.printf(
        "Max          : %.3f ns\n",
        stats.max_ns
    );

    output.println(
        "======================================="
    );
}

}///namespace bench




