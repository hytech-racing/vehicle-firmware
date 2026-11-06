#include "benchmark.h"
#include <algorithm>

namespace bench
{
Benchmark::Benchmark() : Benchmark(BenchmarkConfig{})
{
}

Benchmark::Benchmark(BenchmarkConfig config) : _config(config)
{
    if (_config.measurements == 0)
    {
        _config.measurements = 1;
    }

    if (_config.batch_size == 0)
    {
        _config.batch_size = 1;
    }
    _samples.reserve(_config.measurements);
}

void Benchmark::begin()
{
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    ARM_DWT_CYCCNT = 0;
    _initialized = true;
}

uint32_t Benchmark::_read_cycle_count() noexcept
{
    return ARM_DWT_CYCCNT;
}

double Benchmark::_cycle_to_ns(uint32_t cycles) noexcept
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
    if (_samples.empty())
    {
        return {};
    }

    auto sorted = _samples;
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;

    for (const double sample : _samples)
    {
        sum += sample;
    }

    const double average = sum / static_cast<double>(_samples.size());
    const auto percentile = [&sorted](double p) -> double
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

const std::vector<double>& Benchmark::samples() const noexcept
{
    return _samples;
}

void Benchmark::reset()
{
    _samples.clear();
}

void Benchmark::print_results(Stream& output) const
{
    if (_samples.empty())
    {
        output.println("No benchmark samples available right now");
        return;
    }

    const Statistics stats = statistics();
    output.println();
    output.println("========== Hytech Racing Benchmarking ==========");
    output.printf("Measurements : %u\n", static_cast<unsigned>(_samples.size()));
    output.printf("Min          : %.3f ns\n", stats.min_ns);
    output.printf("Average      : %.3f ns\n", stats.avg_ns);
    output.printf("p50          : %.3f ns\n", stats.percentile50_ns);
    output.printf("p90          : %.3f ns\n", stats.percentile90_ns);
    output.printf("p99          : %.3f ns\n", stats.percentile99_ns);
    output.printf("p99.9        : %.3f ns\n", stats.percentile99_9_ns);
    output.printf("Max          : %.3f ns\n", stats.max_ns);
    output.println("===============================================");

}
}///namespace bench

