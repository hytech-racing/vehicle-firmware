#pragma once
#include <vector>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace bench {
///prevent the compiler from optimizing away a calculated value
///shoulde be compatible with GCC/Clang, not with Windows currently
template <typename T>
inline void do_not_optimize(const T& value) noexcept
{
#if defined(__clang__) || defined(__GNUC__)
    asm volatile("" : : "g"(value) : "memory");
#else
    (void)value;
#endif
}

struct Statistics
{
    double min_ns{};
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
        std::size_t warmup_iterations = 2000;///can disable it by feeding 0
        std::size_t measurements = 3000;///multiple samples
        std::size_t batch_size = 1;///to amortize timer overhead
    };

    Benchmark();
    explicit Benchmark(Config config);

    template <typename Function>
    void warmup(Function&& function);

    template <typename Function>
    void measure(Function&& function);

    [[nodiscard]]
    Statistics statistics() const;

    [[nodiscard]]
    const std::vector<double>& samples() const noexcept;

    void reset();

    void print_results() const;

private:
    Config config_;
    std::vector<double> samples_;
};

///
template <typename Function>
void Benchmark::warmup(Function&& function)///warm up the cache(optional)
{
    for (std::size_t i = 0; i < config_.warmup_iterations; ++i)
    {
        function();
    }
}

template <typename Function>
void Benchmark::measure(Function&& function)
{
    samples_.clear();
    samples_.reserve(config_.measurements);
    for (std::size_t i = 0; i < config_.measurements; ++i)
    {
        ///
        const auto start = std::chrono::steady_clock::now();

        ///batching between the two timer calls to amortize timer overhead
        for (std::size_t j = 0; j < config_.batch_size; ++j)
        {
            function();
        }

        const auto end = std::chrono::steady_clock::now();
        ///
        const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        const auto per_operation_ns = static_cast<double>(elapsed_ns / static_cast<double>(config_.batch_size));
        samples_.push_back(per_operation_ns);
    }
}
}///namespace bench
