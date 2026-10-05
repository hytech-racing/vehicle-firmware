#include "benchmark.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace bench {
Benchmark::Benchmark() : Benchmark(Config{}) {}
Benchmark::Benchmark(Config config) : config_(config)
{
    if (config_.measurements == 0)
    {
        throw std::invalid_argument("measurements must > 0");
    }

    if (config_.batch_size == 0)
    {
        throw std::invalid_argument("batch_size must > 0");
    }
    samples_.reserve(config_.measurements);
}

Statistics Benchmark::statistics() const
{
    if (samples_.empty())
    {
        throw std::logic_error("no benchmark samples available");
    }

    auto sorted = samples_;///copy, preserve the original order of samples
    std::sort(sorted.begin(), sorted.end());

    const auto percentile =
        [&sorted](double p) -> double
        {
            const std::size_t index = static_cast<std::size_t>(p * static_cast<double>( sorted.size() - 1));
            return sorted[index];
        };

    return Statistics{
        .min_ns   = sorted.front(),
        .p50_ns   = percentile(0.50),
        .p90_ns   = percentile(0.90),
        .p99_ns   = percentile(0.99),
        .p99_9_ns = percentile(0.999),
        .max_ns   = sorted.back()
    };
}

const std::vector<double>& Benchmark::samples() const noexcept
{
    return samples_;
}

void Benchmark::reset()
{
    samples_.clear();
}

void Benchmark::print_results() const
{
    const auto stats = statistics();
    std::cout
    << '\n' 
        << "p50   = "
        << stats.p50_ns
        << " ns\n";

    std::cout
        << "p90   = "
        << stats.p90_ns
        << " ns\n";

    std::cout
        << "p99   = "
        << stats.p99_ns
        << " ns\n";

    std::cout
        << "p99.9 = "
        << stats.p99_9_ns
        << " ns\n";

    std::cout
        << "min   = "
        << stats.min_ns
        << " ns\n";

    std::cout
        << "max   = "
        << stats.max_ns
        << " ns\n";
}

}///namespace bench



//////////////   SAMPLE USAGE IN MAIN:   /////////////
// #include "benchmark.h"
// #include <iostream>


// int calculate_arithmetic(int argumentOne, int argumentTwo)
// {

//     ///std::cout << "place holder";
//     return argumentOne * argumentTwo * argumentTwo * 123 * 1234 / 5678 * 123;
//     //return argumentOne * argumentTwo;

// }

// int main()
// {
//     constexpr std::size_t warmup_iterations = 100;
//     constexpr std::size_t measurements = 30;
//     constexpr std::size_t batch_size = 20;

//     ///HOW TO USE THIS BENCHMARK
//     /*
//         0. initilize the benchmark parameters and declare them as constexpr (shown above)
//         1. initialize arguments of the measured function in main() eg. argumentOne, argumentTwo ///do not use constexpr here
//         2. initialize return value eg.result if any ///do not use constexpr here
//         3. warm up the cache(optional)
//         4. measure
//         5. print statistics
//         Note: for 3-5, use trailing drivebrain::bench::do_not_optimize(...) as shown below so the compiler does do optimizations that would invalidate the benchmark
//     */

//     int argumentOne = 100;
//     int argumentTwo = 5;
//     long long result = 0;

//     bench::Benchmark benchmark({
//         .warmup_iterations = warmup_iterations,
//         .measurements = measurements,
//         .batch_size = batch_size
//     });

//     ///optional step 3: warmup
//     benchmark.warmup([&]
//     {
//         result = calculate_arithmetic(argumentOne, argumentTwo);
//         bench::do_not_optimize(result);
//     });

//     ///step 4: measurement
//     benchmark.measure([&]
//     {
//         result = calculate_arithmetic(argumentOne, argumentTwo);
//         bench::do_not_optimize(result);
//     });

//     ///step 5: print stats
//     benchmark.print_results();
//     bench::do_not_optimize(result);///keep the accumulated result observable

//     std::cout << "result = " << result << '\n';
//     return 0;
// }


