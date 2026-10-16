#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace aegistrack {

/**
 * @brief Deterministic pseudo-random number generator based on std::mt19937_64.
 *
 * Guarantees identical random sequences for identical seeds when invoked in the
 * same deterministic simulation order.
 */
class DeterministicRng {
public:
    explicit DeterministicRng(std::uint64_t seed = 42ULL)
        : seed_(seed), engine_(seed) {}

    void reseed(std::uint64_t new_seed) noexcept {
        seed_ = new_seed;
        engine_.seed(new_seed);
    }

    [[nodiscard]] std::uint64_t seed() const noexcept {
        return seed_;
    }

    /// Samples a Gaussian distribution N(mean, stddev^2). If stddev == 0, returns mean.
    [[nodiscard]] double normal(double mean, double stddev) {
        if (!std::isfinite(mean) || !std::isfinite(stddev) || stddev < 0.0) {
            throw std::invalid_argument("DeterministicRng::normal requires finite mean and stddev >= 0");
        }
        if (stddev == 0.0) {
            return mean;
        }
        std::normal_distribution<double> dist(mean, stddev);
        return dist(engine_);
    }

    /// Samples a continuous uniform distribution U[min_val, max_val).
    [[nodiscard]] double uniform(double min_val, double max_val) {
        if (!std::isfinite(min_val) || !std::isfinite(max_val) || min_val > max_val) {
            throw std::invalid_argument("DeterministicRng::uniform requires finite min_val <= max_val");
        }
        if (min_val == max_val) {
            return min_val;
        }
        std::uniform_real_distribution<double> dist(min_val, max_val);
        return dist(engine_);
    }

    /// Samples a Bernoulli trial with success probability p in [0, 1].
    [[nodiscard]] bool bernoulli(double probability) {
        if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0) {
            throw std::invalid_argument("DeterministicRng::bernoulli requires probability in [0, 1]");
        }
        if (probability <= 0.0) {
            return false;
        }
        if (probability >= 1.0) {
            return true;
        }
        std::bernoulli_distribution dist(probability);
        return dist(engine_);
    }

    /// Samples a Poisson distribution with rate lambda >= 0.
    [[nodiscard]] std::uint32_t poisson(double mean_rate) {
        if (!std::isfinite(mean_rate) || mean_rate < 0.0) {
            throw std::invalid_argument("DeterministicRng::poisson requires finite mean_rate >= 0");
        }
        if (mean_rate == 0.0) {
            return 0U;
        }
        std::poisson_distribution<std::uint32_t> dist(mean_rate);
        return dist(engine_);
    }

    [[nodiscard]] std::mt19937_64& engine() noexcept {
        return engine_;
    }

private:
    std::uint64_t seed_;
    std::mt19937_64 engine_;
};

}  // namespace aegistrack
