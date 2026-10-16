#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

namespace aegistrack {

struct TimingMetricStats {
    std::uint64_t sample_count{0};
    double last_us{0.0};
    double mean_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
    double total_us{0.0};
};

/**
 * @brief Lightweight thread-safe internal profiler for live subsystem timing telemetry.
 */
class InternalProfiler {
public:
    InternalProfiler() = default;

    void recordSample(const std::string& scope_name, double duration_us) {
        std::scoped_lock lock(mutex_);
        auto& stats = stats_[scope_name];
        if (stats.sample_count == 0) {
            stats.min_us = duration_us;
            stats.max_us = duration_us;
        } else {
            stats.min_us = std::min(stats.min_us, duration_us);
            stats.max_us = std::max(stats.max_us, duration_us);
        }
        ++stats.sample_count;
        stats.last_us = duration_us;
        stats.total_us += duration_us;
        stats.mean_us = stats.total_us / static_cast<double>(stats.sample_count);
    }

    [[nodiscard]] TimingMetricStats statsFor(const std::string& scope_name) const {
        std::scoped_lock lock(mutex_);
        const auto it = stats_.find(scope_name);
        if (it != stats_.end()) {
            return it->second;
        }
        return TimingMetricStats{};
    }

    [[nodiscard]] std::unordered_map<std::string, TimingMetricStats> allStats() const {
        std::scoped_lock lock(mutex_);
        return stats_;
    }

    void reset() {
        std::scoped_lock lock(mutex_);
        stats_.clear();
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, TimingMetricStats> stats_;
};

/**
 * @brief RAII scope timer that records elapsed microseconds into an InternalProfiler.
 */
class ScopedTimer {
public:
    ScopedTimer(InternalProfiler* profiler, std::string scope_name)
        : profiler_(profiler),
          scope_name_(std::move(scope_name)),
          start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        if (profiler_ != nullptr) {
            const auto end = std::chrono::steady_clock::now();
            const double elapsed_us =
                std::chrono::duration<double, std::micro>(end - start_).count();
            profiler_->recordSample(scope_name_, elapsed_us);
        }
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    ScopedTimer& operator=(ScopedTimer&&) = delete;

private:
    InternalProfiler* profiler_;
    std::string scope_name_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace aegistrack
