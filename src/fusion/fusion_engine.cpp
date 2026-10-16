#include "aegistrack/fusion/fusion_engine.hpp"

#include <chrono>

namespace aegistrack {

FusionEngine::FusionEngine(TrackManagerConfig tracker_config,
                           EventBus* event_bus,
                           InternalProfiler* profiler,
                           std::size_t max_recent_measurements)
    : track_manager_(tracker_config, nullptr, event_bus),
      event_bus_(event_bus),
      profiler_(profiler),
      max_recent_measurements_(max_recent_measurements) {}

TrackStepSummary FusionEngine::step(std::span<const Measurement> ready_measurements,
                                    TimestampSec current_time) {
    const auto t_start = std::chrono::steady_clock::now();

    for (const auto& m : ready_measurements) {
        if (max_recent_measurements_ > 0 &&
            recent_measurements_.size() >= max_recent_measurements_) {
            recent_measurements_.erase(recent_measurements_.begin());
        }
        recent_measurements_.push_back(m);
    }
    total_measurements_processed_ += ready_measurements.size();

    const TrackStepSummary summary =
        track_manager_.processMeasurements(ready_measurements, current_time);

    const auto t_end = std::chrono::steady_clock::now();
    last_update_latency_us_ =
        std::chrono::duration<double, std::micro>(t_end - t_start).count();
    if (profiler_ != nullptr) {
        profiler_->recordSample("fusion.step_us", last_update_latency_us_);
    }

    return summary;
}

void FusionEngine::reset() {
    track_manager_.reset();
    recent_measurements_.clear();
    total_measurements_processed_ = 0;
    last_update_latency_us_ = 0.0;
}

}  // namespace aegistrack
