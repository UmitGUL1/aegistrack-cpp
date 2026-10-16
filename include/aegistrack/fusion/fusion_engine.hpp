#pragma once

#include "aegistrack/core/profiler.hpp"
#include "aegistrack/core/types.hpp"
#include "aegistrack/events/event_bus.hpp"
#include "aegistrack/sensors/measurement.hpp"
#include "aegistrack/tracking/track_manager.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace aegistrack {

/**
 * @brief Coordinates multi-sensor measurement ingestion, track prediction, association,
 * sequential state updates, and live timing telemetry.
 */
class FusionEngine {
public:
    explicit FusionEngine(TrackManagerConfig tracker_config = {},
                          EventBus* event_bus = nullptr,
                          InternalProfiler* profiler = nullptr,
                          std::size_t max_recent_measurements = 600);

    /// Ingests a batch of ready sensor measurements and updates all tracks to current_time.
    TrackStepSummary step(std::span<const Measurement> ready_measurements,
                          TimestampSec current_time);

    void reset();

    [[nodiscard]] const TrackManager& trackManager() const noexcept { return track_manager_; }
    [[nodiscard]] TrackManager& trackManager() noexcept { return track_manager_; }

    [[nodiscard]] const std::vector<Measurement>& recentMeasurements() const noexcept {
        return recent_measurements_;
    }

    [[nodiscard]] std::uint64_t totalMeasurementsProcessed() const noexcept {
        return total_measurements_processed_;
    }
    [[nodiscard]] double lastUpdateLatencyUs() const noexcept { return last_update_latency_us_; }

private:
    TrackManager track_manager_;
    EventBus* event_bus_;
    InternalProfiler* profiler_;
    std::size_t max_recent_measurements_;
    std::vector<Measurement> recent_measurements_;
    std::uint64_t total_measurements_processed_{0};
    double last_update_latency_us_{0.0};
};

}  // namespace aegistrack
