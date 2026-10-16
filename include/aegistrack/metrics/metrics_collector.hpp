#pragma once

#include "aegistrack/core/types.hpp"
#include "aegistrack/sensors/measurement.hpp"
#include "aegistrack/simulation/simulated_object.hpp"
#include "aegistrack/tracking/track.hpp"
#include "aegistrack/tracking/track_manager.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace aegistrack {

struct MetricsSnapshot {
    TimestampSec timestamp{0.0};
    double position_rmse_m{0.0};
    double velocity_rmse_m{0.0};
    double raw_measurement_rmse_m{0.0};
    std::size_t active_ground_truth_count{0};
    std::size_t confirmed_tracks{0};
    std::size_t tentative_tracks{0};
    std::size_t coasting_tracks{0};
    std::size_t matched_track_count{0};
    std::size_t false_track_count{0};
    std::uint64_t cumulative_measurements{0};
    std::uint64_t cumulative_detection_failures{0};
    std::uint64_t cumulative_false_alarms{0};
    std::uint64_t cumulative_tracks_created{0};
    std::uint64_t cumulative_tracks_deleted{0};
    double step_latency_us{0.0};
    double measurements_per_sec{0.0};
};

/**
 * @brief Evaluates estimated tracks and raw sensor measurements against simulation ground truth.
 *
 * Strictly isolated from the tracking pipeline; only reads ground truth after tracking
 * updates have completed.
 */
class MetricsCollector {
public:
    static constexpr std::size_t kHistogramBinCount = 6;
    /// Histogram bin upper bounds in meters: [0..2, 2..5, 5..10, 10..20, 20..40, >40]
    static constexpr std::array<double, 5> kHistogramBinEdgesM{2.0, 5.0, 10.0, 20.0, 40.0};

    explicit MetricsCollector(double evaluation_cutoff_distance_m = 200.0,
                              std::size_t max_history_samples = 2000);

    MetricsSnapshot evaluateStep(TimestampSec current_time,
                                 std::span<const SimulatedObject> ground_truth_objects,
                                 const TrackManager& track_manager,
                                 std::span<const Measurement> step_measurements,
                                 std::uint64_t total_detection_failures,
                                 std::uint64_t total_false_alarms,
                                 double step_latency_us);

    void reset();

    [[nodiscard]] const MetricsSnapshot& latestSnapshot() const noexcept {
        return latest_snapshot_;
    }
    [[nodiscard]] const std::vector<MetricsSnapshot>& history() const noexcept {
        return history_;
    }

    /// Overall cumulative Position RMSE across all evaluated track-object pairs in the session.
    [[nodiscard]] double cumulativePositionRmseM() const noexcept;

    /// Overall cumulative Velocity RMSE across all evaluated track-object pairs in the session.
    [[nodiscard]] double cumulativeVelocityRmseM() const noexcept;

    /// Overall cumulative raw sensor measurement Position RMSE in the session.
    [[nodiscard]] double cumulativeRawMeasurementRmseM() const noexcept;

    [[nodiscard]] const std::array<std::uint64_t, kHistogramBinCount>&
    positionErrorHistogram() const noexcept {
        return error_histogram_;
    }

private:
    void recordErrorSample(double pos_error_m, double vel_error_m_s);

    double evaluation_cutoff_distance_m_;
    std::size_t max_history_samples_;
    MetricsSnapshot latest_snapshot_{};
    std::vector<MetricsSnapshot> history_;

    double sum_sq_pos_error_{0.0};
    double sum_sq_vel_error_{0.0};
    std::uint64_t total_evaluated_pairs_{0};

    double sum_sq_raw_meas_error_{0.0};
    std::uint64_t total_raw_meas_evaluated_{0};

    std::array<std::uint64_t, kHistogramBinCount> error_histogram_{};
};

}  // namespace aegistrack
