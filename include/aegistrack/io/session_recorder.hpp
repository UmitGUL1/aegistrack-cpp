#pragma once

#include "aegistrack/config/scenario_config.hpp"
#include "aegistrack/events/event.hpp"
#include "aegistrack/metrics/metrics_collector.hpp"
#include "aegistrack/sensors/measurement.hpp"
#include "aegistrack/simulation/simulation_engine.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace aegistrack {

struct RecordedSession {
    std::string format_version{"1.0.0"};
    ScenarioConfig scenario_config;
    std::vector<Measurement> recorded_measurements;
    std::vector<SystemEvent> recorded_events;
    std::vector<MetricsSnapshot> recorded_metrics;
    double cumulative_position_rmse_m{0.0};
    double cumulative_velocity_rmse_m{0.0};
    double cumulative_raw_measurement_rmse_m{0.0};
};

struct MeasurementReplayResult {
    TrackManagerConfig tracker_config;
    double cumulative_position_rmse_m{0.0};
    double cumulative_velocity_rmse_m{0.0};
    std::uint64_t total_measurements_processed{0};
    std::uint64_t total_tracks_created{0};
    std::uint64_t total_tracks_deleted{0};
    std::vector<MetricsSnapshot> metrics_history;
};

/**
 * @brief Handles versioned JSON session recording/loading, CSV tabular export,
 * and deterministic measurement replay across alternative tracking algorithms.
 */
class SessionRecorder {
public:
    static constexpr const char* kCurrentFormatVersion = "1.0.0";

    /// Captures a complete RecordedSession from a finished or paused SimulationEngine.
    [[nodiscard]] static RecordedSession capture(const SimulationEngine& engine);

    /// Saves a RecordedSession to a versioned JSON file.
    static void saveSessionJson(const RecordedSession& session,
                                const std::filesystem::path& filepath);

    /// Loads and validates a RecordedSession from a versioned JSON file.
    [[nodiscard]] static RecordedSession loadSessionJson(const std::filesystem::path& filepath);

    /// Exports session JSON + CSV tables (tracks.csv, measurements.csv, metrics.csv, ground_truth.csv)
    /// into output_dir.
    static void exportBundle(const SimulationEngine& engine,
                             const std::filesystem::path& output_dir);

    /// Exports track histories to CSV.
    static void exportTracksCsv(const SimulationEngine& engine,
                                const std::filesystem::path& filepath);

    /// Exports timestamped measurements to CSV.
    static void exportMeasurementsCsv(std::span<const Measurement> measurements,
                                      const std::filesystem::path& filepath);

    /// Exports time-series metrics to CSV.
    static void exportMetricsCsv(std::span<const MetricsSnapshot> metrics,
                                 const std::filesystem::path& filepath);

    /// Exports ground-truth trajectories to CSV.
    static void exportGroundTruthCsv(std::span<const SimulatedObject> objects,
                                     const std::filesystem::path& filepath);

    /**
     * @brief Replays recorded measurements from a session through a custom TrackManagerConfig
     * while evaluating against the deterministic ground truth, enabling algorithm comparison
     * (e.g., NearestNeighbor vs Hungarian) on identical recorded sensor observations.
     */
    [[nodiscard]] static MeasurementReplayResult replayMeasurements(
        const RecordedSession& session,
        const TrackManagerConfig& override_tracker_config);
};

}  // namespace aegistrack
