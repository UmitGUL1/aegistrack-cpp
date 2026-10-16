#pragma once

#include "aegistrack/config/scenario_config.hpp"
#include "aegistrack/core/clock.hpp"
#include "aegistrack/core/profiler.hpp"
#include "aegistrack/core/random.hpp"
#include "aegistrack/events/event_bus.hpp"
#include "aegistrack/fusion/fusion_engine.hpp"
#include "aegistrack/metrics/metrics_collector.hpp"
#include "aegistrack/sensors/sensor_scheduler.hpp"
#include "aegistrack/simulation/simulated_object.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aegistrack {

struct SensorStatusSnapshot {
    SensorId id;
    SensorType type{SensorType::Radar};
    Vector2d position{Vector2d::Zero()};
    double max_range_m{0.0};
    double fov_rad{0.0};
    double boresight_rad{0.0};
    double sampling_frequency_hz{1.0};
    bool operational{true};
    SensorRuntimeStats stats{};
};

/**
 * @brief Immutable thread-safe snapshot of the simulation, sensors, tracks, and analytics
 * for lock-free GUI rendering.
 */
struct SimulationSnapshot {
    std::string scenario_name;
    TimestampSec timestamp{0.0};
    double duration_sec{30.0};
    std::uint64_t step_count{0};
    bool paused{true};
    bool completed{false};
    double playback_speed{1.0};

    std::vector<SimulatedObject> objects;
    std::vector<Track> tracks;
    std::vector<Measurement> recent_measurements;
    std::vector<SensorStatusSnapshot> sensors;
    std::vector<OcclusionZone> occlusion_zones;

    MetricsSnapshot latest_metrics{};
    std::vector<MetricsSnapshot> metrics_history;
    std::array<std::uint64_t, MetricsCollector::kHistogramBinCount> error_histogram{};
    double cumulative_position_rmse_m{0.0};
    double cumulative_velocity_rmse_m{0.0};
    double cumulative_raw_measurement_rmse_m{0.0};

    std::vector<SystemEvent> events;
};

struct ModalityComparisonResult {
    std::string modality_name;
    double cumulative_position_rmse_m{0.0};
    double cumulative_velocity_rmse_m{0.0};
    double cumulative_raw_measurement_rmse_m{0.0};
    std::uint64_t total_measurements_processed{0};
    std::uint64_t total_tracks_created{0};
    std::vector<MetricsSnapshot> metrics_history;
};

struct FusionComparisonReport {
    std::string scenario_name;
    std::uint64_t seed{0};
    ModalityComparisonResult radar_only;
    ModalityComparisonResult gnss_only;
    ModalityComparisonResult radar_gnss_fusion;
};

/**
 * @brief Master simulation coordinator controlling deterministic world stepping,
 * sensor scheduling, fusion execution, and telemetry snapshots.
 */
class SimulationEngine {
public:
    explicit SimulationEngine(ScenarioConfig config);

    /// Loads a new scenario configuration and resets the engine.
    void loadScenario(ScenarioConfig config);

    /// Resets the current scenario to t = 0.0 with its configured seed.
    void reset();

    /// Resets the current scenario and overrides its random seed.
    void resetWithSeed(std::uint64_t new_seed);

    /// Advances the simulation by one deterministic tick (dt_sec). Returns false if completed.
    bool step();

    /// Runs the scenario deterministically until currentTime() >= duration_sec.
    void runToCompletion();

    /// Executes matched Radar-only, GNSS-only, and Fused runs on the current scenario.
    [[nodiscard]] FusionComparisonReport runFusionComparison() const;

    void pause() noexcept { paused_ = true; }
    void resume() noexcept { paused_ = false; }
    [[nodiscard]] bool isPaused() const noexcept { return paused_; }
    [[nodiscard]] bool isCompleted() const noexcept { return completed_; }

    void setPlaybackSpeed(double speed);
    [[nodiscard]] double playbackSpeed() const noexcept { return playback_speed_; }

    void setSensorOperational(const SensorId& sensor_id, bool operational);

    [[nodiscard]] TimestampSec currentTime() const noexcept { return clock_.currentTime(); }
    [[nodiscard]] double durationSec() const noexcept { return config_.duration_sec; }
    [[nodiscard]] const ScenarioConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<SimulatedObject>& objects() const noexcept {
        return objects_;
    }
    [[nodiscard]] const FusionEngine& fusionEngine() const noexcept { return fusion_engine_; }
    [[nodiscard]] FusionEngine& fusionEngine() noexcept { return fusion_engine_; }
    [[nodiscard]] const MetricsCollector& metricsCollector() const noexcept {
        return metrics_collector_;
    }
    [[nodiscard]] const EventBus& eventBus() const noexcept { return event_bus_; }
    [[nodiscard]] const InternalProfiler& profiler() const noexcept { return profiler_; }
    [[nodiscard]] const std::vector<Measurement>& allRecordedMeasurements() const noexcept {
        return all_recorded_measurements_;
    }

    /// Returns a thread-safe immutable snapshot of the latest engine state.
    [[nodiscard]] std::shared_ptr<const SimulationSnapshot> latestSnapshot() const;

private:
    void rebuildFromConfig();
    void updateSnapshot();

    ScenarioConfig config_;
    SimulationClock clock_;
    DeterministicRng rng_;
    EventBus event_bus_;
    InternalProfiler profiler_;
    std::vector<SimulatedObject> objects_;
    SensorScheduler sensor_scheduler_;
    FusionEngine fusion_engine_;
    MetricsCollector metrics_collector_;
    std::vector<Measurement> all_recorded_measurements_;

    bool paused_{true};
    bool completed_{false};
    double playback_speed_{1.0};

    mutable std::mutex snapshot_mutex_;
    std::shared_ptr<const SimulationSnapshot> latest_snapshot_;
};

}  // namespace aegistrack
