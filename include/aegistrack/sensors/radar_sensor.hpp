#pragma once

#include "aegistrack/sensors/isensor.hpp"

#include <numbers>
#include <vector>

namespace aegistrack {

struct RadarSensorConfig {
    SensorId id{"radar_1"};
    Vector2d position{0.0, 0.0};
    double min_range_m{5.0};
    double max_range_m{5000.0};
    double boresight_rad{0.0};
    double fov_rad{2.0 * std::numbers::pi};
    double sampling_frequency_hz{2.0};
    double range_noise_std_m{8.0};
    double bearing_noise_std_rad{0.015};
    double detection_probability{0.95};
    double mean_false_alarms_per_scan{0.2};
    double latency_sec{0.0};
    double range_bias_m{0.0};
    double bearing_bias_rad{0.0};
    bool operational{true};
    std::vector<OutageWindow> outage_windows{};
    std::vector<OcclusionZone> occlusion_zones{};
};

/**
 * @brief Ground-based 2D polar Radar sensor measuring [range (m), bearing (rad)]^T.
 */
class RadarSensor final : public ISensor {
public:
    explicit RadarSensor(RadarSensorConfig config);

    [[nodiscard]] const SensorId& id() const noexcept override { return config_.id; }
    [[nodiscard]] SensorType type() const noexcept override { return SensorType::Radar; }
    [[nodiscard]] double samplingIntervalSec() const noexcept override {
        return sampling_interval_sec_;
    }
    [[nodiscard]] double samplingFrequencyHz() const noexcept override {
        return config_.sampling_frequency_hz;
    }
    [[nodiscard]] Vector2d position() const noexcept override { return config_.position; }

    [[nodiscard]] bool isOperationalAt(TimestampSec timestamp) const noexcept override;
    void setManualOperational(bool operational) noexcept override {
        manual_operational_ = operational;
    }
    [[nodiscard]] bool isManuallyOperational() const noexcept override {
        return manual_operational_;
    }

    [[nodiscard]] std::vector<Measurement> generateMeasurements(
        TimestampSec current_time,
        std::span<const SimulatedObject> objects,
        DeterministicRng& rng,
        std::uint64_t& next_measurement_id) override;

    [[nodiscard]] SensorRuntimeStats runtimeStats() const noexcept override { return stats_; }
    void reset() noexcept override { stats_ = SensorRuntimeStats{}; }

    [[nodiscard]] const RadarSensorConfig& config() const noexcept { return config_; }
    [[nodiscard]] bool isWithinCoverage(const Vector2d& world_pos) const;

private:
    RadarSensorConfig config_;
    double sampling_interval_sec_{0.5};
    bool manual_operational_{true};
    Matrix2d measurement_covariance_{Matrix2d::Identity()};
    SensorRuntimeStats stats_{};
};

}  // namespace aegistrack
