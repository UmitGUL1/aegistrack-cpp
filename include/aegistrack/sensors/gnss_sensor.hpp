#pragma once

#include "aegistrack/sensors/isensor.hpp"

#include <vector>

namespace aegistrack {

struct GnssSensorConfig {
    SensorId id{"gnss_telemetry"};
    double sampling_frequency_hz{1.0};
    double position_noise_std_x_m{5.0};
    double position_noise_std_y_m{5.0};
    double packet_loss_probability{0.05};
    double latency_sec{0.0};
    double bias_x_m{0.0};
    double bias_y_m{0.0};
    double drift_rate_x_m_s{0.0};
    double drift_rate_y_m_s{0.0};
    bool include_telemetry_identity{false};
    bool operational{true};
    std::vector<OutageWindow> outage_windows{};
    std::vector<OcclusionZone> occlusion_zones{};
};

/**
 * @brief Models onboard GNSS telemetry receivers carried by equipped simulated objects.
 *
 * Unlike ground-based radar, GNSS observations represent cooperative position telemetry
 * packets transmitted by objects where SimulatedObject::hasGnssTelemetry() is true.
 */
class GnssSensor final : public ISensor {
public:
    explicit GnssSensor(GnssSensorConfig config);

    [[nodiscard]] const SensorId& id() const noexcept override { return config_.id; }
    [[nodiscard]] SensorType type() const noexcept override { return SensorType::Gnss; }
    [[nodiscard]] double samplingIntervalSec() const noexcept override {
        return sampling_interval_sec_;
    }
    [[nodiscard]] double samplingFrequencyHz() const noexcept override {
        return config_.sampling_frequency_hz;
    }
    [[nodiscard]] Vector2d position() const noexcept override { return Vector2d::Zero(); }

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

    [[nodiscard]] const GnssSensorConfig& config() const noexcept { return config_; }

private:
    [[nodiscard]] bool isOccluded(const Vector2d& world_pos) const noexcept;

    GnssSensorConfig config_;
    double sampling_interval_sec_{1.0};
    bool manual_operational_{true};
    Matrix2d measurement_covariance_{Matrix2d::Identity()};
    SensorRuntimeStats stats_{};
};

}  // namespace aegistrack
