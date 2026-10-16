#include "aegistrack/sensors/gnss_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aegistrack {

GnssSensor::GnssSensor(GnssSensorConfig config)
    : config_(std::move(config)), manual_operational_(config_.operational) {
    if (config_.id.empty()) {
        throw std::invalid_argument("GnssSensor: sensor ID must not be empty");
    }
    if (!std::isfinite(config_.sampling_frequency_hz) || config_.sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("GnssSensor: sampling_frequency_hz must be finite and > 0");
    }
    if (!std::isfinite(config_.position_noise_std_x_m) || config_.position_noise_std_x_m < 0.0 ||
        !std::isfinite(config_.position_noise_std_y_m) || config_.position_noise_std_y_m < 0.0) {
        throw std::invalid_argument("GnssSensor: position noise standard deviations must be >= 0");
    }
    if (!std::isfinite(config_.packet_loss_probability) || config_.packet_loss_probability < 0.0 ||
        config_.packet_loss_probability > 1.0) {
        throw std::invalid_argument("GnssSensor: packet_loss_probability must be in [0, 1]");
    }
    if (!std::isfinite(config_.latency_sec) || config_.latency_sec < 0.0) {
        throw std::invalid_argument("GnssSensor: latency_sec must be >= 0");
    }

    sampling_interval_sec_ = 1.0 / config_.sampling_frequency_hz;
    const double var_x =
        std::max(1e-6, config_.position_noise_std_x_m * config_.position_noise_std_x_m);
    const double var_y =
        std::max(1e-6, config_.position_noise_std_y_m * config_.position_noise_std_y_m);
    measurement_covariance_.setZero();
    measurement_covariance_(0, 0) = var_x;
    measurement_covariance_(1, 1) = var_y;
}

bool GnssSensor::isOperationalAt(TimestampSec timestamp) const noexcept {
    if (!manual_operational_) {
        return false;
    }
    for (const auto& window : config_.outage_windows) {
        if (timestamp + 1e-9 >= window.start_time && timestamp - 1e-9 <= window.end_time) {
            return false;
        }
    }
    return true;
}

bool GnssSensor::isOccluded(const Vector2d& world_pos) const noexcept {
    for (const auto& zone : config_.occlusion_zones) {
        const double dist =
            std::hypot(world_pos.x() - zone.center_x, world_pos.y() - zone.center_y);
        if (dist <= zone.radius_m) {
            return true;
        }
    }
    return false;
}

std::vector<Measurement> GnssSensor::generateMeasurements(
    TimestampSec current_time,
    std::span<const SimulatedObject> objects,
    DeterministicRng& rng,
    std::uint64_t& next_measurement_id) {
    std::vector<Measurement> measurements;
    if (!isOperationalAt(current_time)) {
        return measurements;
    }

    ++stats_.total_scans;

    for (const auto& obj : objects) {
        if (!obj.isActiveAt(current_time) || !obj.hasGnssTelemetry()) {
            continue;
        }
        const auto sampled_state = obj.stateAt(current_time);
        if (!sampled_state) {
            continue;
        }
        const Vector2d true_pos = sampled_state->position();
        if (isOccluded(true_pos)) {
            ++stats_.detection_failures;
            continue;
        }

        if (rng.bernoulli(config_.packet_loss_probability)) {
            ++stats_.detection_failures;
            continue;
        }

        const double total_bias_x = config_.bias_x_m + config_.drift_rate_x_m_s * current_time;
        const double total_bias_y = config_.bias_y_m + config_.drift_rate_y_m_s * current_time;

        const double meas_x =
            true_pos.x() + total_bias_x + rng.normal(0.0, config_.position_noise_std_x_m);
        const double meas_y =
            true_pos.y() + total_bias_y + rng.normal(0.0, config_.position_noise_std_y_m);

        Measurement m;
        m.measurement_id = next_measurement_id++;
        m.sensor_id = config_.id;
        m.sensor_type = SensorType::Gnss;
        m.capture_timestamp = current_time;
        m.arrival_timestamp = current_time + config_.latency_sec;
        m.z = Vector2d{meas_x, meas_y};
        m.R = measurement_covariance_;
        m.sensor_position = Vector2d::Zero();
        m.is_valid = true;
        if (config_.include_telemetry_identity) {
            m.telemetry_source_id = obj.id();
        }
        m.evaluation_ground_truth_id = obj.id();
        m.is_clutter = false;

        measurements.push_back(std::move(m));
        ++stats_.measurements_generated;
    }

    return measurements;
}

}  // namespace aegistrack
