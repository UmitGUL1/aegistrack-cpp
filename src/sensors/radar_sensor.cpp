#include "aegistrack/sensors/radar_sensor.hpp"

#include "aegistrack/math/angles.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aegistrack {

RadarSensor::RadarSensor(RadarSensorConfig config)
    : config_(std::move(config)), manual_operational_(config_.operational) {
    if (config_.id.empty()) {
        throw std::invalid_argument("RadarSensor: sensor ID must not be empty");
    }
    if (!config_.position.allFinite()) {
        throw std::invalid_argument("RadarSensor: sensor position must be finite");
    }
    if (!std::isfinite(config_.sampling_frequency_hz) || config_.sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("RadarSensor: sampling_frequency_hz must be finite and > 0");
    }
    if (!std::isfinite(config_.min_range_m) || !std::isfinite(config_.max_range_m) ||
        config_.min_range_m < 0.0 || config_.max_range_m <= config_.min_range_m) {
        throw std::invalid_argument("RadarSensor: invalid range bounds [min_range_m, max_range_m]");
    }
    if (!std::isfinite(config_.fov_rad) || config_.fov_rad <= 0.0 ||
        config_.fov_rad > math::kTwoPi + 1e-6) {
        throw std::invalid_argument("RadarSensor: fov_rad must be in (0, 2*pi]");
    }
    if (!std::isfinite(config_.range_noise_std_m) || config_.range_noise_std_m < 0.0 ||
        !std::isfinite(config_.bearing_noise_std_rad) || config_.bearing_noise_std_rad < 0.0) {
        throw std::invalid_argument("RadarSensor: noise standard deviations must be >= 0");
    }
    if (!std::isfinite(config_.detection_probability) || config_.detection_probability < 0.0 ||
        config_.detection_probability > 1.0) {
        throw std::invalid_argument("RadarSensor: detection_probability must be in [0, 1]");
    }
    if (!std::isfinite(config_.mean_false_alarms_per_scan) ||
        config_.mean_false_alarms_per_scan < 0.0) {
        throw std::invalid_argument("RadarSensor: mean_false_alarms_per_scan must be >= 0");
    }
    if (!std::isfinite(config_.latency_sec) || config_.latency_sec < 0.0) {
        throw std::invalid_argument("RadarSensor: latency_sec must be >= 0");
    }

    sampling_interval_sec_ = 1.0 / config_.sampling_frequency_hz;
    const double r_var = std::max(1e-6, config_.range_noise_std_m * config_.range_noise_std_m);
    const double theta_var =
        std::max(1e-8, config_.bearing_noise_std_rad * config_.bearing_noise_std_rad);
    measurement_covariance_.setZero();
    measurement_covariance_(0, 0) = r_var;
    measurement_covariance_(1, 1) = theta_var;
}

bool RadarSensor::isOperationalAt(TimestampSec timestamp) const noexcept {
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

bool RadarSensor::isWithinCoverage(const Vector2d& world_pos) const {
    const Vector2d rel = world_pos - config_.position;
    const double range = rel.norm();
    if (range < config_.min_range_m || range > config_.max_range_m) {
        return false;
    }
    if (config_.fov_rad < math::kTwoPi - 1e-6) {
        const double bearing = std::atan2(rel.y(), rel.x());
        const double diff = std::abs(math::angleDifference(bearing, config_.boresight_rad));
        if (diff > 0.5 * config_.fov_rad) {
            return false;
        }
    }
    for (const auto& zone : config_.occlusion_zones) {
        const double dist_to_zone =
            std::hypot(world_pos.x() - zone.center_x, world_pos.y() - zone.center_y);
        if (dist_to_zone <= zone.radius_m) {
            return false;
        }
    }
    return true;
}

std::vector<Measurement> RadarSensor::generateMeasurements(
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
        if (!obj.isActiveAt(current_time)) {
            continue;
        }
        const auto sampled_state = obj.stateAt(current_time);
        if (!sampled_state) {
            continue;
        }
        const Vector2d pos = sampled_state->position();
        if (!isWithinCoverage(pos)) {
            continue;
        }

        if (!rng.bernoulli(config_.detection_probability)) {
            ++stats_.detection_failures;
            continue;
        }

        const Vector2d rel = pos - config_.position;
        const double true_range = rel.norm();
        const double true_bearing = std::atan2(rel.y(), rel.x());

        const double noisy_range =
            std::max(0.1,
                     true_range + config_.range_bias_m +
                         rng.normal(0.0, config_.range_noise_std_m));
        const double noisy_bearing = math::normalizeAnglePi(
            true_bearing + config_.bearing_bias_rad +
            rng.normal(0.0, config_.bearing_noise_std_rad));

        Measurement m;
        m.measurement_id = next_measurement_id++;
        m.sensor_id = config_.id;
        m.sensor_type = SensorType::Radar;
        m.capture_timestamp = current_time;
        m.arrival_timestamp = current_time + config_.latency_sec;
        m.z = Vector2d{noisy_range, noisy_bearing};
        m.R = measurement_covariance_;
        m.sensor_position = config_.position;
        m.is_valid = true;
        m.evaluation_ground_truth_id = obj.id();
        m.is_clutter = false;

        measurements.push_back(std::move(m));
        ++stats_.measurements_generated;
    }

    // Generate Poisson-distributed false radar detections (clutter) within radar coverage
    const std::uint32_t num_clutter = rng.poisson(config_.mean_false_alarms_per_scan);
    for (std::uint32_t c = 0; c < num_clutter; ++c) {
        const double clutter_range = rng.uniform(config_.min_range_m, config_.max_range_m);
        const double half_fov = 0.5 * config_.fov_rad;
        const double clutter_bearing = math::normalizeAnglePi(
            config_.boresight_rad + rng.uniform(-half_fov, half_fov));

        Measurement clutter_meas;
        clutter_meas.measurement_id = next_measurement_id++;
        clutter_meas.sensor_id = config_.id;
        clutter_meas.sensor_type = SensorType::Radar;
        clutter_meas.capture_timestamp = current_time;
        clutter_meas.arrival_timestamp = current_time + config_.latency_sec;
        clutter_meas.z = Vector2d{clutter_range, clutter_bearing};
        clutter_meas.R = measurement_covariance_;
        clutter_meas.sensor_position = config_.position;
        clutter_meas.is_valid = true;
        clutter_meas.evaluation_ground_truth_id = std::nullopt;
        clutter_meas.is_clutter = true;

        measurements.push_back(std::move(clutter_meas));
        ++stats_.measurements_generated;
        ++stats_.false_alarms_generated;
    }

    return measurements;
}

}  // namespace aegistrack
