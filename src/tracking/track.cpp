#include "aegistrack/tracking/track.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aegistrack {

Track::Track(TrackId id,
             TimestampSec creation_time,
             std::unique_ptr<IStateEstimator> estimator,
             const SensorId& initiating_sensor_id,
             std::size_t max_history_points)
    : id_(id),
      status_(TrackLifecycleStatus::Tentative),
      creation_time_(creation_time),
      current_time_(creation_time),
      last_update_time_(creation_time),
      estimator_(std::move(estimator)),
      max_history_points_(max_history_points) {
    if (!estimator_) {
        throw std::invalid_argument("Track requires a non-null IStateEstimator");
    }
    if (!std::isfinite(creation_time)) {
        throw std::invalid_argument("Track requires finite creation_time");
    }
    appendHistoryPoint();
    if (!initiating_sensor_id.empty()) {
        recordSensorContribution(initiating_sensor_id);
    }
}

Track::Track(const Track& other)
    : id_(other.id_),
      status_(other.status_),
      creation_time_(other.creation_time_),
      current_time_(other.current_time_),
      last_update_time_(other.last_update_time_),
      estimator_(other.estimator_ ? other.estimator_->clone() : nullptr),
      hit_count_(other.hit_count_),
      consecutive_hits_(other.consecutive_hits_),
      miss_count_(other.miss_count_),
      consecutive_misses_(other.consecutive_misses_),
      last_nis_(other.last_nis_),
      llr_score_(other.llr_score_),
      max_history_points_(other.max_history_points_),
      history_(other.history_),
      recent_sensors_(other.recent_sensors_) {}

Track& Track::operator=(const Track& other) {
    if (this != &other) {
        id_ = other.id_;
        status_ = other.status_;
        creation_time_ = other.creation_time_;
        current_time_ = other.current_time_;
        last_update_time_ = other.last_update_time_;
        estimator_ = other.estimator_ ? other.estimator_->clone() : nullptr;
        hit_count_ = other.hit_count_;
        consecutive_hits_ = other.consecutive_hits_;
        miss_count_ = other.miss_count_;
        consecutive_misses_ = other.consecutive_misses_;
        last_nis_ = other.last_nis_;
        llr_score_ = other.llr_score_;
        max_history_points_ = other.max_history_points_;
        history_ = other.history_;
        recent_sensors_ = other.recent_sensors_;
    }
    return *this;
}

void Track::predictTo(TimestampSec target_timestamp) {
    if (!std::isfinite(target_timestamp)) {
        throw std::invalid_argument("Track::predictTo requires finite target_timestamp");
    }
    const double dt = target_timestamp - current_time_;
    if (dt < -1e-9) {
        throw std::invalid_argument("Track::predictTo cannot predict backward in time");
    }
    if (dt > 1e-9) {
        estimator_->predict(dt);
        current_time_ = target_timestamp;
    }
}

void Track::predictDelta(double dt) {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument("Track::predictDelta requires finite dt >= 0");
    }
    if (dt > 0.0) {
        estimator_->predict(dt);
        current_time_ += dt;
    }
}

FilterDiagnostics Track::evaluateMeasurement(const Measurement& measurement) const {
    return estimator_->computeInnovation(measurement);
}

FilterDiagnostics Track::updateWithMeasurement(const Measurement& measurement) {
    if (measurement.capture_timestamp > current_time_ + 1e-9) {
        predictTo(measurement.capture_timestamp);
    }

    FilterDiagnostics diag = estimator_->updateMeasurement(measurement);
    last_nis_ = diag.nis;
    last_update_time_ = std::max(last_update_time_, measurement.capture_timestamp);
    current_time_ = std::max(current_time_, measurement.capture_timestamp);

    ++hit_count_;
    ++consecutive_hits_;
    consecutive_misses_ = 0;

    // SPRT Log-Likelihood Ratio increment: Delta_L = +1.8 - 0.5 * NIS
    const double delta_llr = std::clamp(1.8 - 0.35 * diag.nis, -1.0, 1.8);
    llr_score_ = std::clamp(llr_score_ + delta_llr, -6.0, 6.0);

    appendHistoryPoint();
    recordSensorContribution(measurement.sensor_id);
    return diag;
}

void Track::recordMissedDetection() {
    ++miss_count_;
    ++consecutive_misses_;
    consecutive_hits_ = 0;
    llr_score_ = std::clamp(llr_score_ - 0.85, -6.0, 6.0);
}

double Track::existenceProbability() const noexcept {
    return 1.0 / (1.0 + std::exp(-llr_score_));
}

std::vector<Vector2d> Track::predictFuturePath(double horizon_sec,
                                               std::size_t num_steps) const {
    std::vector<Vector2d> points;
    if (num_steps == 0 || horizon_sec <= 0.0) {
        return points;
    }
    points.reserve(num_steps);
    const double step_dt = horizon_sec / static_cast<double>(num_steps);
    const Vector2d p0 = position();
    const Vector2d v0 = velocity();
    for (std::size_t i = 1; i <= num_steps; ++i) {
        const double t = static_cast<double>(i) * step_dt;
        points.emplace_back(p0 + v0 * t);
    }
    return points;
}

void Track::appendHistoryPoint() {
    if (max_history_points_ > 0 && history_.size() >= max_history_points_) {
        history_.erase(history_.begin());
    }
    history_.push_back(TrackHistoryPoint{current_time_, position(), velocity()});
}

void Track::recordSensorContribution(const SensorId& sensor_id) {
    if (sensor_id.empty()) {
        return;
    }
    constexpr std::size_t kMaxRecentSensors = 8;
    if (recent_sensors_.size() >= kMaxRecentSensors) {
        recent_sensors_.erase(recent_sensors_.begin());
    }
    recent_sensors_.push_back(sensor_id);
}

}  // namespace aegistrack
