#pragma once

#include "aegistrack/core/types.hpp"
#include "aegistrack/filtering/istate_estimator.hpp"
#include "aegistrack/math/statistics.hpp"
#include "aegistrack/sensors/measurement.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace aegistrack {

struct TrackHistoryPoint {
    TimestampSec timestamp{0.0};
    Vector2d position{Vector2d::Zero()};
    Vector2d velocity{Vector2d::Zero()};
};

/**
 * @brief Represents one estimated object trajectory and its lifecycle metadata.
 */
class Track {
public:
    Track(TrackId id,
          TimestampSec creation_time,
          std::unique_ptr<IStateEstimator> estimator,
          const SensorId& initiating_sensor_id,
          std::size_t max_history_points = 500);

    Track(const Track& other);
    Track& operator=(const Track& other);
    Track(Track&&) noexcept = default;
    Track& operator=(Track&&) noexcept = default;
    ~Track() = default;

    /// Predicts the track state forward to target_timestamp.
    void predictTo(TimestampSec target_timestamp);

    /// Predicts the track state forward by an explicit dt.
    void predictDelta(double dt);

    /// Computes non-mutating innovation and Mahalanobis distance d^2 for a candidate measurement.
    [[nodiscard]] FilterDiagnostics evaluateMeasurement(const Measurement& measurement) const;

    /// Updates the track with an associated measurement.
    FilterDiagnostics updateWithMeasurement(const Measurement& measurement);

    /// Records a missed detection cycle on this track.
    void recordMissedDetection();

    void setStatus(TrackLifecycleStatus new_status) noexcept { status_ = new_status; }

    [[nodiscard]] TrackId id() const noexcept { return id_; }
    [[nodiscard]] TrackLifecycleStatus status() const noexcept { return status_; }
    [[nodiscard]] const Vector4d& state() const noexcept { return estimator_->state(); }
    [[nodiscard]] const Matrix4d& covariance() const noexcept { return estimator_->covariance(); }

    [[nodiscard]] Vector2d position() const noexcept {
        return estimator_->state().head<2>();
    }
    [[nodiscard]] Vector2d velocity() const noexcept {
        return estimator_->state().tail<2>();
    }
    [[nodiscard]] double speed() const noexcept {
        return velocity().norm();
    }
    [[nodiscard]] double headingRad() const noexcept {
        const Vector2d v = velocity();
        return std::atan2(v.y(), v.x());
    }
    [[nodiscard]] Matrix2d positionCovariance() const noexcept {
        return estimator_->covariance().topLeftCorner<2, 2>();
    }
    [[nodiscard]] math::CovarianceEllipse2D uncertaintyEllipse95() const {
        return math::computeCovarianceEllipse(positionCovariance(), 5.991464547);
    }

    /// Predicts future positions along current state trajectory without mutating the filter.
    [[nodiscard]] std::vector<Vector2d> predictFuturePath(double horizon_sec,
                                                          std::size_t num_steps) const;

    [[nodiscard]] TimestampSec creationTimestamp() const noexcept { return creation_time_; }
    [[nodiscard]] TimestampSec currentTimestamp() const noexcept { return current_time_; }
    [[nodiscard]] TimestampSec lastUpdateTimestamp() const noexcept { return last_update_time_; }
    [[nodiscard]] double ageSec() const noexcept { return current_time_ - creation_time_; }
    [[nodiscard]] double timeSinceLastUpdateSec() const noexcept {
        return current_time_ - last_update_time_;
    }

    [[nodiscard]] std::uint32_t hitCount() const noexcept { return hit_count_; }
    [[nodiscard]] std::uint32_t consecutiveHits() const noexcept { return consecutive_hits_; }
    [[nodiscard]] std::uint32_t missCount() const noexcept { return miss_count_; }
    [[nodiscard]] std::uint32_t consecutiveMisses() const noexcept { return consecutive_misses_; }

    [[nodiscard]] double lastNis() const noexcept { return last_nis_; }

    /**
     * @brief Sequential Probability Ratio Test (SPRT) log-likelihood ratio existence probability
     * P_exist = 1 / (1 + exp(-LLR)) in (0, 1).
     */
    [[nodiscard]] double existenceProbability() const noexcept;
    [[nodiscard]] double logLikelihoodRatio() const noexcept { return llr_score_; }

    [[nodiscard]] const std::vector<TrackHistoryPoint>& history() const noexcept {
        return history_;
    }
    [[nodiscard]] const std::vector<SensorId>& recentSensors() const noexcept {
        return recent_sensors_;
    }

private:
    void appendHistoryPoint();
    void recordSensorContribution(const SensorId& sensor_id);

    TrackId id_{0};
    TrackLifecycleStatus status_{TrackLifecycleStatus::Tentative};
    TimestampSec creation_time_{0.0};
    TimestampSec current_time_{0.0};
    TimestampSec last_update_time_{0.0};
    std::unique_ptr<IStateEstimator> estimator_;
    std::uint32_t hit_count_{1};
    std::uint32_t consecutive_hits_{1};
    std::uint32_t miss_count_{0};
    std::uint32_t consecutive_misses_{0};
    double last_nis_{0.0};
    double llr_score_{0.5};
    std::size_t max_history_points_{500};
    std::vector<TrackHistoryPoint> history_;
    std::vector<SensorId> recent_sensors_;
};

}  // namespace aegistrack
