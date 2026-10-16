#include "aegistrack/metrics/metrics_collector.hpp"

#include "aegistrack/math/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace aegistrack {

MetricsCollector::MetricsCollector(double evaluation_cutoff_distance_m,
                                   std::size_t max_history_samples)
    : evaluation_cutoff_distance_m_(evaluation_cutoff_distance_m),
      max_history_samples_(max_history_samples) {}

MetricsSnapshot MetricsCollector::evaluateStep(
    TimestampSec current_time,
    std::span<const SimulatedObject> ground_truth_objects,
    const TrackManager& track_manager,
    std::span<const Measurement> step_measurements,
    std::uint64_t total_detection_failures,
    std::uint64_t total_false_alarms,
    double step_latency_us) {
    MetricsSnapshot snap{};
    snap.timestamp = current_time;
    snap.confirmed_tracks = track_manager.confirmedTrackCount();
    snap.tentative_tracks = track_manager.tentativeTrackCount();
    snap.coasting_tracks = track_manager.coastingTrackCount();
    snap.cumulative_measurements =
        latest_snapshot_.cumulative_measurements + step_measurements.size();
    snap.cumulative_detection_failures = total_detection_failures;
    snap.cumulative_false_alarms = total_false_alarms;
    snap.cumulative_tracks_created = track_manager.totalTracksCreated();
    snap.cumulative_tracks_deleted = track_manager.totalTracksDeleted();
    snap.step_latency_us = step_latency_us;
    if (current_time > 1e-6) {
        snap.measurements_per_sec =
            static_cast<double>(snap.cumulative_measurements) / current_time;
    }

    std::vector<const SimulatedObject*> active_truth;
    std::unordered_map<ObjectId, const SimulatedObject*> truth_by_id;
    for (const auto& obj : ground_truth_objects) {
        truth_by_id[obj.id()] = &obj;
        if (obj.isActiveAt(current_time)) {
            active_truth.push_back(&obj);
        }
    }
    snap.active_ground_truth_count = active_truth.size();

    // Evaluate raw sensor measurement error on this step
    double step_raw_sq_sum = 0.0;
    std::size_t step_raw_count = 0;
    for (const auto& m : step_measurements) {
        if (m.is_clutter || !m.evaluation_ground_truth_id.has_value() ||
            m.sensor_type == SensorType::BearingOnly) {
            continue;
        }
        const auto it = truth_by_id.find(*m.evaluation_ground_truth_id);
        if (it != truth_by_id.end()) {
            const Vector2d meas_pos = m.toCartesianPosition();
            const auto capture_state = it->second->stateAt(m.capture_timestamp);
            if (!capture_state) {
                continue;
            }
            const Vector2d true_pos = capture_state->position();
            const double sq_err = (meas_pos - true_pos).squaredNorm();
            step_raw_sq_sum += sq_err;
            ++step_raw_count;
            sum_sq_raw_meas_error_ += sq_err;
            ++total_raw_meas_evaluated_;
        }
    }
    if (step_raw_count > 0) {
        snap.raw_measurement_rmse_m =
            std::sqrt(step_raw_sq_sum / static_cast<double>(step_raw_count));
    } else {
        snap.raw_measurement_rmse_m = latest_snapshot_.raw_measurement_rmse_m;
    }

    // Collect candidate tracks for evaluation (prefer Confirmed + Coasting; fallback to Tentative)
    std::vector<const Track*> eval_tracks;
    for (const auto& t : track_manager.tracks()) {
        if (t.status() == TrackLifecycleStatus::Confirmed ||
            t.status() == TrackLifecycleStatus::Coasting) {
            eval_tracks.push_back(&t);
        }
    }
    if (eval_tracks.empty()) {
        for (const auto& t : track_manager.tracks()) {
            if (t.status() == TrackLifecycleStatus::Tentative) {
                eval_tracks.push_back(&t);
            }
        }
    }

    if (!eval_tracks.empty() && !active_truth.empty()) {
        const auto n_tr = static_cast<Eigen::Index>(eval_tracks.size());
        const auto n_gt = static_cast<Eigen::Index>(active_truth.size());
        MatrixXd cost_sq(n_tr, n_gt);
        for (Eigen::Index i = 0; i < n_tr; ++i) {
            for (Eigen::Index j = 0; j < n_gt; ++j) {
                cost_sq(i, j) = (eval_tracks[static_cast<std::size_t>(i)]->position() -
                                 active_truth[static_cast<std::size_t>(j)]->state().position())
                                    .squaredNorm();
            }
        }

        const double max_sq_dist =
            evaluation_cutoff_distance_m_ * evaluation_cutoff_distance_m_;
        const auto assignment = math::HungarianSolver::solveGated(cost_sq, max_sq_dist);

        double step_pos_sq = 0.0;
        double step_vel_sq = 0.0;
        for (const auto& [tr_idx, gt_idx] : assignment.matched_pairs) {
            const Track* tr = eval_tracks[tr_idx];
            const SimulatedObject* gt = active_truth[gt_idx];
            const double pos_sq = (tr->position() - gt->state().position()).squaredNorm();
            const double vel_sq = (tr->velocity() - gt->state().velocity()).squaredNorm();
            step_pos_sq += pos_sq;
            step_vel_sq += vel_sq;
            recordErrorSample(std::sqrt(pos_sq), std::sqrt(vel_sq));
        }

        snap.matched_track_count = assignment.matched_pairs.size();
        snap.false_track_count = assignment.unassigned_rows.size();
        if (!assignment.matched_pairs.empty()) {
            const double denom = static_cast<double>(assignment.matched_pairs.size());
            snap.position_rmse_m = std::sqrt(step_pos_sq / denom);
            snap.velocity_rmse_m = std::sqrt(step_vel_sq / denom);
        }
    }

    latest_snapshot_ = snap;
    if (max_history_samples_ > 0 && history_.size() >= max_history_samples_) {
        history_.erase(history_.begin());
    }
    history_.push_back(snap);
    return snap;
}

void MetricsCollector::recordErrorSample(double pos_error_m, double vel_error_m_s) {
    sum_sq_pos_error_ += pos_error_m * pos_error_m;
    sum_sq_vel_error_ += vel_error_m_s * vel_error_m_s;
    ++total_evaluated_pairs_;

    std::size_t bin = kHistogramBinCount - 1;
    for (std::size_t i = 0; i < kHistogramBinEdgesM.size(); ++i) {
        if (pos_error_m <= kHistogramBinEdgesM[i]) {
            bin = i;
            break;
        }
    }
    ++error_histogram_[bin];
}

double MetricsCollector::cumulativePositionRmseM() const noexcept {
    if (total_evaluated_pairs_ == 0) {
        return 0.0;
    }
    return std::sqrt(sum_sq_pos_error_ / static_cast<double>(total_evaluated_pairs_));
}

double MetricsCollector::cumulativeVelocityRmseM() const noexcept {
    if (total_evaluated_pairs_ == 0) {
        return 0.0;
    }
    return std::sqrt(sum_sq_vel_error_ / static_cast<double>(total_evaluated_pairs_));
}

double MetricsCollector::cumulativeRawMeasurementRmseM() const noexcept {
    if (total_raw_meas_evaluated_ == 0) {
        return 0.0;
    }
    return std::sqrt(sum_sq_raw_meas_error_ / static_cast<double>(total_raw_meas_evaluated_));
}

void MetricsCollector::reset() {
    latest_snapshot_ = MetricsSnapshot{};
    history_.clear();
    sum_sq_pos_error_ = 0.0;
    sum_sq_vel_error_ = 0.0;
    total_evaluated_pairs_ = 0;
    sum_sq_raw_meas_error_ = 0.0;
    total_raw_meas_evaluated_ = 0;
    error_histogram_.fill(0);
}

}  // namespace aegistrack
