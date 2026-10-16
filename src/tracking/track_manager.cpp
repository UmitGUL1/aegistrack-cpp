#include "aegistrack/tracking/track_manager.hpp"

#include "aegistrack/association/hungarian_associator.hpp"
#include "aegistrack/association/nearest_neighbor_associator.hpp"
#include "aegistrack/filtering/extended_kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace aegistrack {

namespace {

std::unique_ptr<IDataAssociator> makeAssociator(AssociationAlgorithmType type) {
    switch (type) {
        case AssociationAlgorithmType::NearestNeighbor:
            return std::make_unique<NearestNeighborAssociator>();
        case AssociationAlgorithmType::Hungarian:
            return std::make_unique<HungarianAssociator>();
    }
    return std::make_unique<HungarianAssociator>();
}

void validateConfig(const TrackManagerConfig& cfg) {
    if (cfg.confirmation_hits == 0) {
        throw std::invalid_argument("TrackManagerConfig: confirmation_hits must be >= 1");
    }
    if (!std::isfinite(cfg.coasting_threshold_sec) || cfg.coasting_threshold_sec <= 0.0 ||
        !std::isfinite(cfg.tentative_max_coast_sec) || cfg.tentative_max_coast_sec <= 0.0 ||
        !std::isfinite(cfg.max_time_without_observation_sec) ||
        cfg.max_time_without_observation_sec <= 0.0) {
        throw std::invalid_argument("TrackManagerConfig: lifecycle timeouts must be finite and > 0");
    }
    if (!std::isfinite(cfg.gate_threshold_2d) || cfg.gate_threshold_2d <= 0.0) {
        throw std::invalid_argument("TrackManagerConfig: gate_threshold_2d must be finite and > 0");
    }
    if (!std::isfinite(cfg.nominal_revisit_interval_sec) ||
        cfg.nominal_revisit_interval_sec <= 0.0 ||
        !std::isfinite(cfg.duplicate_initiation_radius_m) ||
        cfg.duplicate_initiation_radius_m < 0.0) {
        throw std::invalid_argument("TrackManagerConfig: invalid revisit interval or duplicate radius");
    }
    if (!std::isfinite(cfg.initial_velocity_std_m_s) || cfg.initial_velocity_std_m_s <= 0.0 ||
        !std::isfinite(cfg.process_noise_accel_std) || cfg.process_noise_accel_std < 0.0) {
        throw std::invalid_argument("TrackManagerConfig: invalid filter noise parameters");
    }
}

}  // namespace

TrackManager::TrackManager(TrackManagerConfig config,
                           std::unique_ptr<IDataAssociator> associator,
                           EventBus* event_bus)
    : config_(config),
      associator_(std::move(associator)),
      event_bus_(event_bus) {
    validateConfig(config_);
    if (!associator_) {
        associator_ = makeAssociator(config_.association_algorithm);
    }
}

void TrackManager::setConfig(const TrackManagerConfig& config) {
    validateConfig(config);
    const bool alg_changed = (config.association_algorithm != config_.association_algorithm);
    config_ = config;
    if (alg_changed) {
        associator_ = makeAssociator(config_.association_algorithm);
    }
}

void TrackManager::setAssociator(std::unique_ptr<IDataAssociator> associator) {
    if (!associator) {
        throw std::invalid_argument("TrackManager::setAssociator requires non-null associator");
    }
    associator_ = std::move(associator);
}

void TrackManager::advanceTime(TimestampSec target_time) {
    if (!std::isfinite(target_time) || target_time + 1e-9 < current_time_) {
        throw std::invalid_argument("TrackManager::advanceTime requires monotonic target_time");
    }
    for (auto& track : tracks_) {
        if (track.status() != TrackLifecycleStatus::Deleted &&
            target_time > track.currentTimestamp() + 1e-9) {
            track.predictTo(target_time);
        }
    }
    current_time_ = std::max(current_time_, target_time);
    TrackStepSummary dummy_summary{};
    evaluateLifecycleTransitions(current_time_, dummy_summary);
    pruneDeletedTracks();
}

TrackStepSummary TrackManager::processMeasurements(std::span<const Measurement> measurements,
                                                   TimestampSec current_time) {
    if (!std::isfinite(current_time) || current_time + 1e-9 < current_time_) {
        throw std::invalid_argument(
            "TrackManager::processMeasurements requires monotonic current_time");
    }

    TrackStepSummary total_summary{};
    total_summary.measurements_received = measurements.size();

    if (measurements.empty()) {
        advanceTime(current_time);
        return total_summary;
    }

    // Copy and sort measurements chronologically by capture_timestamp, then by sensor_id
    std::vector<Measurement> sorted_meas(measurements.begin(), measurements.end());
    std::stable_sort(sorted_meas.begin(),
                     sorted_meas.end(),
                     [](const Measurement& a, const Measurement& b) {
                         if (a.capture_timestamp != b.capture_timestamp) {
                             return a.capture_timestamp < b.capture_timestamp;
                         }
                         return a.sensor_id < b.sensor_id;
                     });

    // Partition into single-sensor scans so simultaneous multi-sensor observations update
    // tracks sequentially rather than competing within the same 1-to-1 assignment matrix.
    std::size_t idx = 0;
    while (idx < sorted_meas.size()) {
        const TimestampSec scan_t = sorted_meas[idx].capture_timestamp;
        const SensorId& scan_sensor = sorted_meas[idx].sensor_id;
        std::size_t end_idx = idx + 1;
        while (end_idx < sorted_meas.size() &&
               std::abs(sorted_meas[end_idx].capture_timestamp - scan_t) <= 1e-6 &&
               sorted_meas[end_idx].sensor_id == scan_sensor) {
            ++end_idx;
        }

        if (scan_t + 1e-9 >= current_time_) {
            const std::span<const Measurement> scan_span(sorted_meas.data() + idx, end_idx - idx);
            const TrackStepSummary scan_summary = processSingleSensorScan(scan_span, scan_t);
            total_summary.measurements_associated += scan_summary.measurements_associated;
            total_summary.new_tracks_created += scan_summary.new_tracks_created;
            total_summary.tracks_confirmed += scan_summary.tracks_confirmed;
            total_summary.tracks_coasting += scan_summary.tracks_coasting;
            total_summary.tracks_deleted += scan_summary.tracks_deleted;
        }
        idx = end_idx;
    }

    if (current_time > current_time_ + 1e-9) {
        for (auto& track : tracks_) {
            if (track.status() != TrackLifecycleStatus::Deleted &&
                current_time > track.currentTimestamp() + 1e-9) {
                track.predictTo(current_time);
            }
        }
        current_time_ = current_time;
    }

    evaluateLifecycleTransitions(current_time_, total_summary);
    pruneDeletedTracks();
    return total_summary;
}

TrackStepSummary TrackManager::processSingleSensorScan(std::span<const Measurement> scan,
                                                       TimestampSec scan_time) {
    TrackStepSummary summary{};
    current_time_ = std::max(current_time_, scan_time);

    for (auto& track : tracks_) {
        if (track.status() != TrackLifecycleStatus::Deleted &&
            scan_time > track.currentTimestamp() + 1e-9) {
            track.predictTo(scan_time);
        }
    }

    const AssociationResult assoc =
        associator_->associate(tracks_, scan, config_.gate_threshold_2d);

    for (const auto& match : assoc.matches) {
        Track& track = tracks_[match.track_index];
        const Measurement& meas = scan[match.measurement_index];
        const TrackLifecycleStatus prev_status = track.status();

        track.updateWithMeasurement(meas);
        last_miss_check_time_[track.id()] = scan_time;
        ++summary.measurements_associated;

        if (prev_status == TrackLifecycleStatus::Tentative &&
            track.hitCount() >= config_.confirmation_hits &&
            track.consecutiveHits() >= config_.confirmation_hits) {
            track.setStatus(TrackLifecycleStatus::Confirmed);
            ++summary.tracks_confirmed;
            if (event_bus_ != nullptr) {
                event_bus_->publish(SystemEvent{
                    0,
                    scan_time,
                    SystemEventType::TrackConfirmed,
                    track.id(),
                    meas.sensor_id,
                    "Track #" + std::to_string(track.id()) + " confirmed after " +
                        std::to_string(track.hitCount()) + " hits"});
            }
        } else if (prev_status == TrackLifecycleStatus::Coasting) {
            track.setStatus(TrackLifecycleStatus::Confirmed);
            if (event_bus_ != nullptr) {
                event_bus_->publish(SystemEvent{
                    0,
                    scan_time,
                    SystemEventType::TrackRecovered,
                    track.id(),
                    meas.sensor_id,
                    "Track #" + std::to_string(track.id()) + " recovered from coasting"});
            }
        }
    }

    // Initiate tentative tracks from unmatched measurements that are not close duplicates
    for (const std::size_t meas_idx : assoc.unassigned_measurement_indices) {
        const Measurement& unmatched = scan[meas_idx];
        if (!unmatched.isConsistent() || unmatched.sensor_type == SensorType::BearingOnly) {
            continue;
        }
        if (shouldSuppressDuplicateInitiation(unmatched)) {
            continue;
        }

        Track new_track = createTentativeTrack(unmatched);
        last_miss_check_time_[new_track.id()] = scan_time;
        const TrackId created_id = new_track.id();
        if (config_.confirmation_hits <= 1) {
            new_track.setStatus(TrackLifecycleStatus::Confirmed);
            ++summary.tracks_confirmed;
        }
        tracks_.push_back(std::move(new_track));
        ++summary.new_tracks_created;
        ++total_tracks_created_;

        if (event_bus_ != nullptr) {
            event_bus_->publish(SystemEvent{
                0,
                scan_time,
                SystemEventType::TrackCreated,
                created_id,
                unmatched.sensor_id,
                "Initiated tentative Track #" + std::to_string(created_id) + " from sensor '" +
                    unmatched.sensor_id + "'"});
        }
    }

    return summary;
}

bool TrackManager::shouldSuppressDuplicateInitiation(const Measurement& unmatched_meas) const {
    const Vector2d meas_pos = unmatched_meas.toCartesianPosition();
    for (const auto& track : tracks_) {
        if (track.status() == TrackLifecycleStatus::Deleted) {
            continue;
        }
        const double dist = (track.position() - meas_pos).norm();
        if (dist <= config_.duplicate_initiation_radius_m) {
            return true;
        }
        try {
            const FilterDiagnostics diag = track.evaluateMeasurement(unmatched_meas);
            if (diag.valid && diag.nis <= config_.gate_threshold_2d * 2.25) {
                return true;
            }
        } catch (const std::exception&) {
            continue;
        }
    }
    return false;
}

Track TrackManager::createTentativeTrack(const Measurement& meas) {
    const Vector2d init_pos = meas.toCartesianPosition();
    const Matrix2d init_pos_cov = meas.toCartesianCovariance();

    Vector4d x0 = Vector4d::Zero();
    x0(0) = init_pos.x();
    x0(1) = init_pos.y();

    const double v_var = config_.initial_velocity_std_m_s * config_.initial_velocity_std_m_s;
    Matrix4d P0 = Matrix4d::Zero();
    P0.topLeftCorner<2, 2>() = init_pos_cov;
    P0(2, 2) = v_var;
    P0(3, 3) = v_var;

    auto estimator = std::make_unique<ExtendedKalmanFilter>(
        x0, P0, config_.process_noise_accel_std);

    const TrackId track_id = next_track_id_++;
    return Track(track_id, meas.capture_timestamp, std::move(estimator), meas.sensor_id);
}

void TrackManager::evaluateLifecycleTransitions(TimestampSec current_time,
                                                TrackStepSummary& summary) {
    for (auto& track : tracks_) {
        if (track.status() == TrackLifecycleStatus::Deleted) {
            continue;
        }

        const double dt_unseen = current_time - track.lastUpdateTimestamp();
        TimestampSec& last_check = last_miss_check_time_[track.id()];
        while (current_time - last_check >= config_.nominal_revisit_interval_sec - 1e-9 &&
               dt_unseen >= config_.nominal_revisit_interval_sec - 1e-9) {
            track.recordMissedDetection();
            last_check += config_.nominal_revisit_interval_sec;
        }

        if (track.status() == TrackLifecycleStatus::Tentative) {
            if (dt_unseen > config_.tentative_max_coast_sec ||
                track.consecutiveMisses() > config_.max_tentative_misses) {
                track.setStatus(TrackLifecycleStatus::Deleted);
                ++summary.tracks_deleted;
                ++total_tracks_deleted_;
                if (event_bus_ != nullptr) {
                    event_bus_->publish(SystemEvent{
                        0,
                        current_time,
                        SystemEventType::TrackDeleted,
                        track.id(),
                        "",
                        "Deleted unconfirmed tentative Track #" + std::to_string(track.id())});
                }
            }
        } else if (track.status() == TrackLifecycleStatus::Confirmed) {
            if (dt_unseen > config_.max_time_without_observation_sec ||
                track.consecutiveMisses() > config_.max_missed_updates) {
                track.setStatus(TrackLifecycleStatus::Deleted);
                ++summary.tracks_deleted;
                ++total_tracks_deleted_;
                if (event_bus_ != nullptr) {
                    event_bus_->publish(SystemEvent{
                        0,
                        current_time,
                        SystemEventType::TrackDeleted,
                        track.id(),
                        "",
                        "Deleted expired Track #" + std::to_string(track.id())});
                }
            } else if (dt_unseen > config_.coasting_threshold_sec) {
                track.setStatus(TrackLifecycleStatus::Coasting);
                ++summary.tracks_coasting;
                if (event_bus_ != nullptr) {
                    event_bus_->publish(SystemEvent{
                        0,
                        current_time,
                        SystemEventType::TrackCoasting,
                        track.id(),
                        "",
                        "Track #" + std::to_string(track.id()) + " entered coasting mode"});
                }
            }
        } else if (track.status() == TrackLifecycleStatus::Coasting) {
            if (dt_unseen > config_.max_time_without_observation_sec ||
                track.consecutiveMisses() > config_.max_missed_updates) {
                track.setStatus(TrackLifecycleStatus::Deleted);
                ++summary.tracks_deleted;
                ++total_tracks_deleted_;
                if (event_bus_ != nullptr) {
                    event_bus_->publish(SystemEvent{
                        0,
                        current_time,
                        SystemEventType::TrackDeleted,
                        track.id(),
                        "",
                        "Deleted coasting Track #" + std::to_string(track.id()) +
                            " after observation timeout"});
                }
            }
        }
    }
}

void TrackManager::pruneDeletedTracks() {
    constexpr std::size_t kMaxArchivedTracks = 200;
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        if (it->status() == TrackLifecycleStatus::Deleted) {
            last_miss_check_time_.erase(it->id());
            if (archived_tracks_.size() >= kMaxArchivedTracks) {
                archived_tracks_.erase(archived_tracks_.begin());
            }
            archived_tracks_.push_back(std::move(*it));
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

std::size_t TrackManager::confirmedTrackCount() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(tracks_.begin(), tracks_.end(), [](const Track& t) {
            return t.status() == TrackLifecycleStatus::Confirmed;
        }));
}

std::size_t TrackManager::tentativeTrackCount() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(tracks_.begin(), tracks_.end(), [](const Track& t) {
            return t.status() == TrackLifecycleStatus::Tentative;
        }));
}

std::size_t TrackManager::coastingTrackCount() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(tracks_.begin(), tracks_.end(), [](const Track& t) {
            return t.status() == TrackLifecycleStatus::Coasting;
        }));
}

void TrackManager::reset() {
    tracks_.clear();
    archived_tracks_.clear();
    last_miss_check_time_.clear();
    next_track_id_ = 1;
    current_time_ = 0.0;
    total_tracks_created_ = 0;
    total_tracks_deleted_ = 0;
}

}  // namespace aegistrack
