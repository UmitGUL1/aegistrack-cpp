#pragma once

#include "aegistrack/association/idata_associator.hpp"
#include "aegistrack/core/types.hpp"
#include "aegistrack/events/event_bus.hpp"
#include "aegistrack/tracking/track.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

namespace aegistrack {

struct TrackManagerConfig {
    /// Minimum measurement updates required to transition Tentative -> Confirmed.
    std::uint32_t confirmation_hits{3};
    /// Maximum missed revisit intervals before a Tentative track is deleted.
    std::uint32_t max_tentative_misses{2};
    /// Maximum missed revisit intervals before a Confirmed/Coasting track is deleted.
    std::uint32_t max_missed_updates{8};
    /// Elapsed time without observation (s) before a Confirmed track enters Coasting.
    double coasting_threshold_sec{1.5};
    /// Maximum elapsed time without observation (s) before a Tentative track is deleted.
    double tentative_max_coast_sec{2.2};
    /// Maximum elapsed time without observation (s) before a Coasting track is deleted.
    double max_time_without_observation_sec{6.0};
    /// Nominal revisit period (s) used to count missed cycles across asynchronous sensors.
    double nominal_revisit_interval_sec{0.6};
    /// Chi-square gate threshold for 2D measurements (9.21034 = 99% confidence in 2 DOF).
    double gate_threshold_2d{9.21034037};
    /// Initial velocity standard deviation (m/s) for newly initiated tracks.
    double initial_velocity_std_m_s{45.0};
    /// Continuous-time white acceleration process noise std sigma_a (m/s^2).
    double process_noise_accel_std{2.0};
    /// Minimum spatial separation (m) to suppress duplicate tentative track initiation.
    double duplicate_initiation_radius_m{25.0};
    /// Minimum existence probability required for standard visualization display.
    double min_visualization_confidence{0.2};
    /// Association strategy type.
    AssociationAlgorithmType association_algorithm{AssociationAlgorithmType::Hungarian};
};

struct TrackStepSummary {
    std::size_t measurements_received{0};
    std::size_t measurements_associated{0};
    std::size_t new_tracks_created{0};
    std::size_t tracks_confirmed{0};
    std::size_t tracks_coasting{0};
    std::size_t tracks_deleted{0};
};

/**
 * @brief Manages the collection of active tracks, data association, and track lifecycle state machine.
 */
class TrackManager {
public:
    explicit TrackManager(TrackManagerConfig config = {},
                          std::unique_ptr<IDataAssociator> associator = nullptr,
                          EventBus* event_bus = nullptr);

    /// Predicts all active tracks to target_time and updates time-based lifecycle transitions.
    void advanceTime(TimestampSec target_time);

    /// Processes a batch of measurements (potentially from multiple sensors/timestamps).
    TrackStepSummary processMeasurements(std::span<const Measurement> measurements,
                                         TimestampSec current_time);

    void setAssociator(std::unique_ptr<IDataAssociator> associator);
    [[nodiscard]] const IDataAssociator& associator() const noexcept { return *associator_; }

    void reset();

    [[nodiscard]] const std::vector<Track>& tracks() const noexcept { return tracks_; }
    [[nodiscard]] const std::vector<Track>& archivedDeletedTracks() const noexcept {
        return archived_tracks_;
    }

    [[nodiscard]] std::size_t confirmedTrackCount() const noexcept;
    [[nodiscard]] std::size_t tentativeTrackCount() const noexcept;
    [[nodiscard]] std::size_t coastingTrackCount() const noexcept;
    [[nodiscard]] std::uint64_t totalTracksCreated() const noexcept {
        return total_tracks_created_;
    }
    [[nodiscard]] std::uint64_t totalTracksDeleted() const noexcept {
        return total_tracks_deleted_;
    }

    [[nodiscard]] const TrackManagerConfig& config() const noexcept { return config_; }
    void setConfig(const TrackManagerConfig& config);

private:
    TrackStepSummary processSingleSensorScan(std::span<const Measurement> scan,
                                             TimestampSec scan_time);
    void evaluateLifecycleTransitions(TimestampSec current_time, TrackStepSummary& summary);
    bool shouldSuppressDuplicateInitiation(const Measurement& unmatched_meas) const;
    Track createTentativeTrack(const Measurement& meas);
    void pruneDeletedTracks();

    TrackManagerConfig config_;
    std::unique_ptr<IDataAssociator> associator_;
    EventBus* event_bus_;
    std::vector<Track> tracks_;
    std::vector<Track> archived_tracks_;
    std::unordered_map<TrackId, TimestampSec> last_miss_check_time_;
    TrackId next_track_id_{1};
    TimestampSec current_time_{0.0};
    std::uint64_t total_tracks_created_{0};
    std::uint64_t total_tracks_deleted_{0};
};

}  // namespace aegistrack
