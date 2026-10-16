#include "aegistrack/events/event_bus.hpp"
#include "aegistrack/tracking/track_manager.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace aegistrack::testing {

namespace {

Measurement makeGnss(std::uint64_t id, TimestampSec t, double x, double y) {
    Measurement m;
    m.measurement_id = id;
    m.sensor_id = "gnss";
    m.sensor_type = SensorType::Gnss;
    m.capture_timestamp = t;
    m.arrival_timestamp = t;
    m.z = Vector2d{x, y};
    m.R = Matrix2d::Identity() * 4.0;
    m.is_valid = true;
    return m;
}

}  // namespace

TEST(TrackManagementTest, FullLifecycleCreationConfirmationCoastingRecoveryAndDeletion) {
    EventBus bus;
    TrackManagerConfig cfg;
    cfg.confirmation_hits = 3;
    cfg.coasting_threshold_sec = 1.5;
    cfg.max_time_without_observation_sec = 5.0;
    cfg.max_missed_updates = 20;
    cfg.nominal_revisit_interval_sec = 0.5;

    TrackManager tm(cfg, nullptr, &bus);

    // 1. t=0.0: Initiate Tentative track
    tm.processMeasurements(std::vector<Measurement>{makeGnss(1, 0.0, 0.0, 0.0)}, 0.0);
    ASSERT_EQ(tm.tracks().size(), 1U);
    EXPECT_EQ(tm.tracks()[0].status(), TrackLifecycleStatus::Tentative);
    EXPECT_EQ(tm.tentativeTrackCount(), 1U);

    // 2. t=0.5: Second hit -> still Tentative
    tm.processMeasurements(std::vector<Measurement>{makeGnss(2, 0.5, 10.0, 5.0)}, 0.5);
    ASSERT_EQ(tm.tracks().size(), 1U);
    EXPECT_EQ(tm.tracks()[0].status(), TrackLifecycleStatus::Tentative);

    // 3. t=1.0: Third hit -> Confirmed!
    tm.processMeasurements(std::vector<Measurement>{makeGnss(3, 1.0, 20.0, 10.0)}, 1.0);
    ASSERT_EQ(tm.tracks().size(), 1U);
    EXPECT_EQ(tm.tracks()[0].status(), TrackLifecycleStatus::Confirmed);
    EXPECT_EQ(tm.confirmedTrackCount(), 1U);

    // 4. Advance to t=3.0 without observations (dt_unseen = 2.0 > 1.5s) -> Coasting!
    tm.advanceTime(3.0);
    ASSERT_EQ(tm.tracks().size(), 1U);
    EXPECT_EQ(tm.tracks()[0].status(), TrackLifecycleStatus::Coasting);
    EXPECT_EQ(tm.coastingTrackCount(), 1U);

    // 5. t=3.5: Measurement arrives along predicted trajectory (x=70, y=35) -> Recovered to Confirmed!
    tm.processMeasurements(std::vector<Measurement>{makeGnss(4, 3.5, 70.0, 35.0)}, 3.5);
    ASSERT_EQ(tm.tracks().size(), 1U);
    EXPECT_EQ(tm.tracks()[0].status(), TrackLifecycleStatus::Confirmed);

    // 6. Advance to t=9.0 without observations (dt_unseen = 5.5 > 5.0s) -> Deleted & pruned!
    tm.advanceTime(9.0);
    EXPECT_TRUE(tm.tracks().empty());
    EXPECT_EQ(tm.archivedDeletedTracks().size(), 1U);
    EXPECT_EQ(tm.totalTracksDeleted(), 1U);
}

TEST(TrackManagementTest, TentativeTrackDeletedOnTimeoutSuppressingClutter) {
    TrackManagerConfig cfg;
    cfg.confirmation_hits = 3;
    cfg.tentative_max_coast_sec = 1.5;
    TrackManager tm(cfg);

    // Single false alarm at t=0.0
    tm.processMeasurements(std::vector<Measurement>{makeGnss(1, 0.0, 250.0, -300.0)}, 0.0);
    EXPECT_EQ(tm.tentativeTrackCount(), 1U);

    // No follow-up detections by t=2.0 -> Tentative track deleted
    tm.advanceTime(2.0);
    EXPECT_EQ(tm.tentativeTrackCount(), 0U);
    EXPECT_TRUE(tm.tracks().empty());
    EXPECT_EQ(tm.totalTracksDeleted(), 1U);
}

TEST(TrackManagementTest, SimultaneousMultiSensorMeasurementsDoNotSpawnDuplicateTracks) {
    TrackManagerConfig cfg;
    cfg.confirmation_hits = 3;
    cfg.duplicate_initiation_radius_m = 25.0;
    TrackManager tm(cfg);

    Measurement m_gnss = makeGnss(1, 0.0, 500.0, 0.0);
    Measurement m_radar;
    m_radar.measurement_id = 2;
    m_radar.sensor_id = "radar_1";
    m_radar.sensor_type = SensorType::Radar;
    m_radar.capture_timestamp = 0.0;
    m_radar.arrival_timestamp = 0.0;
    m_radar.z = Vector2d{502.0, 0.002};
    m_radar.R = Matrix2d::Identity() * 0.01;
    m_radar.R(0, 0) = 25.0;
    m_radar.sensor_position = Vector2d::Zero();
    m_radar.is_valid = true;

    tm.processMeasurements(std::vector<Measurement>{m_gnss, m_radar}, 0.0);

    // Must fuse into a single track with 2 hits rather than 2 duplicate tracks!
    ASSERT_EQ(tm.tracks().size(), 1U);
    EXPECT_EQ(tm.tracks()[0].hitCount(), 2U);
}

TEST(TrackManagementTest, RejectsZeroRevisitIntervalBeforeProcessing) {
    TrackManagerConfig cfg;
    cfg.nominal_revisit_interval_sec = 0.0;
    EXPECT_THROW(TrackManager manager(cfg), std::invalid_argument);
}

}  // namespace aegistrack::testing
