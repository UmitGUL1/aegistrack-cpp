#include "aegistrack/association/hungarian_associator.hpp"
#include "aegistrack/association/nearest_neighbor_associator.hpp"
#include "aegistrack/filtering/extended_kalman_filter.hpp"
#include "aegistrack/tracking/track.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace aegistrack::testing {

namespace {

Track makeTrack(TrackId id, const Vector4d& state, double pos_var = 16.0) {
    Matrix4d P = Matrix4d::Identity() * pos_var;
    auto ekf = std::make_unique<ExtendedKalmanFilter>(state, P, 1.0);
    return Track(id, 0.0, std::move(ekf), "gnss");
}

Measurement makeGnssMeas(std::uint64_t id, double px, double py, double var = 9.0) {
    Measurement m;
    m.measurement_id = id;
    m.sensor_id = "gnss";
    m.sensor_type = SensorType::Gnss;
    m.capture_timestamp = 1.0;
    m.arrival_timestamp = 1.0;
    m.z = Vector2d{px, py};
    m.R = Matrix2d::Identity() * var;
    m.is_valid = true;
    return m;
}

}  // namespace

TEST(AssociationTest, OneTrackOneMeasurementAssociatesWithinGate) {
    std::vector<Track> tracks{makeTrack(1, Vector4d{100.0, 200.0, 0.0, 0.0})};
    std::vector<Measurement> meas{makeGnssMeas(1, 102.0, 198.5)};

    HungarianAssociator assoc;
    const auto res = assoc.associate(tracks, meas, 9.21034);

    ASSERT_EQ(res.matches.size(), 1U);
    EXPECT_EQ(res.matches[0].track_index, 0U);
    EXPECT_EQ(res.matches[0].measurement_index, 0U);
    EXPECT_TRUE(res.unassigned_track_indices.empty());
    EXPECT_TRUE(res.unassigned_measurement_indices.empty());
}

TEST(AssociationTest, DistantMeasurementsRejectedByMahalanobisGate) {
    std::vector<Track> tracks{makeTrack(1, Vector4d{0.0, 0.0, 0.0, 0.0}, 4.0)};
    std::vector<Measurement> meas{makeGnssMeas(1, 200.0, 200.0, 4.0)};

    HungarianAssociator assoc;
    const auto res = assoc.associate(tracks, meas, 9.21034);

    EXPECT_TRUE(res.matches.empty());
    ASSERT_EQ(res.unassigned_track_indices.size(), 1U);
    ASSERT_EQ(res.unassigned_measurement_indices.size(), 1U);
}

TEST(AssociationTest, MissingAndUnmatchedDetectionsHandledCleanly) {
    std::vector<Track> tracks{
        makeTrack(1, Vector4d{0.0, 0.0, 0.0, 0.0}),
        makeTrack(2, Vector4d{500.0, 500.0, 0.0, 0.0})
    };
    std::vector<Measurement> meas{
        makeGnssMeas(1, 1.0, -1.0),
        makeGnssMeas(2, 1000.0, 1000.0)
    };

    HungarianAssociator assoc;
    const auto res = assoc.associate(tracks, meas, 9.21034);

    ASSERT_EQ(res.matches.size(), 1U);
    EXPECT_EQ(res.matches[0].track_index, 0U);
    EXPECT_EQ(res.matches[0].measurement_index, 0U);
    ASSERT_EQ(res.unassigned_track_indices.size(), 1U);
    EXPECT_EQ(res.unassigned_track_indices[0], 1U);
    ASSERT_EQ(res.unassigned_measurement_indices.size(), 1U);
    EXPECT_EQ(res.unassigned_measurement_indices[0], 1U);
}

TEST(AssociationTest, HungarianResolvesContentionOptimallyComparedToGreedyNearestNeighbor) {
    // Track 0 is at (0, 0); Track 1 is at (6, 0).
    // Meas 0 is at (4, 0) [dist^2 to T0 = 16, dist^2 to T1 = 4].
    // Meas 1 is at (-3, 0) [dist^2 to T0 = 9, dist^2 to T1 = 81 (outside gate)].
    // Greedy NN picks (T1, M0) or if T0 is closer to M0 in another setup:
    // Suppose T0 is at (0,0), T1 is at (3.5, 0).
    // M0 is at (1.0, 0) -> d(T0, M0)^2 = 1.0, d(T1, M0)^2 = 6.25.
    // M1 is at (-2.0, 0) -> d(T0, M1)^2 = 4.0, d(T1, M1)^2 = 30.25 (out of gate if S=1!).
    // With S = 1 (pos_var = 0.5, meas_var = 0.5) and gate = 9.21:
    // Greedy NN assigns (T0 -> M0) with cost 1.0, leaving T1 unable to match M1 (cost 30.25 > 9.21)!
    // Hungarian assigns (T0 -> M1, T1 -> M0) with total cost 4.0 + 6.25 = 10.25, matching BOTH tracks!
    std::vector<Track> tracks{
        makeTrack(1, Vector4d{0.0, 0.0, 0.0, 0.0}, 0.5),
        makeTrack(2, Vector4d{3.5, 0.0, 0.0, 0.0}, 0.5)
    };
    std::vector<Measurement> meas{
        makeGnssMeas(1, 1.0, 0.0, 0.5),
        makeGnssMeas(2, -2.0, 0.0, 0.5)
    };

    NearestNeighborAssociator nn;
    HungarianAssociator hungarian;

    const auto nn_res = nn.associate(tracks, meas, 9.21034);
    const auto hun_res = hungarian.associate(tracks, meas, 9.21034);

    // Greedy NN only matches 1 track because T0 steals M0
    EXPECT_EQ(nn_res.matches.size(), 1U);
    // Global Hungarian matches both tracks within the gate!
    ASSERT_EQ(hun_res.matches.size(), 2U);
    EXPECT_EQ(hun_res.matches[0].track_index, 0U);
    EXPECT_EQ(hun_res.matches[0].measurement_index, 1U);
    EXPECT_EQ(hun_res.matches[1].track_index, 1U);
    EXPECT_EQ(hun_res.matches[1].measurement_index, 0U);
}

TEST(AssociationTest, DifferentCovarianceMagnitudesAffectMahalanobisGating) {
    // Tight covariance track rejects a 15m offset, whereas high-uncertainty coasting track accepts it
    std::vector<Track> tracks{
        makeTrack(1, Vector4d{0.0, 0.0, 0.0, 0.0}, 1.0),
        makeTrack(2, Vector4d{0.0, 100.0, 0.0, 0.0}, 100.0)
    };
    std::vector<Measurement> meas{
        makeGnssMeas(1, 15.0, 0.0, 1.0),
        makeGnssMeas(2, 15.0, 100.0, 1.0)
    };

    HungarianAssociator assoc;
    const auto res = assoc.associate(tracks, meas, 9.21034);

    ASSERT_EQ(res.matches.size(), 1U);
    EXPECT_EQ(res.matches[0].track_index, 1U);
    EXPECT_EQ(res.matches[0].measurement_index, 1U);
}

}  // namespace aegistrack::testing
