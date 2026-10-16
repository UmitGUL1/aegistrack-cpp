#include "aegistrack/core/random.hpp"
#include "aegistrack/math/angles.hpp"
#include "aegistrack/metrics/metrics_collector.hpp"
#include "aegistrack/sensors/gnss_sensor.hpp"
#include "aegistrack/sensors/radar_sensor.hpp"
#include "aegistrack/sensors/sensor_scheduler.hpp"
#include "aegistrack/simulation/simulated_object.hpp"
#include "aegistrack/tracking/track_manager.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace aegistrack::testing {

TEST(SensorTest, RadarGeometryAndNoiseFreeMeasurementsAreExact) {
    RadarSensorConfig cfg;
    cfg.id = "radar_test";
    cfg.position = Vector2d{100.0, 200.0};
    cfg.min_range_m = 1.0;
    cfg.max_range_m = 2000.0;
    cfg.range_noise_std_m = 0.0;
    cfg.bearing_noise_std_rad = 0.0;
    cfg.detection_probability = 1.0;
    cfg.mean_false_alarms_per_scan = 0.0;

    RadarSensor radar(cfg);

    ObjectState st;
    st.id = 1;
    st.px = 400.0;
    st.py = 600.0;
    std::vector<SimulatedObject> objects;
    objects.emplace_back(st, std::make_unique<ConstantVelocityMotionModel>());

    DeterministicRng rng(42);
    std::uint64_t next_id = 1;
    const auto meas = radar.generateMeasurements(0.0, objects, rng, next_id);

    ASSERT_EQ(meas.size(), 1U);
    EXPECT_NEAR(meas[0].z(0), 500.0, 1e-9);
    EXPECT_NEAR(meas[0].z(1), std::atan2(400.0, 300.0), 1e-9);

    const Vector2d cart = meas[0].toCartesianPosition();
    EXPECT_NEAR(cart.x(), 400.0, 1e-9);
    EXPECT_NEAR(cart.y(), 600.0, 1e-9);
}

TEST(SensorTest, GnssTelemetryOnlyGeneratedForEquippedObjects) {
    GnssSensorConfig cfg;
    cfg.id = "gnss_test";
    cfg.position_noise_std_x_m = 0.0;
    cfg.position_noise_std_y_m = 0.0;
    cfg.packet_loss_probability = 0.0;

    GnssSensor gnss(cfg);

    ObjectState st1{1, 50.0, -75.0, 10.0, 0.0, std::nullopt, std::nullopt, 0.0};
    ObjectState st2{2, -120.0, 90.0, 5.0, 5.0, std::nullopt, std::nullopt, 0.0};

    std::vector<SimulatedObject> objects;
    objects.emplace_back(st1, std::make_unique<ConstantVelocityMotionModel>(), "Equipped", true);
    objects.emplace_back(st2, std::make_unique<ConstantVelocityMotionModel>(), "Silent", false);

    DeterministicRng rng(99);
    std::uint64_t next_id = 1;
    const auto meas = gnss.generateMeasurements(0.0, objects, rng, next_id);

    ASSERT_EQ(meas.size(), 1U);
    EXPECT_NEAR(meas[0].z(0), 50.0, 1e-9);
    EXPECT_NEAR(meas[0].z(1), -75.0, 1e-9);
}

TEST(SensorTest, NoiseReproducibilityWithSameSeed) {
    RadarSensorConfig cfg;
    cfg.range_noise_std_m = 10.0;
    cfg.bearing_noise_std_rad = 0.02;
    cfg.detection_probability = 1.0;
    cfg.mean_false_alarms_per_scan = 1.5;

    ObjectState st{1, 500.0, 500.0, 10.0, -5.0, std::nullopt, std::nullopt, 0.0};
    std::vector<SimulatedObject> objects;
    objects.emplace_back(st, std::make_unique<ConstantVelocityMotionModel>());

    RadarSensor r1(cfg);
    RadarSensor r2(cfg);
    DeterministicRng rng1(777);
    DeterministicRng rng2(777);
    std::uint64_t id1 = 1;
    std::uint64_t id2 = 1;

    const auto m1 = r1.generateMeasurements(0.0, objects, rng1, id1);
    const auto m2 = r2.generateMeasurements(0.0, objects, rng2, id2);

    ASSERT_EQ(m1.size(), m2.size());
    for (std::size_t i = 0; i < m1.size(); ++i) {
        EXPECT_DOUBLE_EQ(m1[i].z(0), m2[i].z(0));
        EXPECT_DOUBLE_EQ(m1[i].z(1), m2[i].z(1));
    }
}

TEST(SensorTest, DetectionProbabilityBoundariesAndSensorShutdown) {
    RadarSensorConfig cfg;
    cfg.detection_probability = 0.0;
    cfg.mean_false_alarms_per_scan = 0.0;
    RadarSensor radar_zero(cfg);

    ObjectState st{1, 300.0, 300.0, 0.0, 0.0, std::nullopt, std::nullopt, 0.0};
    std::vector<SimulatedObject> objects;
    objects.emplace_back(st, std::make_unique<ConstantVelocityMotionModel>());

    DeterministicRng rng(42);
    std::uint64_t next_id = 1;
    EXPECT_TRUE(radar_zero.generateMeasurements(0.0, objects, rng, next_id).empty());
    EXPECT_EQ(radar_zero.runtimeStats().detection_failures, 1U);

    cfg.detection_probability = 1.0;
    cfg.outage_windows.push_back(OutageWindow{5.0, 10.0});
    RadarSensor radar_outage(cfg);
    EXPECT_EQ(radar_outage.generateMeasurements(2.0, objects, rng, next_id).size(), 1U);
    EXPECT_TRUE(radar_outage.generateMeasurements(7.0, objects, rng, next_id).empty());

    radar_outage.setManualOperational(false);
    EXPECT_TRUE(radar_outage.generateMeasurements(12.0, objects, rng, next_id).empty());
}

TEST(SensorTest, SensorSchedulerDropsStaleOutOfSequenceMeasurements) {
    SensorSchedulerConfig sched_cfg;
    sched_cfg.oosm_policy = OutOfSequencePolicy::DropStale;
    SensorScheduler scheduler(sched_cfg);

    RadarSensorConfig slow_lat_radar;
    slow_lat_radar.id = "radar_delayed";
    slow_lat_radar.sampling_frequency_hz = 1.0;
    slow_lat_radar.latency_sec = 0.6;
    slow_lat_radar.mean_false_alarms_per_scan = 0.0;
    slow_lat_radar.detection_probability = 1.0;

    GnssSensorConfig fast_gnss;
    fast_gnss.id = "gnss_fast";
    fast_gnss.sampling_frequency_hz = 2.0;
    fast_gnss.latency_sec = 0.0;
    fast_gnss.packet_loss_probability = 0.0;

    scheduler.addSensor( std::make_unique<RadarSensor>(slow_lat_radar));
    scheduler.addSensor(std::make_unique<GnssSensor>(fast_gnss));

    ObjectState st{1, 300.0, 400.0, 10.0, 10.0, std::nullopt, std::nullopt, 0.0};
    std::vector<SimulatedObject> objects;
    objects.emplace_back(st, std::make_unique<ConstantVelocityMotionModel>());

    DeterministicRng rng(42);
    // t = 0.0: gnss_fast arrives immediately (capture=0.0); radar_delayed arrives at t=0.6
    auto ready0 = scheduler.sampleAndReleaseReady(0.0, objects, rng);
    ASSERT_EQ(ready0.size(), 1U);
    EXPECT_EQ(ready0[0].sensor_id, "gnss_fast");

    // t = 0.5: gnss_fast samples at 0.5 and arrives at 0.5 (last_released_capture_time becomes 0.5)
    objects.front().step(0.5, 0.5);
    auto ready05 = scheduler.sampleAndReleaseReady(0.5, objects, rng);
    ASSERT_EQ(ready05.size(), 1U);
    EXPECT_DOUBLE_EQ(ready05[0].capture_timestamp, 0.5);

    // t = 0.6: radar_delayed's t=0.0 sample arrives at 0.6, which is stale (0.0 < 0.5) -> dropped!
    auto ready06 = scheduler.sampleAndReleaseReady(0.6, objects, rng);
    EXPECT_TRUE(ready06.empty());
    EXPECT_EQ(scheduler.droppedStaleCount(), 1U);
}

TEST(SensorTest, CatchUpSamplesUsePositionAtCaptureTime) {
    GnssSensorConfig cfg;
    cfg.id = "gnss_fast";
    cfg.sampling_frequency_hz = 4.0;
    cfg.position_noise_std_x_m = 0.0;
    cfg.position_noise_std_y_m = 0.0;
    cfg.packet_loss_probability = 0.0;

    SensorScheduler scheduler;
    scheduler.addSensor(std::make_unique<GnssSensor>(cfg));
    ObjectState initial{1, 0.0, 0.0, 10.0, 0.0, std::nullopt, std::nullopt, 0.0};
    std::vector<SimulatedObject> objects;
    objects.emplace_back(initial, std::make_unique<ConstantVelocityMotionModel>());
    DeterministicRng rng(42);

    const auto first = scheduler.sampleAndReleaseReady(0.0, objects, rng);
    ASSERT_EQ(first.size(), 1U);
    objects.front().step(1.0, 1.0);
    const auto samples = scheduler.sampleAndReleaseReady(1.0, objects, rng);
    ASSERT_EQ(samples.size(), 4U);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_NEAR(samples[i].capture_timestamp, (i + 1) * 0.25, 1e-9);
        EXPECT_NEAR(samples[i].z(0), (i + 1) * 2.5, 1e-9);
    }
}

TEST(SensorTest, DelayedMeasurementErrorUsesCaptureTimeTruth) {
    ObjectState initial{1, 0.0, 0.0, 10.0, 0.0, std::nullopt, std::nullopt, 0.0};
    std::vector<SimulatedObject> objects;
    objects.emplace_back(initial, std::make_unique<ConstantVelocityMotionModel>());
    objects.front().step(1.0, 1.0);

    Measurement delayed;
    delayed.measurement_id = 1;
    delayed.sensor_id = "gnss";
    delayed.sensor_type = SensorType::Gnss;
    delayed.capture_timestamp = 0.0;
    delayed.arrival_timestamp = 1.0;
    delayed.z = Vector2d{0.0, 0.0};
    delayed.R = Matrix2d::Identity();
    delayed.evaluation_ground_truth_id = 1;
    const std::vector<Measurement> measurements{delayed};

    TrackManager tracks;
    MetricsCollector metrics;
    const auto snapshot = metrics.evaluateStep(1.0, objects, tracks, measurements, 0, 0, 0.0);
    EXPECT_DOUBLE_EQ(snapshot.raw_measurement_rmse_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.cumulativeRawMeasurementRmseM(), 0.0);
}

TEST(SensorTest, IntermediateAccelerationStateUsesPositionAndVelocity) {
    ObjectState initial{1, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0};
    SimulatedObject object(initial, std::make_unique<ConstantAccelerationMotionModel>(4.0, 0.0));
    object.step(1.0, 1.0);
    const auto midway = object.stateAt(0.5);
    ASSERT_TRUE(midway.has_value());
    EXPECT_NEAR(midway->px, 0.5, 1e-9);
    EXPECT_NEAR(midway->vx, 2.0, 1e-9);
}

}  // namespace aegistrack::testing
