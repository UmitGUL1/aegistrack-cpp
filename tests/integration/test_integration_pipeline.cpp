#include "aegistrack/config/configuration_manager.hpp"
#include "aegistrack/io/session_recorder.hpp"
#include "aegistrack/simulation/simulation_engine.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace aegistrack::testing {

TEST(IntegrationPipelineTest, MultiSensorFusionImprovesAccuracyUnderUnbiasedComplementaryNoise) {
    const std::filesystem::path scenario_path =
        std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "fusion-comparison.json";
    const ScenarioConfig cfg = ConfigurationManager::loadFromFile(scenario_path);

    SimulationEngine engine(cfg);
    const FusionComparisonReport report = engine.runFusionComparison();

    EXPECT_GT(report.radar_only.total_measurements_processed, 0U);
    EXPECT_GT(report.gnss_only.total_measurements_processed, 0U);
    EXPECT_GT(report.radar_gnss_fusion.total_measurements_processed, 0U);

    // Under unbiased complementary noise, Radar+GNSS fusion achieves strictly lower Position RMSE
    // than either Radar-only or GNSS-only alone.
    EXPECT_LT(report.radar_gnss_fusion.cumulative_position_rmse_m,
              report.radar_only.cumulative_position_rmse_m);
    EXPECT_LT(report.radar_gnss_fusion.cumulative_position_rmse_m,
              report.gnss_only.cumulative_position_rmse_m);
}

TEST(IntegrationPipelineTest, UnmodeledGnssBiasCanDegradeFusionComparedToUnbiasedRadarOnly) {
    // Section 5.4: Demonstrate a scenario where fusion does NOT improve accuracy over a single
    // sensor because the GNSS telemetry suffers from a severe uncompensated 25m multipath bias.
    const std::filesystem::path scenario_path =
        std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "fusion-comparison.json";
    ScenarioConfig cfg = ConfigurationManager::loadFromFile(scenario_path);
    ASSERT_FALSE(cfg.gnss_sensors.empty());
    cfg.gnss_sensors[0].bias_x_m = 25.0;
    cfg.gnss_sensors[0].bias_y_m = -20.0;
    cfg.gnss_sensors[0].position_noise_std_x_m = 4.0;
    cfg.gnss_sensors[0].position_noise_std_y_m = 4.0;

    SimulationEngine engine(cfg);
    const FusionComparisonReport report = engine.runFusionComparison();

    // Unbiased Radar-only achieves lower position RMSE than fusion corrupted by biased GNSS
    EXPECT_LT(report.radar_only.cumulative_position_rmse_m,
              report.radar_gnss_fusion.cumulative_position_rmse_m);
}

TEST(IntegrationPipelineTest, SensorDropoutAndOcclusionScenariosMaintainAndRecoverTracks) {
    const std::filesystem::path failure_path =
        std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "sensor_failure.json";
    SimulationEngine failure_engine(ConfigurationManager::loadFromFile(failure_path));
    failure_engine.runToCompletion();

    EXPECT_GE(failure_engine.fusionEngine().trackManager().confirmedTrackCount(), 2U);
    EXPECT_LT(failure_engine.metricsCollector().cumulativePositionRmseM(), 25.0);

    const std::filesystem::path occlusion_path =
        std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "occlusion.json";
    SimulationEngine occlusion_engine(ConfigurationManager::loadFromFile(occlusion_path));
    occlusion_engine.runToCompletion();

    EXPECT_EQ(occlusion_engine.fusionEngine().trackManager().confirmedTrackCount(), 2U);
}

TEST(IntegrationPipelineTest, SessionExportReloadAndMeasurementReplayAreDeterministic) {
    const std::filesystem::path basic_path =
        std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "crossing.json";
    SimulationEngine engine(ConfigurationManager::loadFromFile(basic_path));
    engine.runToCompletion();

    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "aegistrack_test_export";
    std::filesystem::remove_all(temp_dir);

    SessionRecorder::exportBundle(engine, temp_dir);

    EXPECT_TRUE(std::filesystem::exists(temp_dir / "session.json"));
    EXPECT_TRUE(std::filesystem::exists(temp_dir / "tracks.csv"));
    EXPECT_TRUE(std::filesystem::exists(temp_dir / "measurements.csv"));
    EXPECT_TRUE(std::filesystem::exists(temp_dir / "metrics.csv"));
    EXPECT_TRUE(std::filesystem::exists(temp_dir / "ground_truth.csv"));

    const RecordedSession loaded = SessionRecorder::loadSessionJson(temp_dir / "session.json");
    EXPECT_EQ(loaded.recorded_measurements.size(), engine.allRecordedMeasurements().size());

    // Replay with Hungarian vs NearestNeighbor on identical recorded measurements
    TrackManagerConfig hun_cfg = loaded.scenario_config.tracker_config;
    hun_cfg.association_algorithm = AssociationAlgorithmType::Hungarian;
    const auto hun_replay = SessionRecorder::replayMeasurements(loaded, hun_cfg);

    TrackManagerConfig nn_cfg = loaded.scenario_config.tracker_config;
    nn_cfg.association_algorithm = AssociationAlgorithmType::NearestNeighbor;
    const auto nn_replay = SessionRecorder::replayMeasurements(loaded, nn_cfg);

    EXPECT_NEAR(hun_replay.cumulative_position_rmse_m,
                engine.metricsCollector().cumulativePositionRmseM(),
                1e-6);
    EXPECT_EQ(hun_replay.total_measurements_processed, nn_replay.total_measurements_processed);

    std::filesystem::remove_all(temp_dir);
}

TEST(IntegrationPipelineTest, GracefulStopStepPauseResumeAndRestart) {
    const std::filesystem::path basic_path =
        std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "basic.json";
    SimulationEngine engine(ConfigurationManager::loadFromFile(basic_path));

    EXPECT_TRUE(engine.isPaused());
    EXPECT_DOUBLE_EQ(engine.currentTime(), 0.0);

    engine.step();
    EXPECT_NEAR(engine.currentTime(), 0.1, 1e-9);

    engine.resume();
    EXPECT_FALSE(engine.isPaused());
    for (int i = 0; i < 10; ++i) {
        engine.step();
    }
    engine.pause();
    EXPECT_TRUE(engine.isPaused());

    engine.reset();
    EXPECT_DOUBLE_EQ(engine.currentTime(), 0.0);
    EXPECT_FALSE(engine.isCompleted());
}

}  // namespace aegistrack::testing
