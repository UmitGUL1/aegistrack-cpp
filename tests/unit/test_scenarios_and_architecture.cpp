#include "aegistrack/association/hungarian_associator.hpp"
#include "aegistrack/association/nearest_neighbor_associator.hpp"
#include "aegistrack/config/configuration_manager.hpp"
#include "aegistrack/events/event_bus.hpp"
#include "aegistrack/sensors/sensor_factory.hpp"
#include "aegistrack/simulation/simulation_engine.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

namespace aegistrack::testing {

TEST(ScenarioAndArchitectureTest, InvalidJsonAndInvalidParametersAreRejected) {
    EXPECT_THROW((void)ConfigurationManager::loadFromString(""), std::invalid_argument);
    EXPECT_THROW((void)ConfigurationManager::loadFromString("{broken json"), std::invalid_argument);

    // Missing objects
    EXPECT_THROW((void)ConfigurationManager::loadFromString(R"({
        "name": "Empty",
        "duration_sec": 10.0,
        "radars": [{"id": "r1"}]
    })"),
                 std::invalid_argument);

    // Negative noise variance / std
    EXPECT_THROW((void)ConfigurationManager::loadFromString(R"({
        "name": "NegativeNoise",
        "duration_sec": 10.0,
        "objects": [{"id": 1, "px": 0.0, "py": 0.0, "vx": 1.0, "vy": 1.0}],
        "radars": [{"id": "r1", "range_noise_std_m": -5.0}]
    })"),
                 std::invalid_argument);
}

TEST(ScenarioAndArchitectureTest, IdenticalSeedsProduceBitIdenticalResultsAndDifferentSeedsDiffer) {
    const std::string json_text = R"({
        "name": "Reproducibility Test",
        "duration_sec": 15.0,
        "dt_sec": 0.1,
        "seed": 100,
        "objects": [
            {"id": 1, "px": -400.0, "py": -200.0, "vx": 25.0, "vy": 15.0, "has_gnss_telemetry": true},
            {"id": 2, "px": 400.0, "py": 200.0, "vx": -20.0, "vy": -10.0, "has_gnss_telemetry": true}
        ],
        "radars": [{"id": "radar_1", "sampling_frequency_hz": 2.0, "range_noise_std_m": 8.0}],
        "gnss_sensors": [{"id": "gnss_1", "sampling_frequency_hz": 1.0, "position_noise_std_x_m": 6.0}]
    })";

    ScenarioConfig cfg1 = ConfigurationManager::loadFromString(json_text);
    ScenarioConfig cfg2 = ConfigurationManager::loadFromString(json_text);
    ScenarioConfig cfg_diff = ConfigurationManager::loadFromString(json_text);
    cfg_diff.seed = 99999ULL;

    SimulationEngine eng1(cfg1);
    SimulationEngine eng2(cfg2);
    SimulationEngine eng_diff(cfg_diff);

    eng1.runToCompletion();
    eng2.runToCompletion();
    eng_diff.runToCompletion();

    EXPECT_DOUBLE_EQ(eng1.metricsCollector().cumulativePositionRmseM(),
                     eng2.metricsCollector().cumulativePositionRmseM());
    EXPECT_DOUBLE_EQ(eng1.metricsCollector().cumulativeVelocityRmseM(),
                     eng2.metricsCollector().cumulativeVelocityRmseM());
    EXPECT_NE(eng1.metricsCollector().cumulativePositionRmseM(),
              eng_diff.metricsCollector().cumulativePositionRmseM());
}

TEST(ScenarioAndArchitectureTest, InterfaceSubstitutionAndEventBusDistribution) {
    EventBus bus;
    int received_events = 0;
    const auto sub_id = bus.subscribe([&](const SystemEvent&) { ++received_events; });

    TrackManagerConfig cfg;
    TrackManager tm(cfg, std::make_unique<NearestNeighborAssociator>(), &bus);
    EXPECT_EQ(tm.associator().type(), AssociationAlgorithmType::NearestNeighbor);

    tm.setAssociator(std::make_unique<HungarianAssociator>());
    EXPECT_EQ(tm.associator().type(), AssociationAlgorithmType::Hungarian);

    auto radar = SensorFactory::createRadar(RadarSensorConfig{});
    auto gnss = SensorFactory::createGnss(GnssSensorConfig{});
    EXPECT_EQ(radar->type(), SensorType::Radar);
    EXPECT_EQ(gnss->type(), SensorType::Gnss);

    bus.publish(SystemEvent{0, 1.0, SystemEventType::TrackCreated, 1, "radar_1", "Test"});
    EXPECT_EQ(received_events, 1);

    bus.unsubscribe(sub_id);
    bus.publish(SystemEvent{0, 2.0, SystemEventType::TrackConfirmed, 1, "radar_1", "Test2"});
    EXPECT_EQ(received_events, 1);
}

TEST(ScenarioAndArchitectureTest, AllBundledJsonScenariosLoadAndValidateCleanly) {
    const std::filesystem::path scenario_dir = std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios";
    const std::vector<std::string> files = {
        "basic.json",
        "multiple_objects.json",
        "crossing.json",
        "noisy_measurements.json",
        "sensor_failure.json",
        "occlusion.json",
        "dense_environment.json",
        "stress.json",
        "fusion-comparison.json",
        "dynamic_maneuvers.json"
    };

    for (const auto& f : files) {
        const auto cfg = ConfigurationManager::loadFromFile(scenario_dir / f);
        EXPECT_FALSE(cfg.name.empty()) << "Failed on " << f;
        EXPECT_FALSE(cfg.objects.empty()) << "Failed on " << f;
    }
}

}  // namespace aegistrack::testing
