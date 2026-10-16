#pragma once

#include "aegistrack/core/types.hpp"
#include "aegistrack/sensors/gnss_sensor.hpp"
#include "aegistrack/sensors/radar_sensor.hpp"
#include "aegistrack/sensors/sensor_scheduler.hpp"
#include "aegistrack/simulation/simulated_object.hpp"
#include "aegistrack/tracking/track_manager.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace aegistrack {

struct ScenarioConfig {
    std::string name{"Default Scenario"};
    std::string description{"Synthetic 2D multi-sensor tracking scenario"};
    double duration_sec{30.0};
    double dt_sec{0.1};
    std::uint64_t seed{42ULL};

    std::vector<SimulatedObjectConfig> objects;
    std::vector<RadarSensorConfig> radars;
    std::vector<GnssSensorConfig> gnss_sensors;

    SensorSchedulerConfig scheduler_config{};
    TrackManagerConfig tracker_config{};

    /// When true (e.g. Scenario I), CLI and Analytics can run matched Radar-only, GNSS-only,
    /// and Radar+GNSS fusion passes on the exact same ground truth.
    bool enable_fusion_comparison{false};
};

}  // namespace aegistrack
