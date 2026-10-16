#pragma once

#include "aegistrack/sensors/gnss_sensor.hpp"
#include "aegistrack/sensors/isensor.hpp"
#include "aegistrack/sensors/radar_sensor.hpp"

#include <memory>

namespace aegistrack {

/**
 * @brief Factory for constructing configured sensor instances.
 */
class SensorFactory {
public:
    [[nodiscard]] static std::unique_ptr<ISensor> createRadar(RadarSensorConfig config);
    [[nodiscard]] static std::unique_ptr<ISensor> createGnss(GnssSensorConfig config);
};

}  // namespace aegistrack
