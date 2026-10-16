#include "aegistrack/sensors/sensor_factory.hpp"

#include <utility>

namespace aegistrack {

std::unique_ptr<ISensor> SensorFactory::createRadar(RadarSensorConfig config) {
    return std::make_unique<RadarSensor>(std::move(config));
}

std::unique_ptr<ISensor> SensorFactory::createGnss(GnssSensorConfig config) {
    return std::make_unique<GnssSensor>(std::move(config));
}

}  // namespace aegistrack
