#pragma once

#include "aegistrack/core/types.hpp"

#include <cstdint>
#include <string>

namespace aegistrack {

enum class SystemEventType {
    TrackCreated,
    TrackConfirmed,
    TrackCoasting,
    TrackRecovered,
    TrackDeleted,
    SensorOutageStarted,
    SensorOutageEnded,
    StaleMeasurementDropped,
    ScenarioStarted,
    ScenarioCompleted
};

[[nodiscard]] constexpr std::string_view toString(SystemEventType type) noexcept {
    switch (type) {
        case SystemEventType::TrackCreated:
            return "TrackCreated";
        case SystemEventType::TrackConfirmed:
            return "TrackConfirmed";
        case SystemEventType::TrackCoasting:
            return "TrackCoasting";
        case SystemEventType::TrackRecovered:
            return "TrackRecovered";
        case SystemEventType::TrackDeleted:
            return "TrackDeleted";
        case SystemEventType::SensorOutageStarted:
            return "SensorOutageStarted";
        case SystemEventType::SensorOutageEnded:
            return "SensorOutageEnded";
        case SystemEventType::StaleMeasurementDropped:
            return "StaleMeasurementDropped";
        case SystemEventType::ScenarioStarted:
            return "ScenarioStarted";
        case SystemEventType::ScenarioCompleted:
            return "ScenarioCompleted";
    }
    return "Unknown";
}

struct SystemEvent {
    std::uint64_t event_id{0};
    TimestampSec timestamp{0.0};
    SystemEventType type{SystemEventType::ScenarioStarted};
    TrackId related_track_id{0};
    SensorId related_sensor_id;
    std::string message;
};

}  // namespace aegistrack
