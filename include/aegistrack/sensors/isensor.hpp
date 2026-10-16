#pragma once

#include "aegistrack/core/random.hpp"
#include "aegistrack/core/types.hpp"
#include "aegistrack/sensors/measurement.hpp"
#include "aegistrack/simulation/simulated_object.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace aegistrack {

/**
 * @brief Time interval during which a sensor experiences a scheduled outage.
 */
struct OutageWindow {
    TimestampSec start_time{0.0};
    TimestampSec end_time{0.0};
};

/**
 * @brief Circular region in the 2D plane where line-of-sight or signal reception is occluded.
 */
struct OcclusionZone {
    double center_x{0.0};
    double center_y{0.0};
    double radius_m{0.0};
};

struct SensorRuntimeStats {
    std::uint64_t total_scans{0};
    std::uint64_t measurements_generated{0};
    std::uint64_t detection_failures{0};
    std::uint64_t false_alarms_generated{0};
};

/**
 * @brief Polymorphic interface for synthetic sensors producing timestamped Measurements.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    [[nodiscard]] virtual const SensorId& id() const noexcept = 0;
    [[nodiscard]] virtual SensorType type() const noexcept = 0;
    [[nodiscard]] virtual double samplingIntervalSec() const noexcept = 0;
    [[nodiscard]] virtual double samplingFrequencyHz() const noexcept = 0;
    [[nodiscard]] virtual Vector2d position() const noexcept = 0;

    [[nodiscard]] virtual bool isOperationalAt(TimestampSec timestamp) const noexcept = 0;
    virtual void setManualOperational(bool operational) noexcept = 0;
    [[nodiscard]] virtual bool isManuallyOperational() const noexcept = 0;

    /**
     * @brief Samples the physical world at current_time and returns generated Measurements.
     */
    [[nodiscard]] virtual std::vector<Measurement> generateMeasurements(
        TimestampSec current_time,
        std::span<const SimulatedObject> objects,
        DeterministicRng& rng,
        std::uint64_t& next_measurement_id) = 0;

    [[nodiscard]] virtual SensorRuntimeStats runtimeStats() const noexcept = 0;
    virtual void reset() noexcept = 0;
};

}  // namespace aegistrack
