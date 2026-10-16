#pragma once

#include "aegistrack/core/random.hpp"
#include "aegistrack/core/types.hpp"
#include "aegistrack/events/event_bus.hpp"
#include "aegistrack/sensors/isensor.hpp"
#include "aegistrack/sensors/measurement.hpp"
#include "aegistrack/simulation/simulated_object.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace aegistrack {

struct SensorSchedulerConfig {
    OutOfSequencePolicy oosm_policy{OutOfSequencePolicy::DropStale};
    double reorder_buffer_window_sec{0.2};
    std::size_t max_in_flight_queue_size{10000};
};

/**
 * @brief Schedules asynchronous sensors at independent rates, models delivery latency,
 * and enforces chronological measurement ordering (DropStale or BoundedReorderBuffer).
 */
class SensorScheduler {
public:
    explicit SensorScheduler(SensorSchedulerConfig config = {}, EventBus* event_bus = nullptr);

    void addSensor(std::unique_ptr<ISensor> sensor);

    /**
     * @brief Samples all sensors due at or before current_time, queues measurements with latency,
     * and returns the chronologically ordered measurements ready for processing at current_time.
     */
    [[nodiscard]] std::vector<Measurement> sampleAndReleaseReady(
        TimestampSec current_time,
        std::span<const SimulatedObject> objects,
        DeterministicRng& rng);

    void reset();

    [[nodiscard]] std::size_t sensorCount() const noexcept { return entries_.size(); }
    [[nodiscard]] const ISensor* sensorAt(std::size_t index) const {
        return entries_.at(index).sensor.get();
    }
    [[nodiscard]] ISensor* sensorAt(std::size_t index) {
        return entries_.at(index).sensor.get();
    }
    [[nodiscard]] ISensor* findSensor(const SensorId& id) noexcept;
    [[nodiscard]] const ISensor* findSensor(const SensorId& id) const noexcept;

    [[nodiscard]] std::uint64_t droppedStaleCount() const noexcept { return dropped_stale_count_; }
    [[nodiscard]] std::uint64_t droppedOverflowCount() const noexcept {
        return dropped_overflow_count_;
    }

private:
    struct ScheduledSensorEntry {
        std::unique_ptr<ISensor> sensor;
        TimestampSec next_sample_time{0.0};
        bool was_operational{true};
    };

    SensorSchedulerConfig config_;
    EventBus* event_bus_;
    std::vector<ScheduledSensorEntry> entries_;
    std::vector<Measurement> in_flight_queue_;
    TimestampSec last_released_capture_time_{-1.0};
    std::uint64_t next_measurement_id_{1};
    std::uint64_t dropped_stale_count_{0};
    std::uint64_t dropped_overflow_count_{0};
};

}  // namespace aegistrack
