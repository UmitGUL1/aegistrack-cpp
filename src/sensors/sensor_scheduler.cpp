#include "aegistrack/sensors/sensor_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aegistrack {

SensorScheduler::SensorScheduler(SensorSchedulerConfig config, EventBus* event_bus)
    : config_(config), event_bus_(event_bus) {
    if (!std::isfinite(config_.reorder_buffer_window_sec) ||
        config_.reorder_buffer_window_sec < 0.0) {
        throw std::invalid_argument(
            "SensorScheduler: reorder_buffer_window_sec must be finite and >= 0");
    }
    if (config_.max_in_flight_queue_size == 0) {
        throw std::invalid_argument(
            "SensorScheduler: max_in_flight_queue_size must be > 0");
    }
}

void SensorScheduler::addSensor(std::unique_ptr<ISensor> sensor) {
    if (!sensor) {
        throw std::invalid_argument("SensorScheduler::addSensor requires non-null sensor");
    }
    for (const auto& entry : entries_) {
        if (entry.sensor->id() == sensor->id()) {
            throw std::invalid_argument("SensorScheduler::addSensor: duplicate sensor ID '" +
                                        sensor->id() + "'");
        }
    }
    const bool op = sensor->isOperationalAt(0.0);
    entries_.push_back(ScheduledSensorEntry{std::move(sensor), 0.0, op});
}

ISensor* SensorScheduler::findSensor(const SensorId& id) noexcept {
    for (auto& entry : entries_) {
        if (entry.sensor->id() == id) {
            return entry.sensor.get();
        }
    }
    return nullptr;
}

const ISensor* SensorScheduler::findSensor(const SensorId& id) const noexcept {
    for (const auto& entry : entries_) {
        if (entry.sensor->id() == id) {
            return entry.sensor.get();
        }
    }
    return nullptr;
}

std::vector<Measurement> SensorScheduler::sampleAndReleaseReady(
    TimestampSec current_time,
    std::span<const SimulatedObject> objects,
    DeterministicRng& rng) {
    if (!std::isfinite(current_time) || current_time < 0.0) {
        throw std::invalid_argument("SensorScheduler::sampleAndReleaseReady requires current_time >= 0");
    }

    for (auto& entry : entries_) {
        const bool op_now = entry.sensor->isOperationalAt(current_time);
        if (entry.was_operational && !op_now && event_bus_ != nullptr) {
            event_bus_->publish(SystemEvent{0,
                                            current_time,
                                            SystemEventType::SensorOutageStarted,
                                            0,
                                            entry.sensor->id(),
                                            "Sensor '" + entry.sensor->id() + "' entered outage"});
        } else if (!entry.was_operational && op_now && event_bus_ != nullptr) {
            event_bus_->publish(SystemEvent{0,
                                            current_time,
                                            SystemEventType::SensorOutageEnded,
                                            0,
                                            entry.sensor->id(),
                                            "Sensor '" + entry.sensor->id() + "' resumed operation"});
        }
        entry.was_operational = op_now;

        while (entry.next_sample_time <= current_time + 1e-9) {
            const TimestampSec sample_time = entry.next_sample_time;
            auto batch = entry.sensor->generateMeasurements(
                sample_time, objects, rng, next_measurement_id_);
            for (auto& m : batch) {
                if (in_flight_queue_.size() >= config_.max_in_flight_queue_size) {
                    ++dropped_overflow_count_;
                    continue;
                }
                in_flight_queue_.push_back(std::move(m));
            }
            entry.next_sample_time += entry.sensor->samplingIntervalSec();
        }
    }

    std::vector<Measurement> arrived;
    std::vector<Measurement> remaining;
    remaining.reserve(in_flight_queue_.size());

    const TimestampSec release_horizon =
        (config_.oosm_policy == OutOfSequencePolicy::BoundedReorderBuffer)
            ? (current_time - config_.reorder_buffer_window_sec)
            : current_time;

    for (auto& m : in_flight_queue_) {
        if (m.arrival_timestamp <= current_time + 1e-9) {
            if (config_.oosm_policy == OutOfSequencePolicy::BoundedReorderBuffer &&
                m.capture_timestamp > release_horizon + 1e-9) {
                remaining.push_back(std::move(m));
            } else {
                arrived.push_back(std::move(m));
            }
        } else {
            remaining.push_back(std::move(m));
        }
    }
    in_flight_queue_ = std::move(remaining);

    // Sort arrived measurements chronologically by capture_timestamp, then by measurement_id
    std::stable_sort(arrived.begin(), arrived.end(), [](const Measurement& a, const Measurement& b) {
        if (a.capture_timestamp != b.capture_timestamp) {
            return a.capture_timestamp < b.capture_timestamp;
        }
        return a.measurement_id < b.measurement_id;
    });

    std::vector<Measurement> ready;
    ready.reserve(arrived.size());
    for (auto& m : arrived) {
        if (m.capture_timestamp + 1e-9 < last_released_capture_time_) {
            ++dropped_stale_count_;
            if (event_bus_ != nullptr) {
                event_bus_->publish(SystemEvent{
                    0,
                    current_time,
                    SystemEventType::StaleMeasurementDropped,
                    0,
                    m.sensor_id,
                    "Dropped stale out-of-sequence measurement from '" + m.sensor_id + "'"});
            }
            continue;
        }
        last_released_capture_time_ = m.capture_timestamp;
        ready.push_back(std::move(m));
    }

    return ready;
}

void SensorScheduler::reset() {
    for (auto& entry : entries_) {
        entry.sensor->reset();
        entry.next_sample_time = 0.0;
        entry.was_operational = entry.sensor->isOperationalAt(0.0);
    }
    in_flight_queue_.clear();
    last_released_capture_time_ = -1.0;
    next_measurement_id_ = 1;
    dropped_stale_count_ = 0;
    dropped_overflow_count_ = 0;
}

}  // namespace aegistrack
