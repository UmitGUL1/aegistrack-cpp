#pragma once

#include "aegistrack/core/types.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace aegistrack {

/**
 * @brief Deterministic simulation clock strictly decoupled from wall-clock time.
 *
 * Uses integer step accumulation alongside nominal step size where possible to
 * minimize floating-point drift over long simulation runs.
 */
class SimulationClock {
public:
    explicit SimulationClock(TimestampSec initial_time = 0.0, double nominal_dt = 0.05)
        : initial_time_(initial_time),
          current_time_(initial_time),
          nominal_dt_(nominal_dt),
          step_count_(0) {
        if (!std::isfinite(initial_time) || !std::isfinite(nominal_dt) || nominal_dt <= 0.0) {
            throw std::invalid_argument("SimulationClock requires finite initial_time and nominal_dt > 0");
        }
    }

    /// Advances the simulation clock by nominal_dt_.
    TimestampSec tick() noexcept {
        ++step_count_;
        current_time_ = initial_time_ + static_cast<double>(step_count_) * nominal_dt_;
        return current_time_;
    }

    /// Advances the simulation clock by an explicit positive delta.
    TimestampSec advance(double dt) {
        if (!std::isfinite(dt) || dt < 0.0) {
            throw std::invalid_argument("SimulationClock::advance requires finite non-negative dt");
        }
        ++step_count_;
        current_time_ += dt;
        return current_time_;
    }

    /// Resets the clock back to initial_time.
    void reset(TimestampSec initial_time = 0.0) {
        if (!std::isfinite(initial_time)) {
            throw std::invalid_argument("SimulationClock::reset requires finite initial_time");
        }
        initial_time_ = initial_time;
        current_time_ = initial_time;
        step_count_ = 0;
    }

    [[nodiscard]] TimestampSec currentTime() const noexcept {
        return current_time_;
    }

    [[nodiscard]] double nominalDt() const noexcept {
        return nominal_dt_;
    }

    void setNominalDt(double dt) {
        if (!std::isfinite(dt) || dt <= 0.0) {
            throw std::invalid_argument("SimulationClock::setNominalDt requires finite dt > 0");
        }
        nominal_dt_ = dt;
        initial_time_ = current_time_;
        step_count_ = 0;
    }

    [[nodiscard]] std::uint64_t stepCount() const noexcept {
        return step_count_;
    }

private:
    TimestampSec initial_time_;
    TimestampSec current_time_;
    double nominal_dt_;
    std::uint64_t step_count_;
};

}  // namespace aegistrack
