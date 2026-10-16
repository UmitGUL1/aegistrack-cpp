#pragma once

#include "aegistrack/core/object_state.hpp"
#include "aegistrack/simulation/motion_model.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aegistrack {

/**
 * @brief Scheduled kinematic maneuver interval for dynamic maneuver evaluation scenarios.
 */
struct ManeuverCommand {
    TimestampSec start_time{0.0};
    TimestampSec end_time{0.0};
    MotionModelType model_type{MotionModelType::ConstantVelocity};
    double ax{0.0};
    double ay{0.0};
    double turn_rate_rad_s{0.0};
};

struct SimulatedObjectConfig {
    ObjectId id{1};
    std::string label{"Target"};
    ObjectState initial_state{};
    MotionModelType initial_motion_model{MotionModelType::ConstantVelocity};
    double turn_rate_rad_s{0.0};
    TimestampSec spawn_time{0.0};
    std::optional<TimestampSec> despawn_time{std::nullopt};
    bool has_gnss_telemetry{true};
    std::vector<ManeuverCommand> maneuvers{};
    std::size_t max_history_length{2000};
};

/**
 * @brief Ground-truth moving entity managed exclusively by the SimulationEngine.
 */
class SimulatedObject {
public:
    explicit SimulatedObject(SimulatedObjectConfig config);
    SimulatedObject(ObjectState initial_state,
                    std::unique_ptr<IMotionModel> motion_model,
                    std::string label = "Target",
                    bool has_gnss = true);

    SimulatedObject(const SimulatedObject& other);
    SimulatedObject& operator=(const SimulatedObject& other);
    SimulatedObject(SimulatedObject&&) noexcept = default;
    SimulatedObject& operator=(SimulatedObject&&) noexcept = default;
    ~SimulatedObject() = default;

    /// Advances object state by dt if active at new_timestamp.
    void step(double dt, TimestampSec new_timestamp);

    /// Resets object back to its initial configured state.
    void reset();

    [[nodiscard]] ObjectId id() const noexcept { return state_.id; }
    [[nodiscard]] const std::string& label() const noexcept { return config_.label; }
    [[nodiscard]] const ObjectState& state() const noexcept { return state_; }
    /// Interpolated ground truth at a sensor's capture time, if retained in history.
    [[nodiscard]] std::optional<ObjectState> stateAt(TimestampSec timestamp) const;
    [[nodiscard]] const std::vector<ObjectState>& trajectoryHistory() const noexcept {
        return history_;
    }
    [[nodiscard]] bool hasGnssTelemetry() const noexcept { return config_.has_gnss_telemetry; }
    [[nodiscard]] bool isActiveAt(TimestampSec timestamp) const noexcept;
    [[nodiscard]] MotionModelType currentMotionModelType() const noexcept;

private:
    void applyScheduledManeuver(TimestampSec timestamp);

    SimulatedObjectConfig config_;
    ObjectState state_;
    std::unique_ptr<IMotionModel> base_motion_model_;
    std::unique_ptr<IMotionModel> active_motion_model_;
    std::vector<ObjectState> history_;
    bool spawned_{false};
};

}  // namespace aegistrack
