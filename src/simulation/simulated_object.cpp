#include "aegistrack/simulation/simulated_object.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace aegistrack {

namespace {

std::unique_ptr<IMotionModel> createModelFromConfig(MotionModelType type,
                                                    double ax,
                                                    double ay,
                                                    double turn_rate_rad_s) {
    switch (type) {
        case MotionModelType::ConstantVelocity:
            return std::make_unique<ConstantVelocityMotionModel>();
        case MotionModelType::ConstantAcceleration:
            return std::make_unique<ConstantAccelerationMotionModel>(ax, ay);
        case MotionModelType::CoordinatedTurn:
            return std::make_unique<CoordinatedTurnMotionModel>(turn_rate_rad_s);
    }
    return std::make_unique<ConstantVelocityMotionModel>();
}

}  // namespace

SimulatedObject::SimulatedObject(SimulatedObjectConfig config)
    : config_(std::move(config)), state_(config_.initial_state) {
    state_.id = config_.id;
    if (!state_.isFinite()) {
        throw std::invalid_argument("SimulatedObject: initial_state contains non-finite values");
    }
    base_motion_model_ = createModelFromConfig(config_.initial_motion_model,
                                               state_.ax.value_or(0.0),
                                               state_.ay.value_or(0.0),
                                               config_.turn_rate_rad_s);
    active_motion_model_ = base_motion_model_->clone();
    if (isActiveAt(state_.timestamp)) {
        spawned_ = true;
        history_.push_back(state_);
    }
}

SimulatedObject::SimulatedObject(ObjectState initial_state,
                                 std::unique_ptr<IMotionModel> motion_model,
                                 std::string label,
                                 bool has_gnss)
    : state_(initial_state),
      base_motion_model_(std::move(motion_model)),
      spawned_(true) {
    if (!base_motion_model_) {
        throw std::invalid_argument("SimulatedObject requires a non-null motion_model");
    }
    if (!state_.isFinite()) {
        throw std::invalid_argument("SimulatedObject: initial_state contains non-finite values");
    }
    config_.id = state_.id;
    config_.label = std::move(label);
    config_.initial_state = state_;
    config_.initial_motion_model = base_motion_model_->type();
    config_.spawn_time = 0.0;
    config_.has_gnss_telemetry = has_gnss;
    active_motion_model_ = base_motion_model_->clone();
    history_.push_back(state_);
}

SimulatedObject::SimulatedObject(const SimulatedObject& other)
    : config_(other.config_),
      state_(other.state_),
      base_motion_model_(other.base_motion_model_ ? other.base_motion_model_->clone() : nullptr),
      active_motion_model_(other.active_motion_model_ ? other.active_motion_model_->clone()
                                                      : nullptr),
      history_(other.history_),
      spawned_(other.spawned_) {}

SimulatedObject& SimulatedObject::operator=(const SimulatedObject& other) {
    if (this != &other) {
        config_ = other.config_;
        state_ = other.state_;
        base_motion_model_ =
            other.base_motion_model_ ? other.base_motion_model_->clone() : nullptr;
        active_motion_model_ =
            other.active_motion_model_ ? other.active_motion_model_->clone() : nullptr;
        history_ = other.history_;
        spawned_ = other.spawned_;
    }
    return *this;
}

bool SimulatedObject::isActiveAt(TimestampSec timestamp) const noexcept {
    if (timestamp + 1e-9 < config_.spawn_time) {
        return false;
    }
    if (config_.despawn_time.has_value() && timestamp - 1e-9 > *config_.despawn_time) {
        return false;
    }
    return true;
}

std::optional<ObjectState> SimulatedObject::stateAt(TimestampSec timestamp) const {
    if (!std::isfinite(timestamp) || !isActiveAt(timestamp) || history_.empty()) {
        return std::nullopt;
    }
    const auto upper = std::lower_bound(history_.begin(), history_.end(), timestamp,
        [](const ObjectState& state, TimestampSec time) { return state.timestamp < time; });
    if (upper == history_.end()) {
        return std::abs(history_.back().timestamp - timestamp) <= 1e-9
                   ? std::optional<ObjectState>(history_.back()) : std::nullopt;
    }
    if (std::abs(upper->timestamp - timestamp) <= 1e-9) {
        return *upper;
    }
    if (upper == history_.begin()) {
        return std::nullopt;
    }
    const auto& lower = *std::prev(upper);
    const double dt = upper->timestamp - lower.timestamp;
    const double fraction = (timestamp - lower.timestamp) / dt;
    const double t2 = fraction * fraction;
    const double t3 = t2 * fraction;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + fraction;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    const double dh00 = 6.0 * t2 - 6.0 * fraction;
    const double dh10 = 3.0 * t2 - 4.0 * fraction + 1.0;
    const double dh01 = -dh00;
    const double dh11 = 3.0 * t2 - 2.0 * fraction;
    ObjectState result = lower;
    result.px = h00 * lower.px + h10 * dt * lower.vx +
                h01 * upper->px + h11 * dt * upper->vx;
    result.py = h00 * lower.py + h10 * dt * lower.vy +
                h01 * upper->py + h11 * dt * upper->vy;
    result.vx = (dh00 * lower.px + dh01 * upper->px) / dt +
                dh10 * lower.vx + dh11 * upper->vx;
    result.vy = (dh00 * lower.py + dh01 * upper->py) / dt +
                dh10 * lower.vy + dh11 * upper->vy;
    result.timestamp = timestamp;
    return result;
}

MotionModelType SimulatedObject::currentMotionModelType() const noexcept {
    return active_motion_model_ ? active_motion_model_->type() : config_.initial_motion_model;
}

void SimulatedObject::applyScheduledManeuver(TimestampSec timestamp) {
    for (const auto& cmd : config_.maneuvers) {
        if (timestamp + 1e-9 >= cmd.start_time && timestamp - 1e-9 <= cmd.end_time) {
            state_.ax = cmd.ax;
            state_.ay = cmd.ay;
            active_motion_model_ =
                createModelFromConfig(cmd.model_type, cmd.ax, cmd.ay, cmd.turn_rate_rad_s);
            return;
        }
    }
    state_.ax = config_.initial_state.ax;
    state_.ay = config_.initial_state.ay;
    active_motion_model_ = base_motion_model_->clone();
}

void SimulatedObject::step(double dt, TimestampSec new_timestamp) {
    if (!isActiveAt(new_timestamp)) {
        return;
    }
    if (!spawned_) {
        spawned_ = true;
        state_ = config_.initial_state;
        state_.id = config_.id;
        state_.timestamp = new_timestamp;
        history_.push_back(state_);
        return;
    }

    applyScheduledManeuver(state_.timestamp);
    state_ = active_motion_model_->propagate(state_, dt);
    state_.timestamp = new_timestamp;

    if (config_.max_history_length > 0 && history_.size() >= config_.max_history_length) {
        history_.erase(history_.begin());
    }
    history_.push_back(state_);
}

void SimulatedObject::reset() {
    state_ = config_.initial_state;
    state_.id = config_.id;
    active_motion_model_ = base_motion_model_->clone();
    history_.clear();
    spawned_ = isActiveAt(state_.timestamp);
    if (spawned_) {
        history_.push_back(state_);
    }
}

}  // namespace aegistrack
