#include "aegistrack/simulation/motion_model.hpp"

#include <cmath>
#include <stdexcept>

namespace aegistrack {

ObjectState ConstantVelocityMotionModel::propagate(const ObjectState& current, double dt) const {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument("ConstantVelocityMotionModel::propagate requires finite dt >= 0");
    }
    if (!current.isFinite()) {
        throw std::invalid_argument("ConstantVelocityMotionModel::propagate requires finite state");
    }

    ObjectState next = current;
    next.px = current.px + current.vx * dt;
    next.py = current.py + current.vy * dt;
    next.timestamp = current.timestamp + dt;
    return next;
}

ObjectState ConstantAccelerationMotionModel::propagate(const ObjectState& current,
                                                       double dt) const {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument(
            "ConstantAccelerationMotionModel::propagate requires finite dt >= 0");
    }
    if (!current.isFinite()) {
        throw std::invalid_argument(
            "ConstantAccelerationMotionModel::propagate requires finite state");
    }

    const double ax = current.ax.value_or(default_ax_);
    const double ay = current.ay.value_or(default_ay_);

    ObjectState next = current;
    next.px = current.px + current.vx * dt + 0.5 * ax * dt * dt;
    next.py = current.py + current.vy * dt + 0.5 * ay * dt * dt;
    next.vx = current.vx + ax * dt;
    next.vy = current.vy + ay * dt;
    next.ax = ax;
    next.ay = ay;
    next.timestamp = current.timestamp + dt;
    return next;
}

ObjectState CoordinatedTurnMotionModel::propagate(const ObjectState& current, double dt) const {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument("CoordinatedTurnMotionModel::propagate requires finite dt >= 0");
    }
    if (!current.isFinite() || !std::isfinite(turn_rate_rad_s_)) {
        throw std::invalid_argument("CoordinatedTurnMotionModel::propagate requires finite inputs");
    }

    ObjectState next = current;
    next.timestamp = current.timestamp + dt;

    constexpr double kSmallOmegaEps = 1e-7;
    if (std::abs(turn_rate_rad_s_) <= kSmallOmegaEps) {
        next.px = current.px + current.vx * dt;
        next.py = current.py + current.vy * dt;
        return next;
    }

    const double omega = turn_rate_rad_s_;
    const double theta = omega * dt;
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);

    next.px = current.px + (sin_theta / omega) * current.vx -
              ((1.0 - cos_theta) / omega) * current.vy;
    next.py = current.py + ((1.0 - cos_theta) / omega) * current.vx +
              (sin_theta / omega) * current.vy;
    next.vx = cos_theta * current.vx - sin_theta * current.vy;
    next.vy = sin_theta * current.vx + cos_theta * current.vy;
    next.ax = -omega * next.vy;
    next.ay = omega * next.vx;
    return next;
}

}  // namespace aegistrack
