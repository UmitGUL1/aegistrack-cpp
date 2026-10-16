#pragma once

#include "aegistrack/core/types.hpp"

#include <cmath>
#include <optional>

namespace aegistrack {

/**
 * @brief Represents the ground-truth physical state of a simulated object.
 *
 * Coordinate frame: Local 2D Cartesian frame (East-North) in meters:
 * - px, py: Position in meters (m).
 * - vx, vy: Velocity in meters per second (m/s).
 * - ax, ay: Optional linear acceleration in meters per second squared (m/s^2).
 * - timestamp: Simulation timestamp in seconds (s).
 */
struct ObjectState {
    ObjectId id{0};
    double px{0.0};
    double py{0.0};
    double vx{0.0};
    double vy{0.0};
    std::optional<double> ax{std::nullopt};
    std::optional<double> ay{std::nullopt};
    TimestampSec timestamp{0.0};

    [[nodiscard]] Vector2d position() const noexcept {
        return Vector2d{px, py};
    }

    [[nodiscard]] Vector2d velocity() const noexcept {
        return Vector2d{vx, vy};
    }

    [[nodiscard]] Vector4d stateVector4d() const noexcept {
        return Vector4d{px, py, vx, vy};
    }

    [[nodiscard]] double speed() const noexcept {
        return std::hypot(vx, vy);
    }

    [[nodiscard]] double headingRad() const noexcept {
        return std::atan2(vy, vx);
    }

    [[nodiscard]] bool isFinite() const noexcept {
        const bool base_finite =
            std::isfinite(px) && std::isfinite(py) && std::isfinite(vx) && std::isfinite(vy) &&
            std::isfinite(timestamp);
        if (!base_finite) {
            return false;
        }
        if (ax.has_value() && !std::isfinite(*ax)) {
            return false;
        }
        if (ay.has_value() && !std::isfinite(*ay)) {
            return false;
        }
        return true;
    }
};

}  // namespace aegistrack
