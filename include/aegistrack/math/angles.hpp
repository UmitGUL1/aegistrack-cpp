#pragma once

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace aegistrack::math {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kHalfPi = 0.5 * std::numbers::pi;

/// Converts degrees to radians.
[[nodiscard]] constexpr double degToRad(double degrees) noexcept {
    return degrees * (kPi / 180.0);
}

/// Converts radians to degrees.
[[nodiscard]] constexpr double radToDeg(double radians) noexcept {
    return radians * (180.0 / kPi);
}

/**
 * @brief Normalizes an angle in radians to the half-open interval (-pi, pi].
 *
 * Essential for computing bearing innovations without wrap-around discontinuities
 * across the +/-pi branch cut.
 */
[[nodiscard]] inline double normalizeAnglePi(double angle_rad) {
    if (!std::isfinite(angle_rad)) {
        throw std::invalid_argument("normalizeAnglePi requires a finite angle");
    }
    double wrapped = std::fmod(angle_rad + kPi, kTwoPi);
    if (wrapped <= 0.0) {
        wrapped += kTwoPi;
    }
    return wrapped - kPi;
}

/**
 * @brief Normalizes an angle in radians to the interval [0, 2*pi).
 */
[[nodiscard]] inline double normalizeAngleTwoPi(double angle_rad) {
    if (!std::isfinite(angle_rad)) {
        throw std::invalid_argument("normalizeAngleTwoPi requires a finite angle");
    }
    double wrapped = std::fmod(angle_rad, kTwoPi);
    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

/**
 * @brief Returns the shortest signed angular difference (target - source) in (-pi, pi].
 */
[[nodiscard]] inline double angleDifference(double target_rad, double source_rad) {
    return normalizeAnglePi(target_rad - source_rad);
}

}  // namespace aegistrack::math
