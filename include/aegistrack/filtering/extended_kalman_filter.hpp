#pragma once

#include "aegistrack/filtering/kalman_filter.hpp"

namespace aegistrack {

/**
 * @brief Extended Kalman Filter (EKF) supporting both linear Cartesian (GNSS)
 * and nonlinear polar (Radar range/bearing, Bearing-only) measurement updates.
 */
class ExtendedKalmanFilter final : public KalmanFilter {
public:
    static constexpr double kMinSingularityRangeM = 1e-4;

    explicit ExtendedKalmanFilter(
        const Vector4d& initial_state = Vector4d::Zero(),
        const Matrix4d& initial_covariance = Matrix4d::Identity() * 100.0,
        double process_noise_accel_std = 1.5);

    [[nodiscard]] FilterDiagnostics computeInnovation(
        const Measurement& measurement) const override;

    FilterDiagnostics updateMeasurement(const Measurement& measurement) override;

    [[nodiscard]] std::unique_ptr<IStateEstimator> clone() const override {
        return std::make_unique<ExtendedKalmanFilter>(*this);
    }

    /**
     * @brief Nonlinear radar measurement function h(x) = [range, bearing]^T relative to sensor_pos.
     */
    [[nodiscard]] static Vector2d predictRadarMeasurement(const Vector4d& state,
                                                          const Vector2d& sensor_pos);

    /**
     * @brief Analytical Jacobian dh/dx (2x4) of polar radar measurement with singularity guard.
     */
    [[nodiscard]] static Matrix2x4d computeRadarJacobian(const Vector4d& state,
                                                         const Vector2d& sensor_pos);
};

}  // namespace aegistrack
