#pragma once

#include "aegistrack/filtering/istate_estimator.hpp"

namespace aegistrack {

/**
 * @brief Linear Kalman Filter for a 2D Constant-Velocity state [px, py, vx, vy]^T.
 *
 * Uses continuous-time white acceleration process noise Q(dt, sigma_a^2),
 * Cholesky/LDLT decomposition for Kalman gain solving, and the Joseph stabilized
 * covariance update formula:
 *   P_updated = (I - K*H) * P_pred * (I - K*H)^T + K * R * K^T
 */
class KalmanFilter : public IStateEstimator {
public:
    explicit KalmanFilter(const Vector4d& initial_state = Vector4d::Zero(),
                          const Matrix4d& initial_covariance = Matrix4d::Identity() * 100.0,
                          double process_noise_accel_std = 1.5);

    void predict(double dt) override;

    [[nodiscard]] FilterDiagnostics computeInnovation(
        const Measurement& measurement) const override;

    FilterDiagnostics updateMeasurement(const Measurement& measurement) override;

    /// Direct linear measurement update for custom H and R matrices.
    FilterDiagnostics updateLinear(const VectorXd& z, const MatrixXd& H, const MatrixXd& R);

    [[nodiscard]] const Vector4d& state() const noexcept override { return x_; }
    [[nodiscard]] const Matrix4d& covariance() const noexcept override { return P_; }
    void setState(const Vector4d& x, const Matrix4d& P) override;

    [[nodiscard]] double processNoiseAccelStd() const noexcept override {
        return process_noise_accel_std_;
    }
    void setProcessNoiseAccelStd(double sigma_a) override;

    [[nodiscard]] const FilterDiagnostics& lastDiagnostics() const noexcept override {
        return last_diagnostics_;
    }

    [[nodiscard]] std::unique_ptr<IStateEstimator> clone() const override {
        return std::make_unique<KalmanFilter>(*this);
    }

    [[nodiscard]] static Matrix4d buildStateTransitionMatrix(double dt);
    [[nodiscard]] static Matrix4d buildProcessNoiseMatrix(double dt, double accel_noise_std);
    [[nodiscard]] static Matrix2x4d buildCartesianPositionMatrix() noexcept;

protected:
    FilterDiagnostics applyJosephUpdate(const VectorXd& innovation,
                                        const MatrixXd& H,
                                        const MatrixXd& R);

    Vector4d x_;
    Matrix4d P_;
    double process_noise_accel_std_;
    FilterDiagnostics last_diagnostics_;
};

}  // namespace aegistrack
