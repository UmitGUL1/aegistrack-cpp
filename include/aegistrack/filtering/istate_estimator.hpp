#pragma once

#include "aegistrack/core/types.hpp"
#include "aegistrack/sensors/measurement.hpp"

#include <memory>

namespace aegistrack {

/**
 * @brief Diagnostic output from evaluating or incorporating a measurement in a state estimator.
 */
struct FilterDiagnostics {
    VectorXd innovation;
    MatrixXd innovation_covariance;
    double nis{0.0};  ///< Normalized Innovation Squared (squared Mahalanobis distance d^2)
    bool valid{false};
};

/**
 * @brief Interface for state estimation filters (Kalman Filter, Extended Kalman Filter).
 */
class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;

    /// Predicts state and state covariance forward by dt seconds.
    virtual void predict(double dt) = 0;

    /// Computes innovation, innovation covariance S, and NIS (d^2) without mutating filter state.
    [[nodiscard]] virtual FilterDiagnostics computeInnovation(
        const Measurement& measurement) const = 0;

    /// Incorporates a measurement and updates state and covariance using Joseph stabilized form.
    virtual FilterDiagnostics updateMeasurement(const Measurement& measurement) = 0;

    [[nodiscard]] virtual const Vector4d& state() const noexcept = 0;
    [[nodiscard]] virtual const Matrix4d& covariance() const noexcept = 0;
    virtual void setState(const Vector4d& x, const Matrix4d& P) = 0;

    [[nodiscard]] virtual double processNoiseAccelStd() const noexcept = 0;
    virtual void setProcessNoiseAccelStd(double sigma_a) = 0;

    [[nodiscard]] virtual const FilterDiagnostics& lastDiagnostics() const noexcept = 0;
    [[nodiscard]] virtual std::unique_ptr<IStateEstimator> clone() const = 0;
};

}  // namespace aegistrack
