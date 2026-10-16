#include "aegistrack/filtering/kalman_filter.hpp"

#include "aegistrack/math/statistics.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>

namespace aegistrack {

KalmanFilter::KalmanFilter(const Vector4d& initial_state,
                           const Matrix4d& initial_covariance,
                           double process_noise_accel_std)
    : x_(initial_state),
      P_(initial_covariance),
      process_noise_accel_std_(process_noise_accel_std) {
    if (!x_.allFinite() || !P_.allFinite()) {
        throw std::invalid_argument("KalmanFilter: initial state and covariance must be finite");
    }
    if (!std::isfinite(process_noise_accel_std_) || process_noise_accel_std_ < 0.0) {
        throw std::invalid_argument("KalmanFilter: process_noise_accel_std must be >= 0");
    }
    if (!math::isPositiveSemiDefinite(P_, 1e-7)) {
        throw std::invalid_argument(
            "KalmanFilter: initial covariance must be symmetric positive semidefinite");
    }
    P_ = math::symmetrizeAndRegularizeCovariance(P_, 1e-8);
}

Matrix4d KalmanFilter::buildStateTransitionMatrix(double dt) {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument("KalmanFilter::buildStateTransitionMatrix requires dt >= 0");
    }
    Matrix4d F = Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
}

Matrix4d KalmanFilter::buildProcessNoiseMatrix(double dt, double accel_noise_std) {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument("KalmanFilter::buildProcessNoiseMatrix requires dt >= 0");
    }
    if (!std::isfinite(accel_noise_std) || accel_noise_std < 0.0) {
        throw std::invalid_argument(
            "KalmanFilter::buildProcessNoiseMatrix requires accel_noise_std >= 0");
    }

    const double q = accel_noise_std * accel_noise_std;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt * 0.5;
    const double dt4 = dt2 * dt2 * 0.25;

    Matrix4d Q = Matrix4d::Zero();
    Q(0, 0) = q * dt4;
    Q(0, 2) = q * dt3;
    Q(1, 1) = q * dt4;
    Q(1, 3) = q * dt3;
    Q(2, 0) = q * dt3;
    Q(2, 2) = q * dt2;
    Q(3, 1) = q * dt3;
    Q(3, 3) = q * dt2;
    return Q;
}

Matrix2x4d KalmanFilter::buildCartesianPositionMatrix() noexcept {
    Matrix2x4d H = Matrix2x4d::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    return H;
}

void KalmanFilter::predict(double dt) {
    if (!std::isfinite(dt) || dt < 0.0) {
        throw std::invalid_argument("KalmanFilter::predict requires finite dt >= 0");
    }
    if (dt == 0.0) {
        return;
    }

    const Matrix4d F = buildStateTransitionMatrix(dt);
    const Matrix4d Q = buildProcessNoiseMatrix(dt, process_noise_accel_std_);

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
    P_ = math::symmetrizeAndRegularizeCovariance(P_, 1e-8);
}

FilterDiagnostics KalmanFilter::computeInnovation(const Measurement& measurement) const {
    if (!measurement.isConsistent()) {
        throw std::invalid_argument("KalmanFilter::computeInnovation received invalid measurement");
    }
    if (measurement.sensor_type != SensorType::Gnss) {
        throw std::invalid_argument(
            "Linear KalmanFilter only supports linear Cartesian GNSS measurements; use EKF for Radar");
    }

    const Matrix2x4d H = buildCartesianPositionMatrix();
    FilterDiagnostics diag;
    diag.innovation = measurement.z - H * x_;
    diag.innovation_covariance =
        0.5 * ((H * P_ * H.transpose() + measurement.R) +
               (H * P_ * H.transpose() + measurement.R).transpose());
    diag.nis = math::mahalanobisDistanceSquared(diag.innovation, diag.innovation_covariance);
    diag.valid = true;
    return diag;
}

FilterDiagnostics KalmanFilter::updateMeasurement(const Measurement& measurement) {
    if (!measurement.isConsistent()) {
        throw std::invalid_argument("KalmanFilter::updateMeasurement received invalid measurement");
    }
    if (measurement.sensor_type != SensorType::Gnss) {
        throw std::invalid_argument(
            "Linear KalmanFilter only supports linear Cartesian GNSS measurements; use EKF for Radar");
    }
    const Matrix2x4d H = buildCartesianPositionMatrix();
    return updateLinear(measurement.z, H, measurement.R);
}

FilterDiagnostics KalmanFilter::updateLinear(const VectorXd& z,
                                             const MatrixXd& H,
                                             const MatrixXd& R) {
    if (z.size() == 0 || H.rows() != z.size() || H.cols() != 4 || R.rows() != z.size() ||
        R.cols() != z.size()) {
        throw std::invalid_argument("KalmanFilter::updateLinear: incompatible matrix dimensions");
    }
    if (!z.allFinite() || !H.allFinite() || !R.allFinite()) {
        throw std::invalid_argument("KalmanFilter::updateLinear: non-finite inputs");
    }
    if (!math::isPositiveSemiDefinite(R, 1e-8)) {
        throw std::invalid_argument(
            "KalmanFilter::updateLinear: measurement covariance R must be PSD");
    }

    const VectorXd innovation = z - H * x_;
    return applyJosephUpdate(innovation, H, R);
}

FilterDiagnostics KalmanFilter::applyJosephUpdate(const VectorXd& innovation,
                                                  const MatrixXd& H,
                                                  const MatrixXd& R) {
    MatrixXd S = H * P_ * H.transpose() + R;
    S = 0.5 * (S + S.transpose());

    const double nis = math::mahalanobisDistanceSquared(innovation, S);

    // Solve S * K^T = H * P^T without explicit matrix inversion
    const MatrixXd HPt = H * P_.transpose();
    MatrixXd Kt;
    Eigen::LLT<MatrixXd> llt(S);
    if (llt.info() == Eigen::Success) {
        Kt = llt.solve(HPt);
    } else {
        MatrixXd S_reg = S;
        S_reg.diagonal().array() += 1e-7;
        Eigen::LDLT<MatrixXd> ldlt(S_reg);
        if (ldlt.info() != Eigen::Success) {
            throw std::runtime_error(
                "KalmanFilter::applyJosephUpdate: failed to factorize innovation covariance S");
        }
        Kt = ldlt.solve(HPt);
    }

    const MatrixXd K = Kt.transpose();
    x_ = x_ + K * innovation;

    // Joseph stabilized covariance form:
    // P_updated = (I - K*H) * P_pred * (I - K*H)^T + K * R * K^T
    const Matrix4d I = Matrix4d::Identity();
    const Matrix4d I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    P_ = math::symmetrizeAndRegularizeCovariance(P_, 1e-8);

    last_diagnostics_.innovation = innovation;
    last_diagnostics_.innovation_covariance = S;
    last_diagnostics_.nis = nis;
    last_diagnostics_.valid = true;
    return last_diagnostics_;
}

void KalmanFilter::setState(const Vector4d& x, const Matrix4d& P) {
    if (!x.allFinite() || !P.allFinite()) {
        throw std::invalid_argument("KalmanFilter::setState requires finite state and covariance");
    }
    if (!math::isPositiveSemiDefinite(P, 1e-7)) {
        throw std::invalid_argument("KalmanFilter::setState requires PSD covariance");
    }
    x_ = x;
    P_ = math::symmetrizeAndRegularizeCovariance(P, 1e-8);
}

void KalmanFilter::setProcessNoiseAccelStd(double sigma_a) {
    if (!std::isfinite(sigma_a) || sigma_a < 0.0) {
        throw std::invalid_argument(
            "KalmanFilter::setProcessNoiseAccelStd requires finite sigma_a >= 0");
    }
    process_noise_accel_std_ = sigma_a;
}

}  // namespace aegistrack
