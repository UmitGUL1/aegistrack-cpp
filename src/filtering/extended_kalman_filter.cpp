#include "aegistrack/filtering/extended_kalman_filter.hpp"

#include "aegistrack/math/angles.hpp"
#include "aegistrack/math/statistics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aegistrack {

ExtendedKalmanFilter::ExtendedKalmanFilter(const Vector4d& initial_state,
                                           const Matrix4d& initial_covariance,
                                           double process_noise_accel_std)
    : KalmanFilter(initial_state, initial_covariance, process_noise_accel_std) {}

Vector2d ExtendedKalmanFilter::predictRadarMeasurement(const Vector4d& state,
                                                       const Vector2d& sensor_pos) {
    if (!state.allFinite() || !sensor_pos.allFinite()) {
        throw std::invalid_argument(
            "ExtendedKalmanFilter::predictRadarMeasurement requires finite inputs");
    }
    const double dx = state(0) - sensor_pos.x();
    const double dy = state(1) - sensor_pos.y();
    const double range = std::hypot(dx, dy);
    const double bearing = (range < kMinSingularityRangeM) ? 0.0 : std::atan2(dy, dx);
    return Vector2d{range, bearing};
}

Matrix2x4d ExtendedKalmanFilter::computeRadarJacobian(const Vector4d& state,
                                                      const Vector2d& sensor_pos) {
    if (!state.allFinite() || !sensor_pos.allFinite()) {
        throw std::invalid_argument(
            "ExtendedKalmanFilter::computeRadarJacobian requires finite inputs");
    }
    double dx = state(0) - sensor_pos.x();
    double dy = state(1) - sensor_pos.y();
    double r2 = dx * dx + dy * dy;

    if (r2 < kMinSingularityRangeM * kMinSingularityRangeM) {
        dx = kMinSingularityRangeM;
        dy = 0.0;
        r2 = kMinSingularityRangeM * kMinSingularityRangeM;
    }
    const double r = std::sqrt(r2);

    Matrix2x4d H_j = Matrix2x4d::Zero();
    H_j(0, 0) = dx / r;
    H_j(0, 1) = dy / r;
    H_j(1, 0) = -dy / r2;
    H_j(1, 1) = dx / r2;
    return H_j;
}

FilterDiagnostics ExtendedKalmanFilter::computeInnovation(const Measurement& measurement) const {
    if (!measurement.isConsistent()) {
        throw std::invalid_argument(
            "ExtendedKalmanFilter::computeInnovation received invalid measurement");
    }

    if (measurement.sensor_type == SensorType::Gnss) {
        return KalmanFilter::computeInnovation(measurement);
    }

    if (measurement.sensor_type == SensorType::Radar) {
        const Vector2d z_pred = predictRadarMeasurement(x_, measurement.sensor_position);
        const Matrix2x4d H_j = computeRadarJacobian(x_, measurement.sensor_position);

        Vector2d innovation;
        innovation(0) = measurement.z(0) - z_pred(0);
        innovation(1) = math::angleDifference(measurement.z(1), z_pred(1));

        Matrix2d S = H_j * P_ * H_j.transpose() + measurement.R.topLeftCorner<2, 2>();
        S = 0.5 * (S + S.transpose());

        FilterDiagnostics diag;
        diag.innovation = innovation;
        diag.innovation_covariance = S;
        diag.nis = math::mahalanobisDistanceSquared(innovation, S);
        diag.valid = true;
        return diag;
    }

    if (measurement.sensor_type == SensorType::BearingOnly) {
        const Vector2d z_pred = predictRadarMeasurement(x_, measurement.sensor_position);
        const Matrix2x4d H_full = computeRadarJacobian(x_, measurement.sensor_position);
        const Eigen::Matrix<double, 1, 4> H_b = H_full.row(1);

        VectorXd innovation(1);
        innovation(0) = math::angleDifference(measurement.z(0), z_pred(1));

        MatrixXd S = H_b * P_ * H_b.transpose() + measurement.R.topLeftCorner<1, 1>();
        FilterDiagnostics diag;
        diag.innovation = innovation;
        diag.innovation_covariance = S;
        diag.nis = math::mahalanobisDistanceSquared(innovation, S);
        diag.valid = true;
        return diag;
    }

    throw std::invalid_argument("ExtendedKalmanFilter: unsupported sensor type");
}

FilterDiagnostics ExtendedKalmanFilter::updateMeasurement(const Measurement& measurement) {
    if (!measurement.isConsistent()) {
        throw std::invalid_argument(
            "ExtendedKalmanFilter::updateMeasurement received invalid measurement");
    }

    if (measurement.sensor_type == SensorType::Gnss) {
        return KalmanFilter::updateMeasurement(measurement);
    }

    if (measurement.sensor_type == SensorType::Radar) {
        const Vector2d z_pred = predictRadarMeasurement(x_, measurement.sensor_position);
        const Matrix2x4d H_j = computeRadarJacobian(x_, measurement.sensor_position);

        Vector2d innovation;
        innovation(0) = measurement.z(0) - z_pred(0);
        innovation(1) = math::angleDifference(measurement.z(1), z_pred(1));

        return applyJosephUpdate(innovation, H_j, measurement.R.topLeftCorner<2, 2>());
    }

    if (measurement.sensor_type == SensorType::BearingOnly) {
        const Vector2d z_pred = predictRadarMeasurement(x_, measurement.sensor_position);
        const Matrix2x4d H_full = computeRadarJacobian(x_, measurement.sensor_position);
        const Eigen::Matrix<double, 1, 4> H_b = H_full.row(1);

        VectorXd innovation(1);
        innovation(0) = math::angleDifference(measurement.z(0), z_pred(1));

        return applyJosephUpdate(innovation, H_b, measurement.R.topLeftCorner<1, 1>());
    }

    throw std::invalid_argument("ExtendedKalmanFilter: unsupported sensor type");
}

}  // namespace aegistrack
