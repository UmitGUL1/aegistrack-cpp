#include "aegistrack/core/random.hpp"
#include "aegistrack/filtering/extended_kalman_filter.hpp"
#include "aegistrack/filtering/kalman_filter.hpp"
#include "aegistrack/math/angles.hpp"
#include "aegistrack/math/statistics.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace aegistrack::testing {

TEST(MathematicsTest, StatePredictionFollowsConstantVelocityEquations) {
    Vector4d x0;
    x0 << 100.0, -50.0, 12.5, -8.0;
    Matrix4d P0 = Matrix4d::Identity() * 10.0;

    KalmanFilter kf(x0, P0, 1.5);
    kf.predict(2.0);

    EXPECT_NEAR(kf.state()(0), 100.0 + 12.5 * 2.0, 1e-9);
    EXPECT_NEAR(kf.state()(1), -50.0 - 8.0 * 2.0, 1e-9);
    EXPECT_NEAR(kf.state()(2), 12.5, 1e-9);
    EXPECT_NEAR(kf.state()(3), -8.0, 1e-9);
}

TEST(MathematicsTest, CovariancePredictionMatchesAnalyticalFormula) {
    Vector4d x0 = Vector4d::Zero();
    Matrix4d P0 = Matrix4d::Identity() * 4.0;
    const double sigma_a = 2.0;
    const double dt = 0.5;

    KalmanFilter kf(x0, P0, sigma_a);
    kf.predict(dt);

    const Matrix4d F = KalmanFilter::buildStateTransitionMatrix(dt);
    const Matrix4d Q = KalmanFilter::buildProcessNoiseMatrix(dt, sigma_a);
    const Matrix4d expected_P = F * P0 * F.transpose() + Q;

    EXPECT_NEAR((kf.covariance() - expected_P).cwiseAbs().maxCoeff(), 0.0, 1e-9);
    EXPECT_TRUE(math::isPositiveSemiDefinite(kf.covariance()));
}

TEST(MathematicsTest, LinearKalmanUpdateReducesPositionUncertainty) {
    Vector4d x0;
    x0 << 0.0, 0.0, 10.0, 5.0;
    Matrix4d P0 = Matrix4d::Identity() * 100.0;

    KalmanFilter kf(x0, P0, 1.0);
    kf.predict(1.0);

    const double prior_trace = kf.covariance().topLeftCorner<2, 2>().trace();

    Measurement m;
    m.measurement_id = 1;
    m.sensor_id = "gnss";
    m.sensor_type = SensorType::Gnss;
    m.capture_timestamp = 1.0;
    m.arrival_timestamp = 1.0;
    m.z = Vector2d{11.0, 4.5};
    m.R = Matrix2d::Identity() * 9.0;

    const FilterDiagnostics diag = kf.updateMeasurement(m);
    EXPECT_TRUE(diag.valid);
    EXPECT_GT(diag.nis, 0.0);

    const double post_trace = kf.covariance().topLeftCorner<2, 2>().trace();
    EXPECT_LT(post_trace, prior_trace);
    EXPECT_TRUE(math::isPositiveSemiDefinite(kf.covariance()));
}

TEST(MathematicsTest, EkfRadarJacobianMatchesCentralFiniteDifferences) {
    Vector4d state;
    state << 340.0, -210.0, 15.0, 22.0;
    const Vector2d radar_pos{-50.0, 80.0};

    const Matrix2x4d analytical_H =
        ExtendedKalmanFilter::computeRadarJacobian(state, radar_pos);

    constexpr double kEps = 1e-6;
    Matrix2x4d numerical_H = Matrix2x4d::Zero();
    for (int col = 0; col < 4; ++col) {
        Vector4d state_plus = state;
        Vector4d state_minus = state;
        state_plus(col) += kEps;
        state_minus(col) -= kEps;

        const Vector2d h_plus =
            ExtendedKalmanFilter::predictRadarMeasurement(state_plus, radar_pos);
        const Vector2d h_minus =
            ExtendedKalmanFilter::predictRadarMeasurement(state_minus, radar_pos);

        numerical_H(0, col) = (h_plus(0) - h_minus(0)) / (2.0 * kEps);
        numerical_H(1, col) = math::angleDifference(h_plus(1), h_minus(1)) / (2.0 * kEps);
    }

    EXPECT_NEAR((analytical_H - numerical_H).cwiseAbs().maxCoeff(), 0.0, 1e-7);
}

TEST(MathematicsTest, EkfRadarSingularityIsHandledSafelyAtZeroRange) {
    Vector4d state;
    state << 50.0, -25.0, 10.0, 0.0;
    const Vector2d radar_pos{50.0, -25.0};

    const Vector2d h = ExtendedKalmanFilter::predictRadarMeasurement(state, radar_pos);
    const Matrix2x4d H = ExtendedKalmanFilter::computeRadarJacobian(state, radar_pos);

    EXPECT_TRUE(h.allFinite());
    EXPECT_TRUE(H.allFinite());
}

TEST(MathematicsTest, BearingNormalizationWrapsBranchCutProperly) {
    EXPECT_NEAR(math::normalizeAnglePi(0.0), 0.0, 1e-12);
    EXPECT_NEAR(math::normalizeAnglePi(math::kPi), math::kPi, 1e-12);
    EXPECT_NEAR(math::normalizeAnglePi(-math::kPi), math::kPi, 1e-12);
    EXPECT_NEAR(math::normalizeAnglePi(math::kPi + 0.2), -math::kPi + 0.2, 1e-12);
    EXPECT_NEAR(math::normalizeAnglePi(-math::kPi - 0.3), math::kPi - 0.3, 1e-12);

    // Difference across the +/- pi boundary: (+3.10 rad) and (-3.10 rad) are ~0.083 rad apart
    const double diff = math::angleDifference(3.10, -3.10);
    EXPECT_NEAR(diff, -(math::kTwoPi - 6.20), 1e-12);
}

TEST(MathematicsTest, MatrixDimensionAndNearSingularCovarianceChecks) {
    VectorXd nu(2);
    nu << 2.0, -1.0;

    MatrixXd bad_S(3, 3);
    bad_S.setIdentity();
    EXPECT_THROW((void)math::mahalanobisDistanceSquared(nu, bad_S), std::invalid_argument);

    // Near-singular rank-1 positive semi-definite matrix
    MatrixXd singular_S(2, 2);
    singular_S << 1.0, 1.0,
                  1.0, 1.0;
    const double d2 = math::mahalanobisDistanceSquared(nu, singular_S);
    EXPECT_TRUE(std::isfinite(d2));
    EXPECT_GT(d2, 0.0);

    // Non-PSD matrix with negative eigenvalue
    MatrixXd non_psd(2, 2);
    non_psd << -5.0, 0.0,
               0.0,  2.0;
    EXPECT_FALSE(math::isPositiveSemiDefinite(non_psd));
}

TEST(MathematicsTest, KalmanFilteringDemonstrablyReducesEstimationErrorOverRawMeasurements) {
    DeterministicRng rng(12345);
    Vector4d true_state;
    true_state << 0.0, 0.0, 20.0, 15.0;

    Vector4d init_est;
    init_est << 10.0, -10.0, 18.0, 14.0;
    Matrix4d init_P = Matrix4d::Identity() * 100.0;

    ExtendedKalmanFilter ekf(init_est, init_P, 0.5);

    const double dt = 0.2;
    const double meas_std = 12.0;
    double raw_sq_err_sum = 0.0;
    double filtered_sq_err_sum = 0.0;
    constexpr int kSteps = 100;

    for (int step = 1; step <= kSteps; ++step) {
        true_state(0) += true_state(2) * dt;
        true_state(1) += true_state(3) * dt;

        ekf.predict(dt);

        const Vector2d z{true_state(0) + rng.normal(0.0, meas_std),
                         true_state(1) + rng.normal(0.0, meas_std)};
        Measurement m;
        m.measurement_id = static_cast<std::uint64_t>(step);
        m.sensor_id = "gnss";
        m.sensor_type = SensorType::Gnss;
        m.capture_timestamp = step * dt;
        m.arrival_timestamp = step * dt;
        m.z = z;
        m.R = Matrix2d::Identity() * (meas_std * meas_std);

        ekf.updateMeasurement(m);

        if (step > 10) {
            raw_sq_err_sum += (z - true_state.head<2>()).squaredNorm();
            filtered_sq_err_sum += (ekf.state().head<2>() - true_state.head<2>()).squaredNorm();
        }
    }

    const double raw_rmse = std::sqrt(raw_sq_err_sum / 90.0);
    const double filtered_rmse = std::sqrt(filtered_sq_err_sum / 90.0);
    EXPECT_LT(filtered_rmse, raw_rmse * 0.55);
}

}  // namespace aegistrack::testing
