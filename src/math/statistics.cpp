#include "aegistrack/math/statistics.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aegistrack::math {

double mahalanobisDistanceSquared(const VectorXd& innovation,
                                  const MatrixXd& innovation_covariance) {
    const auto dim = innovation.size();
    if (dim == 0) {
        throw std::invalid_argument("mahalanobisDistanceSquared: innovation vector is empty");
    }
    if (innovation_covariance.rows() != dim || innovation_covariance.cols() != dim) {
        throw std::invalid_argument(
            "mahalanobisDistanceSquared: covariance dimensions do not match innovation");
    }
    if (!innovation.allFinite() || !innovation_covariance.allFinite()) {
        throw std::invalid_argument("mahalanobisDistanceSquared: inputs contain non-finite values");
    }

    MatrixXd S = 0.5 * (innovation_covariance + innovation_covariance.transpose());

    Eigen::LLT<MatrixXd> llt(S);
    if (llt.info() == Eigen::Success) {
        const VectorXd solved = llt.solve(innovation);
        if (solved.allFinite()) {
            const double d2 = innovation.dot(solved);
            return std::max(0.0, d2);
        }
    }

    // Fallback for near-singular innovation covariance: add small Tikhonov diagonal regularization
    constexpr double kRegularizationEps = 1e-7;
    S.diagonal().array() += kRegularizationEps;

    Eigen::LDLT<MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        throw std::invalid_argument(
            "mahalanobisDistanceSquared: innovation covariance is not positive definite");
    }

    const VectorXd solved = ldlt.solve(innovation);
    if (!solved.allFinite()) {
        throw std::invalid_argument(
            "mahalanobisDistanceSquared: linear solve produced non-finite result");
    }
    const double d2 = innovation.dot(solved);
    if (d2 < -1e-6) {
        throw std::invalid_argument(
            "mahalanobisDistanceSquared: negative quadratic form indicates non-PSD matrix");
    }
    return std::max(0.0, d2);
}

bool isPositiveSemiDefinite(const MatrixXd& matrix, double tolerance) {
    if (matrix.rows() == 0 || matrix.rows() != matrix.cols() || !matrix.allFinite()) {
        return false;
    }
    const double symmetry_error = (matrix - matrix.transpose()).cwiseAbs().maxCoeff();
    if (symmetry_error > std::max(1e-6, tolerance * 100.0)) {
        return false;
    }
    const MatrixXd sym = 0.5 * (matrix + matrix.transpose());
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig(sym, Eigen::EigenvaluesOnly);
    if (eig.info() != Eigen::Success) {
        return false;
    }
    return eig.eigenvalues().minCoeff() >= -tolerance;
}

MatrixXd symmetrizeAndRegularizeCovariance(const MatrixXd& covariance, double min_diagonal) {
    if (covariance.rows() != covariance.cols() || covariance.rows() == 0) {
        throw std::invalid_argument("symmetrizeAndRegularizeCovariance requires non-empty square matrix");
    }
    if (!covariance.allFinite()) {
        throw std::invalid_argument("symmetrizeAndRegularizeCovariance: non-finite matrix entries");
    }
    MatrixXd sym = 0.5 * (covariance + covariance.transpose());
    for (Eigen::Index i = 0; i < sym.rows(); ++i) {
        if (sym(i, i) < min_diagonal) {
            sym(i, i) = min_diagonal;
        }
    }
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig(sym);
    if (eig.info() == Eigen::Success) {
        VectorXd evals = eig.eigenvalues();
        bool clamped = false;
        for (Eigen::Index i = 0; i < evals.size(); ++i) {
            if (evals(i) < min_diagonal) {
                evals(i) = min_diagonal;
                clamped = true;
            }
        }
        if (clamped) {
            sym = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();
            sym = 0.5 * (sym + sym.transpose());
        }
    }
    return sym;
}

double chiSquareThreshold(int degrees_of_freedom, double confidence_probability) {
    if (degrees_of_freedom <= 0) {
        throw std::invalid_argument("chiSquareThreshold: degrees_of_freedom must be >= 1");
    }
    if (!std::isfinite(confidence_probability) || confidence_probability <= 0.0 ||
        confidence_probability >= 1.0) {
        throw std::invalid_argument("chiSquareThreshold: confidence_probability must be in (0, 1)");
    }

    // Exact quantiles for common measurement dimensions (1D Bearing, 2D Radar/GNSS, 3D, 4D)
    if (degrees_of_freedom == 1) {
        if (std::abs(confidence_probability - 0.90) < 1e-3) return 2.70554345;
        if (std::abs(confidence_probability - 0.95) < 1e-3) return 3.84145882;
        if (std::abs(confidence_probability - 0.99) < 1e-3) return 6.63489660;
        if (std::abs(confidence_probability - 0.997) < 1e-3) return 8.80746835;
        if (std::abs(confidence_probability - 0.999) < 1e-3) return 10.82756617;
    } else if (degrees_of_freedom == 2) {
        // For DOF = 2, exact inverse CDF of exponential distribution: -2 * ln(1 - p)
        return -2.0 * std::log(1.0 - confidence_probability);
    } else if (degrees_of_freedom == 3) {
        if (std::abs(confidence_probability - 0.90) < 1e-3) return 6.25138863;
        if (std::abs(confidence_probability - 0.95) < 1e-3) return 7.81472790;
        if (std::abs(confidence_probability - 0.99) < 1e-3) return 11.34486673;
        if (std::abs(confidence_probability - 0.997) < 1e-3) return 13.93136966;
        if (std::abs(confidence_probability - 0.999) < 1e-3) return 16.26623620;
    } else if (degrees_of_freedom == 4) {
        if (std::abs(confidence_probability - 0.90) < 1e-3) return 7.77944034;
        if (std::abs(confidence_probability - 0.95) < 1e-3) return 9.48772904;
        if (std::abs(confidence_probability - 0.99) < 1e-3) return 13.27670414;
        if (std::abs(confidence_probability - 0.997) < 1e-3) return 16.01383335;
        if (std::abs(confidence_probability - 0.999) < 1e-3) return 18.46682695;
    }

    // Beasley-Springer-Moro normal quantile + Wilson-Hilferty cube-root approximation
    const double p = confidence_probability;
    const double t = std::sqrt(-2.0 * std::log(1.0 - p));
    const double c0 = 2.515517;
    const double c1 = 0.802853;
    const double c2 = 0.010328;
    const double d1 = 1.432788;
    const double d2 = 0.189269;
    const double d3 = 0.001308;
    const double z = t - (c0 + c1 * t + c2 * t * t) / (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);

    const double k = static_cast<double>(degrees_of_freedom);
    const double term = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
    return k * std::pow(std::max(0.0, term), 3.0);
}

CovarianceEllipse2D computeCovarianceEllipse(const Matrix2d& position_covariance,
                                             double chi_square_scale) {
    if (!position_covariance.allFinite() || !std::isfinite(chi_square_scale) ||
        chi_square_scale <= 0.0) {
        return CovarianceEllipse2D{};
    }
    const Matrix2d sym = 0.5 * (position_covariance + position_covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix2d> solver(sym);
    if (solver.info() != Eigen::Success) {
        return CovarianceEllipse2D{};
    }
    const double lambda_min = std::max(0.0, solver.eigenvalues()(0));
    const double lambda_max = std::max(0.0, solver.eigenvalues()(1));
    const Vector2d major_axis_vec = solver.eigenvectors().col(1);

    CovarianceEllipse2D ellipse;
    ellipse.semi_major_m = std::sqrt(chi_square_scale * lambda_max);
    ellipse.semi_minor_m = std::sqrt(chi_square_scale * lambda_min);
    ellipse.orientation_rad = std::atan2(major_axis_vec.y(), major_axis_vec.x());
    return ellipse;
}

}  // namespace aegistrack::math
