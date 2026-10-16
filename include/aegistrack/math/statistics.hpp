#pragma once

#include "aegistrack/core/types.hpp"

namespace aegistrack::math {

/**
 * @brief Geometric parameters of a 2D uncertainty ellipse derived from a 2x2 position covariance.
 */
struct CovarianceEllipse2D {
    double semi_major_m{0.0};
    double semi_minor_m{0.0};
    double orientation_rad{0.0};
};

/**
 * @brief Computes squared Mahalanobis distance d^2 = nu^T * S^-1 * nu using Cholesky/LDLT solver.
 *
 * Avoids explicit matrix inversion. If S is near-singular, applies a small diagonal
 * regularization floor before solving.
 *
 * @throws std::invalid_argument if dimensions mismatch, inputs are non-finite, or S is non-positive.
 */
[[nodiscard]] double mahalanobisDistanceSquared(const VectorXd& innovation,
                                                const MatrixXd& innovation_covariance);

/**
 * @brief Checks if a square matrix is symmetric positive semidefinite within numerical tolerance.
 */
[[nodiscard]] bool isPositiveSemiDefinite(const MatrixXd& matrix, double tolerance = 1e-9);

/**
 * @brief Enforces symmetry P_sym = 0.5 * (P + P^T) and clamps minimum diagonal variance.
 */
[[nodiscard]] MatrixXd symmetrizeAndRegularizeCovariance(const MatrixXd& covariance,
                                                         double min_diagonal = 1e-9);

/**
 * @brief Returns the chi-squared gate threshold gamma for a given degrees of freedom (1..4)
 * and confidence probability (e.g. 0.95, 0.99, 0.997).
 */
[[nodiscard]] double chiSquareThreshold(int degrees_of_freedom, double confidence_probability = 0.99);

/**
 * @brief Computes the 2D covariance ellipse axes and orientation for a 2x2 position covariance.
 *
 * Default chi_square_scale = 5.99146 corresponds to 95% confidence in 2D.
 */
[[nodiscard]] CovarianceEllipse2D computeCovarianceEllipse(const Matrix2d& position_covariance,
                                                           double chi_square_scale = 5.991464547);

}  // namespace aegistrack::math
