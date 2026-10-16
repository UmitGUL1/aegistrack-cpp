#include "aegistrack/math/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace aegistrack::math {

HungarianAssignment HungarianSolver::solveGated(const MatrixXd& cost_matrix, double max_gate_cost) {
    if (!std::isfinite(max_gate_cost) || max_gate_cost < 0.0) {
        throw std::invalid_argument("HungarianSolver::solveGated requires finite max_gate_cost >= 0");
    }

    const auto num_rows = static_cast<std::size_t>(cost_matrix.rows());
    const auto num_cols = static_cast<std::size_t>(cost_matrix.cols());

    HungarianAssignment result;
    if (num_rows == 0) {
        result.unassigned_cols.resize(num_cols);
        for (std::size_t j = 0; j < num_cols; ++j) {
            result.unassigned_cols[j] = j;
        }
        return result;
    }
    if (num_cols == 0) {
        result.unassigned_rows.resize(num_rows);
        for (std::size_t i = 0; i < num_rows; ++i) {
            result.unassigned_rows[i] = i;
        }
        return result;
    }

    // Augmented matrix has N rows and W = M + N columns.
    // Columns [0, M-1] represent real measurements.
    // Column M + i represents the dummy "unassigned" option for row i.
    // Setting dummy_cost = (N + 1) * (max_gate_cost + 1.0) guarantees maximum-cardinality
    // matching among admissible gated pairs (cost <= max_gate_cost) while minimizing total
    // Mahalanobis distance, and forbidden_penalty >> N * dummy_cost prevents any out-of-gate pair.
    const std::size_t aug_cols = num_cols + num_rows;
    const double dummy_cost =
        static_cast<double>(num_rows + 1) * (max_gate_cost + 1.0);
    const double forbidden_penalty =
        std::max(kForbiddenCost, static_cast<double>(num_rows + 2) * dummy_cost * 100.0);

    MatrixXd aug_cost = MatrixXd::Constant(static_cast<Eigen::Index>(num_rows),
                                           static_cast<Eigen::Index>(aug_cols),
                                           forbidden_penalty);

    for (std::size_t i = 0; i < num_rows; ++i) {
        for (std::size_t j = 0; j < num_cols; ++j) {
            const double c =
                cost_matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
            if (std::isfinite(c) && c >= 0.0 && c <= max_gate_cost) {
                aug_cost(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = c;
            }
        }
        aug_cost(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(num_cols + i)) =
            dummy_cost;
    }

    // 1-indexed Kuhn-Munkres / Jonker-Volgenant shortest augmenting path algorithm
    // for rectangular N x W matrix where N <= W.
    std::vector<double> u(num_rows + 1, 0.0);
    std::vector<double> v(aug_cols + 1, 0.0);
    std::vector<std::size_t> p(aug_cols + 1, 0);
    std::vector<std::size_t> way(aug_cols + 1, 0);

    for (std::size_t i = 1; i <= num_rows; ++i) {
        p[0] = i;
        std::size_t j0 = 0;
        std::vector<double> minv(aug_cols + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(aug_cols + 1, false);

        do {
            used[j0] = true;
            const std::size_t i0 = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            std::size_t j1 = 0;

            for (std::size_t j = 1; j <= aug_cols; ++j) {
                if (!used[j]) {
                    const double cur =
                        aug_cost(static_cast<Eigen::Index>(i0 - 1),
                                 static_cast<Eigen::Index>(j - 1)) -
                        u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (std::size_t j = 0; j <= aug_cols; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const std::size_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<bool> row_matched(num_rows, false);
    std::vector<bool> col_matched(num_cols, false);

    for (std::size_t j = 1; j <= num_cols; ++j) {
        if (p[j] != 0) {
            const std::size_t row_idx = p[j] - 1;
            const std::size_t col_idx = j - 1;
            const double original_cost = cost_matrix(static_cast<Eigen::Index>(row_idx),
                                                     static_cast<Eigen::Index>(col_idx));
            if (std::isfinite(original_cost) && original_cost >= 0.0 &&
                original_cost <= max_gate_cost) {
                result.matched_pairs.emplace_back(row_idx, col_idx);
                result.total_cost += original_cost;
                row_matched[row_idx] = true;
                col_matched[col_idx] = true;
            }
        }
    }

    std::sort(result.matched_pairs.begin(), result.matched_pairs.end());

    for (std::size_t i = 0; i < num_rows; ++i) {
        if (!row_matched[i]) {
            result.unassigned_rows.push_back(i);
        }
    }
    for (std::size_t j = 0; j < num_cols; ++j) {
        if (!col_matched[j]) {
            result.unassigned_cols.push_back(j);
        }
    }

    return result;
}

}  // namespace aegistrack::math
