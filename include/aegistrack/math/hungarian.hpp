#pragma once

#include "aegistrack/core/types.hpp"

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace aegistrack::math {

struct HungarianAssignment {
    /// Matched (row_index, col_index) pairs that passed the gate threshold.
    std::vector<std::pair<std::size_t, std::size_t>> matched_pairs;
    /// Row indices (tracks) that were not assigned to any valid column.
    std::vector<std::size_t> unassigned_rows;
    /// Column indices (measurements) that were not assigned to any row.
    std::vector<std::size_t> unassigned_cols;
    /// Total cost of matched pairs.
    double total_cost{0.0};
};

/**
 * @brief Global minimum-cost gated bipartite assignment solver using the Kuhn-Munkres algorithm.
 *
 * Uses the Bar-Shalom augmented cost matrix formulation (N x (M + N)) where each row i has a
 * dedicated dummy unassigned column with cost = max_gate_cost, preventing forbidden (out-of-gate)
 * pairs from distorting valid assignments when tracks compete for measurements.
 */
class HungarianSolver {
public:
    static constexpr double kForbiddenCost = 1e12;

    /**
     * @brief Solves gated rectangular assignment for an N x M cost matrix.
     *
     * @param cost_matrix N x M matrix of non-negative distances (e.g., squared Mahalanobis distance).
     * @param max_gate_cost Maximum allowable cost for a valid match (e.g., chi-square gate threshold).
     *                      Any entry > max_gate_cost is treated as forbidden.
     */
    [[nodiscard]] static HungarianAssignment solveGated(const MatrixXd& cost_matrix,
                                                        double max_gate_cost);
};

}  // namespace aegistrack::math
