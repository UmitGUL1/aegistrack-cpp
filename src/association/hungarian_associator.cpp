#include "aegistrack/association/hungarian_associator.hpp"

#include "aegistrack/math/hungarian.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace aegistrack {

AssociationResult HungarianAssociator::associate(
    std::span<const Track> tracks,
    std::span<const Measurement> measurements,
    double gate_threshold_2d) const {
    if (!std::isfinite(gate_threshold_2d) || gate_threshold_2d <= 0.0) {
        throw std::invalid_argument(
            "HungarianAssociator::associate requires finite gate_threshold_2d > 0");
    }

    const std::size_t n_tracks = tracks.size();
    const std::size_t n_meas = measurements.size();

    AssociationResult result;
    result.distance_matrix = MatrixXd::Constant(static_cast<Eigen::Index>(n_tracks),
                                                static_cast<Eigen::Index>(n_meas),
                                                math::HungarianSolver::kForbiddenCost);

    // Normalized cost matrix where 1D measurements have their NIS scaled by (gate_2d / gate_1d)
    // so a single gate_threshold_2d applies uniformly in HungarianSolver::solveGated.
    MatrixXd solver_cost = result.distance_matrix;

    for (std::size_t i = 0; i < n_tracks; ++i) {
        if (tracks[i].status() == TrackLifecycleStatus::Deleted) {
            continue;
        }
        for (std::size_t j = 0; j < n_meas; ++j) {
            if (!measurements[j].isConsistent()) {
                continue;
            }
            try {
                const FilterDiagnostics diag = tracks[i].evaluateMeasurement(measurements[j]);
                if (!diag.valid || !std::isfinite(diag.nis)) {
                    continue;
                }
                result.distance_matrix(static_cast<Eigen::Index>(i),
                                       static_cast<Eigen::Index>(j)) = diag.nis;
                double scaled_cost = diag.nis;
                if (measurements[j].dimension() == 1) {
                    scaled_cost = diag.nis * (9.21034037 / 6.63489660);
                }
                solver_cost(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
                    scaled_cost;
            } catch (const std::exception&) {
                continue;
            }
        }
    }

    const math::HungarianAssignment assignment =
        math::HungarianSolver::solveGated(solver_cost, gate_threshold_2d);

    std::vector<bool> track_matched(n_tracks, false);
    std::vector<bool> meas_matched(n_meas, false);

    for (const auto& [r, c] : assignment.matched_pairs) {
        if (tracks[r].status() == TrackLifecycleStatus::Deleted || !measurements[c].isConsistent()) {
            continue;
        }
        const double raw_d2 =
            result.distance_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
        result.matches.push_back(TrackMeasurementMatch{r, c, raw_d2});
        track_matched[r] = true;
        meas_matched[c] = true;
    }

    for (std::size_t i = 0; i < n_tracks; ++i) {
        if (!track_matched[i] && tracks[i].status() != TrackLifecycleStatus::Deleted) {
            result.unassigned_track_indices.push_back(i);
        }
    }
    for (std::size_t j = 0; j < n_meas; ++j) {
        if (!meas_matched[j] && measurements[j].isConsistent()) {
            result.unassigned_measurement_indices.push_back(j);
        }
    }

    return result;
}

}  // namespace aegistrack
