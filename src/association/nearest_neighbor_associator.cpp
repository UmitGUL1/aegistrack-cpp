#include "aegistrack/association/nearest_neighbor_associator.hpp"

#include "aegistrack/math/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace aegistrack {

namespace {

double effectiveGateForDimension(Eigen::Index dim, double gate_threshold_2d) noexcept {
    if (dim == 1) {
        // Scale 2D chi-square threshold to 1D chi-square (6.6349 / 9.2103 at 99%)
        return gate_threshold_2d * (6.63489660 / 9.21034037);
    }
    return gate_threshold_2d;
}

}  // namespace

AssociationResult NearestNeighborAssociator::associate(
    std::span<const Track> tracks,
    std::span<const Measurement> measurements,
    double gate_threshold_2d) const {
    if (!std::isfinite(gate_threshold_2d) || gate_threshold_2d <= 0.0) {
        throw std::invalid_argument(
            "NearestNeighborAssociator::associate requires finite gate_threshold_2d > 0");
    }

    const std::size_t n_tracks = tracks.size();
    const std::size_t n_meas = measurements.size();

    AssociationResult result;
    result.distance_matrix = MatrixXd::Constant(static_cast<Eigen::Index>(n_tracks),
                                                static_cast<Eigen::Index>(n_meas),
                                                math::HungarianSolver::kForbiddenCost);

    struct Candidate {
        std::size_t track_idx;
        std::size_t meas_idx;
        double d2;
    };
    std::vector<Candidate> candidates;

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
                const double dim_gate =
                    effectiveGateForDimension(measurements[j].dimension(), gate_threshold_2d);
                if (diag.nis <= dim_gate) {
                    candidates.push_back(Candidate{i, j, diag.nis});
                }
            } catch (const std::exception&) {
                continue;
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.d2 - b.d2) > 1e-12) {
            return a.d2 < b.d2;
        }
        if (a.track_idx != b.track_idx) {
            return a.track_idx < b.track_idx;
        }
        return a.meas_idx < b.meas_idx;
    });

    std::vector<bool> track_used(n_tracks, false);
    std::vector<bool> meas_used(n_meas, false);

    for (const auto& cand : candidates) {
        if (!track_used[cand.track_idx] && !meas_used[cand.meas_idx]) {
            track_used[cand.track_idx] = true;
            meas_used[cand.meas_idx] = true;
            result.matches.push_back(
                TrackMeasurementMatch{cand.track_idx, cand.meas_idx, cand.d2});
        }
    }

    std::sort(result.matches.begin(),
              result.matches.end(),
              [](const TrackMeasurementMatch& a, const TrackMeasurementMatch& b) {
                  return a.track_index < b.track_index;
              });

    for (std::size_t i = 0; i < n_tracks; ++i) {
        if (!track_used[i] && tracks[i].status() != TrackLifecycleStatus::Deleted) {
            result.unassigned_track_indices.push_back(i);
        }
    }
    for (std::size_t j = 0; j < n_meas; ++j) {
        if (!meas_used[j] && measurements[j].isConsistent()) {
            result.unassigned_measurement_indices.push_back(j);
        }
    }

    return result;
}

}  // namespace aegistrack
