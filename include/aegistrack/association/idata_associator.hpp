#pragma once

#include "aegistrack/core/types.hpp"
#include "aegistrack/sensors/measurement.hpp"
#include "aegistrack/tracking/track.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace aegistrack {

struct TrackMeasurementMatch {
    std::size_t track_index{0};
    std::size_t measurement_index{0};
    double mahalanobis_distance_sq{0.0};
};

struct AssociationResult {
    std::vector<TrackMeasurementMatch> matches;
    std::vector<std::size_t> unassigned_track_indices;
    std::vector<std::size_t> unassigned_measurement_indices;
    MatrixXd distance_matrix;  ///< N_tracks x M_measurements squared Mahalanobis distances
};

/**
 * @brief Strategy interface for gated multi-target data association.
 */
class IDataAssociator {
public:
    virtual ~IDataAssociator() = default;

    /**
     * @brief Associates a set of predicted tracks with a single-sensor measurement scan.
     *
     * @param tracks Active tracks predicted to the scan timestamp.
     * @param measurements Measurements belonging to a single sensor scan.
     * @param gate_threshold_2d Chi-square gate threshold for 2D measurements (default 9.2103 = 99%).
     */
    [[nodiscard]] virtual AssociationResult associate(
        std::span<const Track> tracks,
        std::span<const Measurement> measurements,
        double gate_threshold_2d = 9.21034037) const = 0;

    [[nodiscard]] virtual AssociationAlgorithmType type() const noexcept = 0;
};

}  // namespace aegistrack
