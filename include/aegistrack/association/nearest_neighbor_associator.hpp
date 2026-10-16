#pragma once

#include "aegistrack/association/idata_associator.hpp"

namespace aegistrack {

/**
 * @brief Greedy Global Nearest Neighbor (GNN) baseline associator with Mahalanobis gating.
 *
 * Evaluates all gated (track, measurement) candidate pairs sorted by ascending squared
 * Mahalanobis distance and greedily assigns disjoint pairs.
 */
class NearestNeighborAssociator final : public IDataAssociator {
public:
    [[nodiscard]] AssociationResult associate(
        std::span<const Track> tracks,
        std::span<const Measurement> measurements,
        double gate_threshold_2d = 9.21034037) const override;

    [[nodiscard]] AssociationAlgorithmType type() const noexcept override {
        return AssociationAlgorithmType::NearestNeighbor;
    }
};

}  // namespace aegistrack
