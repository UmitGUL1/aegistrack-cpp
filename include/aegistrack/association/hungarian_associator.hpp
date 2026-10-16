#pragma once

#include "aegistrack/association/idata_associator.hpp"

namespace aegistrack {

/**
 * @brief Optimal global assignment associator using Mahalanobis gating and the Hungarian algorithm.
 */
class HungarianAssociator final : public IDataAssociator {
public:
    [[nodiscard]] AssociationResult associate(
        std::span<const Track> tracks,
        std::span<const Measurement> measurements,
        double gate_threshold_2d = 9.21034037) const override;

    [[nodiscard]] AssociationAlgorithmType type() const noexcept override {
        return AssociationAlgorithmType::Hungarian;
    }
};

}  // namespace aegistrack
