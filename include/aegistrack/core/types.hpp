#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <string_view>

namespace aegistrack {

/// Unique internal simulation object identifier (ground truth).
/// Never exposed to IDataAssociator, TrackManager, or IStateEstimator.
using ObjectId = std::uint64_t;

/// Stable tracker-assigned identifier for an estimated trajectory.
using TrackId = std::uint64_t;

/// Unique identifier for a sensor instance (e.g., "radar_north", "gnss_1").
using SensorId = std::string;

/// Simulation timestamp in seconds relative to scenario start (t = 0.0 s).
using TimestampSec = double;

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;
using Matrix2x4d = Eigen::Matrix<double, 2, 4>;
using Matrix4x2d = Eigen::Matrix<double, 4, 2>;

/// Supported sensor modalities.
enum class SensorType {
    Radar,       ///< 2D polar measurement: [range (m), bearing (rad)]^T
    Gnss,        ///< 2D Cartesian telemetry: [px (m), py (m)]^T
    BearingOnly  ///< 1D passive bearing measurement: [bearing (rad)]
};

/// Lifecycle state of a tracked object.
enum class TrackLifecycleStatus {
    Tentative,  ///< Newly initiated track awaiting confirmation hits
    Confirmed,  ///< Actively tracked object with sufficient measurement support
    Coasting,   ///< Confirmed track temporarily lacking measurements (predicting)
    Deleted     ///< Expired track scheduled for removal/archival
};

/// Kinematic motion model type for ground-truth simulation.
enum class MotionModelType {
    ConstantVelocity,
    ConstantAcceleration,
    CoordinatedTurn
};

/// Multi-target data association algorithm type.
enum class AssociationAlgorithmType {
    NearestNeighbor,  ///< Greedy Mahalanobis-gated nearest neighbor assignment
    Hungarian         ///< Global minimum-cost gated assignment (Kuhn-Munkres)
};

/// Policy for handling out-of-sequence measurements (OOSM) caused by sensor latency.
enum class OutOfSequencePolicy {
    DropStale,             ///< Reject measurements older than the latest processed timestamp
    BoundedReorderBuffer   ///< Buffer measurements within a time window and sort by capture time
};

/// Converts SensorType to a human-readable string view.
[[nodiscard]] constexpr std::string_view toString(SensorType type) noexcept {
    switch (type) {
        case SensorType::Radar:
            return "Radar";
        case SensorType::Gnss:
            return "GNSS";
        case SensorType::BearingOnly:
            return "BearingOnly";
    }
    return "Unknown";
}

/// Converts TrackLifecycleStatus to a human-readable string view.
[[nodiscard]] constexpr std::string_view toString(TrackLifecycleStatus status) noexcept {
    switch (status) {
        case TrackLifecycleStatus::Tentative:
            return "Tentative";
        case TrackLifecycleStatus::Confirmed:
            return "Confirmed";
        case TrackLifecycleStatus::Coasting:
            return "Coasting";
        case TrackLifecycleStatus::Deleted:
            return "Deleted";
    }
    return "Unknown";
}

/// Converts MotionModelType to a human-readable string view.
[[nodiscard]] constexpr std::string_view toString(MotionModelType model) noexcept {
    switch (model) {
        case MotionModelType::ConstantVelocity:
            return "ConstantVelocity";
        case MotionModelType::ConstantAcceleration:
            return "ConstantAcceleration";
        case MotionModelType::CoordinatedTurn:
            return "CoordinatedTurn";
    }
    return "Unknown";
}

/// Converts AssociationAlgorithmType to a human-readable string view.
[[nodiscard]] constexpr std::string_view toString(AssociationAlgorithmType alg) noexcept {
    switch (alg) {
        case AssociationAlgorithmType::NearestNeighbor:
            return "NearestNeighbor";
        case AssociationAlgorithmType::Hungarian:
            return "Hungarian";
    }
    return "Unknown";
}

}  // namespace aegistrack
