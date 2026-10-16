#pragma once

#include "aegistrack/core/object_state.hpp"
#include "aegistrack/core/types.hpp"

#include <memory>

namespace aegistrack {

/**
 * @brief Strategy interface for propagating ground-truth object kinematics.
 */
class IMotionModel {
public:
    virtual ~IMotionModel() = default;

    [[nodiscard]] virtual ObjectState propagate(const ObjectState& current, double dt) const = 0;
    [[nodiscard]] virtual MotionModelType type() const noexcept = 0;
    [[nodiscard]] virtual std::unique_ptr<IMotionModel> clone() const = 0;
};

/**
 * @brief 2D Constant Velocity (CV) kinematic model.
 */
class ConstantVelocityMotionModel final : public IMotionModel {
public:
    [[nodiscard]] ObjectState propagate(const ObjectState& current, double dt) const override;
    [[nodiscard]] MotionModelType type() const noexcept override {
        return MotionModelType::ConstantVelocity;
    }
    [[nodiscard]] std::unique_ptr<IMotionModel> clone() const override {
        return std::make_unique<ConstantVelocityMotionModel>(*this);
    }
};

/**
 * @brief 2D Constant Acceleration (CA) kinematic model.
 */
class ConstantAccelerationMotionModel final : public IMotionModel {
public:
    ConstantAccelerationMotionModel(double ax = 0.0, double ay = 0.0) noexcept
        : default_ax_(ax), default_ay_(ay) {}

    [[nodiscard]] ObjectState propagate(const ObjectState& current, double dt) const override;
    [[nodiscard]] MotionModelType type() const noexcept override {
        return MotionModelType::ConstantAcceleration;
    }
    [[nodiscard]] std::unique_ptr<IMotionModel> clone() const override {
        return std::make_unique<ConstantAccelerationMotionModel>(*this);
    }

    [[nodiscard]] double defaultAx() const noexcept { return default_ax_; }
    [[nodiscard]] double defaultAy() const noexcept { return default_ay_; }

private:
    double default_ax_;
    double default_ay_;
};

/**
 * @brief 2D Coordinated Turn (CT) kinematic model with exact circular-arc integration.
 */
class CoordinatedTurnMotionModel final : public IMotionModel {
public:
    explicit CoordinatedTurnMotionModel(double turn_rate_rad_s) noexcept
        : turn_rate_rad_s_(turn_rate_rad_s) {}

    [[nodiscard]] ObjectState propagate(const ObjectState& current, double dt) const override;
    [[nodiscard]] MotionModelType type() const noexcept override {
        return MotionModelType::CoordinatedTurn;
    }
    [[nodiscard]] std::unique_ptr<IMotionModel> clone() const override {
        return std::make_unique<CoordinatedTurnMotionModel>(*this);
    }

    [[nodiscard]] double turnRateRadPerSec() const noexcept { return turn_rate_rad_s_; }
    void setTurnRateRadPerSec(double omega) noexcept { turn_rate_rad_s_ = omega; }

private:
    double turn_rate_rad_s_;
};

}  // namespace aegistrack
