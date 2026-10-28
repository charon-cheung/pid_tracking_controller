#include "fixed_heading_yaw_gate.hpp"

#include <cmath>
#include <stdexcept>

#include "angles/angles.h"

namespace pid_tracking_controller
{
namespace
{
// Numerical tolerance, not a deadband for intentional path heading changes (rad/m).
constexpr double kFixedYawRateTolerance = 1e-9;
constexpr double kTargetYawTolerance = 1e-9;
}  // namespace

FixedHeadingYawGate::FixedHeadingYawGate(const FixedHeadingYawGateConfig & config)
: config_(config)
{
  if (!std::isfinite(config.enter_error) || !std::isfinite(config.exit_error) ||
    config.exit_error <= 0.0 || config.enter_error <= config.exit_error ||
    config.enter_error > std::acos(-1.0))
  {
    throw std::invalid_argument("fixed-heading yaw thresholds require 0 < exit < enter <= pi");
  }
}

void FixedHeadingYawGate::reset()
{
  target_yaw_.reset();
  correcting_ = false;
  suppressing_ = false;
}

void FixedHeadingYawGate::apply(
  const HolonomicTrackingReference & reference, HolonomicRawCommand & command)
{
  if (!std::isfinite(reference.current.target_yaw) ||
    !std::isfinite(reference.current.target_yaw_rate_per_meter) ||
    !std::isfinite(command.yaw_error))
  {
    throw std::domain_error("fixed-heading yaw gate received non-finite input");
  }
  if (!config_.enabled || reference.mode != HolonomicReferenceMode::kTranslation ||
    std::abs(reference.current.target_yaw_rate_per_meter) > kFixedYawRateTolerance)
  {
    reset();
    return;
  }

  if (!target_yaw_ || std::abs(angles::normalize_angle(
      reference.current.target_yaw - *target_yaw_)) > kTargetYawTolerance)
  {
    correcting_ = false;
    target_yaw_ = reference.current.target_yaw;
  }
  // The control law already normalizes the error; preserve threshold equality.
  const double error = std::abs(command.yaw_error);
  if (error >= config_.enter_error) {
    correcting_ = true;
  } else if (error <= config_.exit_error) {
    correcting_ = false;
  }
  suppressing_ = !correcting_;
  if (suppressing_) {
    command.omega_ff = 0.0;
    command.omega_fb = 0.0;
    command.omega = 0.0;
  }
}

}  // namespace pid_tracking_controller
