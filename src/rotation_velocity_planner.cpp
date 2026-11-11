#include "rotation_velocity_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pid_tracking_controller
{

RotationVelocityPlanner::RotationVelocityPlanner(const RotationVelocityPlannerConfig & config)
: config_(config)
{
  if (!std::isfinite(config.kp_yaw) || config.kp_yaw <= 0.0 ||
    !std::isfinite(config.response_delay) || config.response_delay < 0.0 ||
    !std::isfinite(config.max_omega) || config.max_omega <= 0.0 ||
    !std::isfinite(config.max_deceleration) || config.max_deceleration <= 0.0 ||
    !std::isfinite(config.yaw_tolerance) || config.yaw_tolerance < 0.0)
  {
    throw std::invalid_argument("invalid rotation velocity planner configuration");
  }
}

RotationVelocityPlan RotationVelocityPlanner::plan(
  double yaw_error, double measured_omega, double previous_omega, double dt) const
{
  if (!std::isfinite(yaw_error) || !std::isfinite(measured_omega) ||
    !std::isfinite(previous_omega) || !std::isfinite(dt) || dt < 0.0)
  {
    throw std::domain_error("rotation velocity inputs must be finite with nonnegative dt");
  }
  const double error = std::remainder(yaw_error, 2.0 * std::acos(-1.0));
  const double remaining = std::abs(error);
  const double direction = std::copysign(1.0, error);
  const double toward_speed = std::max({0.0, direction * measured_omega,
        direction * previous_omega});
  // Include one control interval so braking need not wait for the next sample.
  const double delay = config_.response_delay + dt;
  const double deceleration = config_.max_deceleration;
  RotationVelocityPlan result;
  result.predicted_stopping_angle = toward_speed * delay +
    toward_speed * toward_speed / (2.0 * deceleration);
  if (remaining <= config_.yaw_tolerance) {
    return result;  // Limiter ramps to zero; completion still checks raw motion.
  }

  // Solve w*T + w^2/(2*a) <= remaining for a steady commanded turn. The
  // rationalized root avoids cancellation when the remaining angle is small.
  const double a_delay = deceleration * delay;
  const double delay_cap = 2.0 * deceleration * remaining /
    (std::sqrt(a_delay * a_delay + 2.0 * deceleration * remaining) + a_delay);
  // Feedback may still be faster than the latest output while braking. Reserve
  // the travel during response delay using the larger speed toward the target.
  const double response_cap = std::sqrt(2.0 * deceleration *
      std::max(0.0, remaining - toward_speed * delay));
  result.stopping_speed_limit = std::min(delay_cap, response_cap);
  result.omega = direction * std::min({config_.kp_yaw * remaining,
        config_.max_omega, result.stopping_speed_limit});
  return result;
}

}  // namespace pid_tracking_controller
