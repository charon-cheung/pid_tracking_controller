#include "legacy_carrot_velocity_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "common/math/math_helper.h"

namespace pid_tracking_controller
{
namespace
{

bool planIsFinite(const LegacyCarrotVelocityPlan & plan)
{
  return std::isfinite(plan.carrot_distance) &&
         std::isfinite(plan.alpha) &&
         std::isfinite(plan.sin_alpha) &&
         std::isfinite(plan.cos_alpha) &&
         std::isfinite(plan.steering_gain_ratio) &&
         std::isfinite(plan.cruise_speed) &&
         std::isfinite(plan.angular_planning_limit) &&
         std::isfinite(plan.angular_limited_speed) &&
         std::isfinite(plan.reference_speed) &&
         std::isfinite(plan.required_angular_at_cruise) &&
         std::isfinite(plan.raw_linear_command) &&
         std::isfinite(plan.raw_angular_command);
}

}  // namespace

void validateLegacyCarrotVelocityPlannerConfig(
  const LegacyCarrotVelocityPlannerConfig & config)
{
  if (!std::isfinite(config.model_k_v) ||
    config.model_k_v <= 0.0)
  {
    throw std::invalid_argument("model_k_v must be finite and positive");
  }
  if (!std::isfinite(config.model_k_w) ||
    config.model_k_w <= 0.0)
  {
    throw std::invalid_argument("model_k_w must be finite and positive");
  }
  if (!std::isfinite(config.characteristic_length) ||
    config.characteristic_length <= 0.0)
  {
    throw std::invalid_argument("characteristic_length must be finite and positive");
  }
  if (!std::isfinite(config.cruise_speed) ||
    config.cruise_speed <= 0.0)
  {
    throw std::invalid_argument("cruise_speed must be finite and positive");
  }
  if (!std::isfinite(config.max_angular_speed) ||
    config.max_angular_speed <= 0.0)
  {
    throw std::invalid_argument("max_angular_speed must be finite and positive");
  }
  if (!std::isfinite(config.angular_speed_reserve) ||
    config.angular_speed_reserve < 0.0 ||
    config.angular_speed_reserve >= config.max_angular_speed)
  {
    throw std::invalid_argument(
            "angular_speed_reserve must be finite and smaller than max_angular_speed");
  }

  const double steering_gain_ratio = config.model_k_w / config.model_k_v;
  const double angular_gain = steering_gain_ratio / config.characteristic_length;
  if (!std::isfinite(steering_gain_ratio) || steering_gain_ratio <= 0.0 ||
    !std::isfinite(angular_gain) || angular_gain <= 0.0)
  {
    throw std::invalid_argument("model gain ratio and angular gain must be finite and positive");
  }
}

void validateLegacyCarrotVelocityPlannerActivation(
  bool enabled,
  double max_linear_speed,
  const LegacyCarrotVelocityPlannerConfig & config)
{
  if (!enabled) {
    return;
  }

  validateLegacyCarrotVelocityPlannerConfig(config);
  if (!std::isfinite(max_linear_speed) || max_linear_speed <= 0.0 ||
    config.cruise_speed > max_linear_speed)
  {
    throw std::invalid_argument(
            "max_linear_speed must be finite, positive, and not smaller than cruise_speed");
  }
}

LegacyCarrotVelocityPlan planLegacyCarrotVelocity(
  double ex_body,
  double ey_body,
  const LegacyCarrotVelocityPlannerConfig & config)
{
  validateLegacyCarrotVelocityPlannerConfig(config);
  if (!std::isfinite(ex_body) || !std::isfinite(ey_body)) {
    throw std::domain_error("body-frame carrot error must be finite");
  }

  LegacyCarrotVelocityPlan plan;
  plan.carrot_distance = std::hypot(ex_body, ey_body);
  plan.steering_gain_ratio = config.model_k_w / config.model_k_v;
  plan.cruise_speed = config.cruise_speed;
  plan.angular_planning_limit =
    config.max_angular_speed - config.angular_speed_reserve;
  plan.angular_limited_speed = config.cruise_speed;

  if (plan.carrot_distance <= rmp::common::math::kMathEpsilon) {
    plan.speed_limit_reason = SpeedLimitReason::kZeroCarrotDistance;
    if (!planIsFinite(plan)) {
      throw std::domain_error("velocity planner produced non-finite output");
    }
    return plan;
  }

  plan.alpha = std::atan2(ey_body, ex_body);
  plan.sin_alpha = ey_body / plan.carrot_distance;
  plan.cos_alpha = ex_body / plan.carrot_distance;
  plan.required_angular_at_cruise =
    config.cruise_speed * plan.steering_gain_ratio /
    config.characteristic_length * plan.sin_alpha;

  if (!std::isfinite(plan.required_angular_at_cruise)) {
    throw std::domain_error("velocity planner produced non-finite angular demand");
  }

  if (std::abs(plan.required_angular_at_cruise) > plan.angular_planning_limit) {
    plan.angular_limited_speed = config.cruise_speed *
      (plan.angular_planning_limit / std::abs(plan.required_angular_at_cruise));
    plan.reference_speed = plan.angular_limited_speed;
    plan.speed_limit_reason = SpeedLimitReason::kAngular;
  } else {
    plan.reference_speed = config.cruise_speed;
    plan.speed_limit_reason = SpeedLimitReason::kCruise;
  }
  plan.raw_linear_command = plan.reference_speed * plan.cos_alpha;
  plan.raw_angular_command = plan.required_angular_at_cruise *
    (plan.reference_speed / config.cruise_speed);

  if (!planIsFinite(plan)) {
    throw std::domain_error("velocity planner produced non-finite output");
  }
  return plan;
}

const char * speedLimitReasonName(SpeedLimitReason reason)
{
  switch (reason) {
    case SpeedLimitReason::kCruise:
      return "cruise";
    case SpeedLimitReason::kAngular:
      return "angular";
    case SpeedLimitReason::kZeroCarrotDistance:
      return "zero_carrot_distance";
    case SpeedLimitReason::kNotApplicable:
    default:
      return "not_applicable";
  }
}

}  // namespace pid_tracking_controller
