#ifndef PID_TRACKING_CONTROLLER__LEGACY_CARROT_VELOCITY_PLANNER_HPP_
#define PID_TRACKING_CONTROLLER__LEGACY_CARROT_VELOCITY_PLANNER_HPP_

namespace pid_tracking_controller
{

enum class SpeedLimitReason
{
  kNotApplicable,
  kCruise,
  kAngular,
  kZeroCarrotDistance
};

struct LegacyCarrotVelocityPlannerConfig
{
  double model_k_v{1.0};
  double model_k_w{1.0};
  double characteristic_length{0.2};
  double cruise_speed{0.51};
  double max_angular_speed{0.6};
  double angular_speed_reserve{0.1};
};

struct LegacyCarrotVelocityPlan
{
  double carrot_distance{0.0};
  double alpha{0.0};
  double sin_alpha{0.0};
  double cos_alpha{0.0};
  double steering_gain_ratio{0.0};
  double cruise_speed{0.0};
  double angular_planning_limit{0.0};
  double angular_limited_speed{0.0};
  double reference_speed{0.0};
  double required_angular_at_cruise{0.0};
  double raw_linear_command{0.0};
  double raw_angular_command{0.0};
  SpeedLimitReason speed_limit_reason{SpeedLimitReason::kNotApplicable};
};

void validateLegacyCarrotVelocityPlannerConfig(
  const LegacyCarrotVelocityPlannerConfig & config);

void validateLegacyCarrotVelocityPlannerActivation(
  bool enabled,
  double max_linear_speed,
  const LegacyCarrotVelocityPlannerConfig & config);

LegacyCarrotVelocityPlan planLegacyCarrotVelocity(
  double ex_body,
  double ey_body,
  const LegacyCarrotVelocityPlannerConfig & config);

const char * speedLimitReasonName(SpeedLimitReason reason);

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__LEGACY_CARROT_VELOCITY_PLANNER_HPP_
