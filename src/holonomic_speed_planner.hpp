#ifndef PID_TRACKING_CONTROLLER__HOLONOMIC_SPEED_PLANNER_HPP_
#define PID_TRACKING_CONTROLLER__HOLONOMIC_SPEED_PLANNER_HPP_

#include <limits>
#include <optional>

#include "holonomic_command_limiter.hpp"

#include "holonomic_tracking_reference.hpp"
#include "prepared_path.hpp"

namespace pid_tracking_controller
{

enum class HolonomicSpeedLimitReason
{
  kNominal,
  kExternal,
  kBodyAxis,
  kCombinedSpeed,
  kYawRate,
  kTangentRate,
  kStoppingDistance,
  kFeedbackBudget,
  kAccelerationPreview,
  kAccelerationBudget,
  kRotation,
  kTerminal
};

struct HolonomicSpeedPlannerConfig
{
  double nominal_tracking_speed{0.8};
  double max_vx{0.8};
  double max_vy{0.8};
  double max_xy_speed{0.8};
  double max_omega{0.8};
  double max_ax{0.5};
  double max_ay{0.5};
  double a_ref_up{0.5};
  double a_ref_down{0.5};
  double stopping_deceleration{0.12};
  // Total observation, control-cycle and drive response allowance, in seconds.
  double stopping_delay{0.5};
  // Combined approach speed; leave margin above the drive's per-axis deadband.
  double stopping_min_speed{0.04};
  double yaw_feedback_reserve{0.1};
  double configured_min_rate_window{0.3};
};

struct HolonomicSpeedPlan
{
  double reference_velocity_target{0.0};
  double reference_velocity{0.0};
  double axis_speed_limit{std::numeric_limits<double>::infinity()};
  double yaw_speed_limit{std::numeric_limits<double>::infinity()};
  double yaw_ff_speed_limit{std::numeric_limits<double>::infinity()};
  double tangent_speed_limit{std::numeric_limits<double>::infinity()};
  double stopping_speed_limit{std::numeric_limits<double>::infinity()};
  double stopping_distance{0.0};
  double stopping_delay_distance{0.0};
  double external_speed_limit{std::numeric_limits<double>::infinity()};
  double feedback_speed_limit{std::numeric_limits<double>::infinity()};
  // No feasible FF speed, or its minimum cannot yet be reached under acceleration limits.
  bool feedback_unreachable{false};
  double acceleration_speed_limit{std::numeric_limits<double>::infinity()};
  bool acceleration_unreachable{false};
  double max_abs_target_yaw_rate_per_meter{0.0};
  double max_abs_path_tangent_rate_per_meter{0.0};
  double representative_path_spacing{0.0};
  double rate_window_length{0.0};
  HolonomicSpeedLimitReason speed_limit_reason{HolonomicSpeedLimitReason::kNominal};
};

struct HolonomicTranslationFeedback
{
  double vx{0.0};
  double vy{0.0};
};

double convertExternalSpeedLimitToAbsolute(
  double speed_limit,
  bool percentage,
  double max_xy_speed);

class HolonomicSpeedPlanner
{
public:
  explicit HolonomicSpeedPlanner(const HolonomicSpeedPlannerConfig & config);

  void setPath(PreparedPathPtr path);
  void seed(double reference_velocity);

  // observed_speed is a nonnegative planar speed, conservatively including both
  // measured motion and the last output; stopping_delay already includes pose age.
  HolonomicSpeedPlan plan(
    const HolonomicTrackingReference & reference,
    double robot_yaw,
    double dt,
    double external_speed_limit = std::numeric_limits<double>::infinity(),
    double observed_speed = 0.0,
    HolonomicTranslationFeedback feedback = {},
    std::optional<HolonomicBodyCommand> previous_command = std::nullopt);

private:
  HolonomicSpeedPlannerConfig config_;
  PreparedPathPtr path_;
  double previous_reference_velocity_{0.0};
  double representative_path_spacing_{0.0};
  double rate_window_length_{0.0};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__HOLONOMIC_SPEED_PLANNER_HPP_
