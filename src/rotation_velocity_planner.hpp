#ifndef PID_TRACKING_CONTROLLER__ROTATION_VELOCITY_PLANNER_HPP_
#define PID_TRACKING_CONTROLLER__ROTATION_VELOCITY_PLANNER_HPP_

namespace pid_tracking_controller
{

struct RotationVelocityPlannerConfig
{
  double kp_yaw{2.5};
  double response_delay{0.20};
  double max_omega{0.30};
  double max_deceleration{0.50};
  double yaw_tolerance{0.05};
};

struct RotationVelocityPlan
{
  double omega{0.0};
  double stopping_speed_limit{0.0};
  double predicted_stopping_angle{0.0};
};

// Stateless target generation for admitted in-place rotations only. Admission,
// command slew, physical stop confirmation and completion belong to the caller.
class RotationVelocityPlanner
{
public:
  explicit RotationVelocityPlanner(const RotationVelocityPlannerConfig & config);

  // Use raw measured omega and the last accepted output (after safety limiting).
  // dt is the current valid control interval; the caller handles gaps/reseeding.
  RotationVelocityPlan plan(
    double yaw_error, double measured_omega, double previous_omega, double dt) const;

private:
  RotationVelocityPlannerConfig config_;
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__ROTATION_VELOCITY_PLANNER_HPP_
