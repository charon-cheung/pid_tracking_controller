#ifndef PID_TRACKING_CONTROLLER__ROTATION_COMPLETION_TRACKER_HPP_
#define PID_TRACKING_CONTROLLER__ROTATION_COMPLETION_TRACKER_HPP_

namespace pid_tracking_controller
{

struct RotationCompletionTrackerConfig
{
  double rotation_yaw_tolerance{0.0};
  double rotation_omega_tolerance{0.0};
  double rotation_xy_speed_tolerance{0.0};
  double rotation_settle_time{0.0};
};

struct RotationCompletionStatus
{
  bool yaw_within_tolerance{false};
  bool omega_within_tolerance{false};
  bool xy_speed_within_tolerance{false};
  bool all_conditions{false};
  double settled_duration{0.0};
  bool acknowledge_candidate{false};
};

class RotationCompletionTracker
{
public:
  explicit RotationCompletionTracker(const RotationCompletionTrackerConfig & config);

  RotationCompletionStatus update(
    double normalized_yaw_error,
    double measured_vx,
    double measured_vy,
    double measured_omega,
    double dt);

  void reset();

private:
  RotationCompletionTrackerConfig config_;
  double settled_duration_{0.0};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__ROTATION_COMPLETION_TRACKER_HPP_
