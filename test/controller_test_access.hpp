#ifndef PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_
#define PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_

#include "pid_tracking_controller/pid_tracking_controller.hpp"
#include "rclcpp/clock.hpp"
#include "holonomic_command_limiter.hpp"
#include "holonomic_speed_planner.hpp"
#include "rotation_completion_tracker.hpp"
#include "rotation_entry_gate.hpp"

namespace pid_tracking_controller
{

class PidTrackingControllerTestAccess
{
public:
  static void initializeHolonomic(
    PidTrackingController & controller, const HolonomicSpeedPlannerConfig & speed,
    const HolonomicCommandLimiterConfig & limits,
    const RotationCompletionTrackerConfig & rotation)
  {
    controller.holonomic_speed_planner_ = std::make_unique<HolonomicSpeedPlanner>(speed);
    controller.holonomic_command_limiter_ = std::make_unique<HolonomicCommandLimiter>(limits);
    controller.rotation_completion_tracker_ = std::make_unique<RotationCompletionTracker>(rotation);
    controller.rotation_entry_gate_ = std::make_unique<RotationEntryGate>(RotationEntryGateConfig{
      rotation.rotation_xy_speed_tolerance, rotation.rotation_omega_tolerance, 0.2, 8.0});
    controller.max_xy_speed_ = speed.max_xy_speed;
    controller.control_gap_threshold_ = limits.control_gap_threshold;
    controller.control_time_valid_ = false;
  }

  static void setLookaheadParameters(
    PidTrackingController & controller, double time, double minimum, double maximum)
  {
    controller.lookahead_time_ = time;
    controller.min_lookahead_dist_ = minimum;
    controller.max_lookahead_dist_ = maximum;
  }

  static void setCostmap(
    PidTrackingController & controller, nav2_costmap_2d::Costmap2D * costmap)
  {
    controller.costmap_ = costmap;
  }

  static void setClock(
    PidTrackingController & controller, const rclcpp::Clock::SharedPtr & clock)
  {
    controller.clock_ = clock;
  }

  static void setTf(
    PidTrackingController & controller, const std::shared_ptr<tf2_ros::Buffer> & tf)
  {
    controller.tf_ = tf;
  }

  static bool hasPreparedPath(const PidTrackingController & controller)
  {
    return static_cast<bool>(controller.prepared_path_);
  }
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_
