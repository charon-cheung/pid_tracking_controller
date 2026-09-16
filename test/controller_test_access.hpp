#ifndef PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_
#define PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_

#include <vector>

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

  static void setModelParameters(
    PidTrackingController & controller, double k_v, double k_w, double l)
  {
    controller.model_k_v_ = k_v;
    controller.model_k_w_ = k_w;
    controller.l_ = l;
  }

  static void setModelSpeedParameters(
    PidTrackingController & controller, double cruise_speed,
    double angular_speed_reserve, double max_angular_speed)
  {
    controller.cruise_speed_ = cruise_speed;
    controller.angular_speed_reserve_ = angular_speed_reserve;
    controller.max_w_ = max_angular_speed;
  }

  static void setLookaheadParameters(
    PidTrackingController & controller, double time, double minimum, double maximum)
  {
    controller.lookahead_time_ = time;
    controller.min_lookahead_dist_ = minimum;
    controller.max_lookahead_dist_ = maximum;
  }

  static void setNoModelParameters(PidTrackingController & controller, double dt)
  {
    controller.d_t_ = dt;
    controller.k_v_p_ = 2.0;
    controller.k_v_i_ = 0.01;
    controller.k_v_d_ = 0.10;
    controller.k_w_p_ = 1.0;
    controller.k_w_i_ = 0.01;
    controller.k_w_d_ = 0.10;
    controller.max_v_ = 0.7;
    controller.min_v_ = 0.05;
    controller.max_v_inc_ = 0.5;
    controller.max_w_ = 0.2;
    controller.min_w_ = 0.0;
    controller.max_w_inc_ = 0.2;
    controller.e_v_ = 0.0;
    controller.e_w_ = 0.0;
    controller.i_v_ = 0.0;
    controller.i_w_ = 0.0;
  }

  static void replaceGlobalPlan(
    PidTrackingController & controller,
    const std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    controller.global_plan_ = plan;
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

  static void setMaximumVelocityIncrements(
    PidTrackingController & controller, double linear, double angular)
  {
    controller.max_v_inc_ = linear;
    controller.max_w_inc_ = angular;
  }

  static const std::vector<geometry_msgs::msg::PoseStamped> & globalPlan(
    const PidTrackingController & controller)
  {
    return controller.global_plan_;
  }

  static const std::vector<geometry_msgs::msg::PoseStamped> & plan(
    const PidTrackingController & controller)
  {
    return controller.global_plan_;
  }

  static std::size_t trackingSampleCount(const PidTrackingController & controller)
  {
    return controller.tracking_statistics_.tracking_sample_count;
  }

  static std::size_t alignmentSampleCount(const PidTrackingController & controller)
  {
    return controller.tracking_statistics_.alignment_sample_count;
  }

  static double trackingSumSquaredLateralError(
    const PidTrackingController & controller)
  {
    return controller.tracking_statistics_.tracking_sum_squared_lateral_error;
  }

  static double trackingMaximumAbsoluteLateralError(
    const PidTrackingController & controller)
  {
    return controller.tracking_statistics_.tracking_maximum_absolute_lateral_error;
  }

  static const std::vector<double> & trackingAbsoluteLateralErrorSamples(
    const PidTrackingController & controller)
  {
    return controller.tracking_statistics_.tracking_absolute_lateral_error_samples;
  }
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_
