#ifndef PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_
#define PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <limits>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace pid_tracking_controller
{

class PidTrackingControllerTestAccess;
class PathProgressTracker;
class RotationCompletionTracker;
class RotationEntryGate;
class HolonomicSpeedPlanner;
class HolonomicCommandLimiter;
struct PreparedPath;
struct PathProgressReference;

class PidTrackingController : public nav2_core::Controller
{
public:
  PidTrackingController();
  ~PidTrackingController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void reset() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  double getLookAheadDistance(double velocity);

private:
  friend class PidTrackingControllerTestAccess;

  void resetRotationState(bool preserve_braking = false);
  geometry_msgs::msg::TwistStamped computeHolonomicCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const PathProgressReference & path_reference,
    double lookahead_distance,
    const rclcpp::Time & now);

  std::string base_frame_{"base_link"};
  double lookahead_time_{0.5};
  double min_lookahead_dist_{0.3};
  double max_lookahead_dist_{0.9};

  double kp_yaw_{1.0};
  double kp_x_{1.0};
  double kp_y_{1.0};
  double max_xy_speed_{0.20};
  double control_gap_threshold_{0.50};
  double last_commanded_planar_speed_{0.0};
  std::atomic<double> external_speed_limit_{std::numeric_limits<double>::infinity()};
  std::unique_ptr<HolonomicSpeedPlanner> holonomic_speed_planner_;
  std::unique_ptr<HolonomicCommandLimiter> holonomic_command_limiter_;
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool control_time_valid_{false};
  bool rotation_last_all_conditions_{false};
  int last_holonomic_mode_{-1};
  double rotation_yaw_tolerance_{0.05};
  double rotation_omega_tolerance_{0.05};
  double rotation_xy_speed_tolerance_{0.05};
  double rotation_settle_time_{0.30};
  std::unique_ptr<RotationCompletionTracker> rotation_completion_tracker_;
  std::unique_ptr<RotationEntryGate> rotation_entry_gate_;
  bool rotation_pending_{false};
  bool last_rotation_braking_{false};
  std::size_t pending_rotation_segment_index_{0};
  bool rotation_ack_candidate_{false};

  std::shared_ptr<const PreparedPath> prepared_path_;
  std::unique_ptr<PathProgressTracker> path_progress_tracker_;
  bool progress_handoff_pending_{false};
  bool progress_handoff_failure_logged_{false};
  std::string controller_plugin_name;

  std::size_t plan_revision_{0};
  double last_diagnostics_log_time_seconds_{0.0};
  bool force_diagnostics_log_{true};

  rclcpp::Logger logger_{rclcpp::get_logger("PidTrackingController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    raw_cmd_pub_;
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_
