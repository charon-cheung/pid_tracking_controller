#ifndef PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_
#define PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
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
class HolonomicSpeedPlanner;
class HolonomicCommandLimiter;
struct PreparedPath;
struct PathProgressReference;
struct LegacyCarrotVelocityPlan;
struct HolonomicTrackingReference;
struct HolonomicRawCommand;
struct RotationCompletionStatus;

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

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  bool isGoalReached();
  void updateParam(bool, double) {}
  void setFactor(double factor);
  double getYawAngle(geometry_msgs::msg::PoseStamped & pose);
  bool shouldRotateToGoal(
    const geometry_msgs::msg::PoseStamped & current,
    const geometry_msgs::msg::PoseStamped & goal);
  bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);

  double linearRegularization(
    const geometry_msgs::msg::Twist & measured_twist, double desired_velocity);
  double angularRegularization(
    const geometry_msgs::msg::Twist & measured_twist, double desired_velocity);
  double getLookAheadDistance(double velocity);

  Eigen::Vector2d NoModelPidControl(
    Eigen::Vector3d state,
    Eigen::Vector3d desired_state,
    Eigen::Vector2d reference_input);
  Eigen::Vector2d UseModelPidControl(
    Eigen::Vector3d state,
    Eigen::Vector3d desired_state,
    Eigen::Vector2d reference_input);

private:
  friend class PidTrackingControllerTestAccess;

  struct TrackingStatistics
  {
    std::size_t sample_count{0};
    std::size_t linear_saturation_count{0};
    std::size_t angular_saturation_count{0};
    std::size_t tracking_sample_count{0};
    std::size_t tracking_linear_saturation_count{0};
    std::size_t tracking_angular_saturation_count{0};
    std::size_t tracking_raw_angular_saturation_count{0};
    std::size_t tracking_planner_limit_active_count{0};
    std::size_t alignment_sample_count{0};
    std::size_t alignment_angular_saturation_count{0};
    double tracking_sum_squared_lateral_error{0.0};
    double tracking_maximum_absolute_lateral_error{0.0};
    double tracking_sum_squared_linear_speed_error{0.0};
    std::vector<double> tracking_absolute_lateral_error_samples;
    double sum_squared_lateral_error{0.0};
    double maximum_absolute_lateral_error{0.0};
    double traveled_distance{0.0};
    double sum_actual_planar_speed{0.0};
    std::vector<double> absolute_lateral_error_samples;
    geometry_msgs::msg::Point previous_position;
    bool previous_position_valid{false};
  };

  void resetTrackingStatistics();
  void updateAndLogTrackingDiagnostics(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const geometry_msgs::msg::TwistStamped & command,
    const Eigen::Vector2d & raw_command,
    const LegacyCarrotVelocityPlan & velocity_plan,
    bool alignment_phase,
    double lookahead_distance,
    const PathProgressReference & path_reference,
    const HolonomicTrackingReference * holonomic_reference,
    const HolonomicRawCommand * shadow_rotation_command,
    const RotationCompletionStatus * shadow_rotation_status);
  void resetRotationState();
  geometry_msgs::msg::TwistStamped computeHolonomicCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const PathProgressReference & path_reference,
    double lookahead_distance,
    const rclcpp::Time & now);

  double factor_{0.0};

  double max_v_{0.7};
  double min_v_{0.05};
  double max_v_inc_{0.5};
  double max_w_{0.2};
  double min_w_{0.0};
  double max_w_inc_{0.2};

  double goal_dist_tol_{0.05};
  double rotate_tol_{0.5};

  std::string base_frame_{"base_link"};
  std::string map_frame_{"map"};
  double lookahead_time_{0.5};
  double min_lookahead_dist_{0.3};
  double max_lookahead_dist_{0.9};

  bool goal_reached_{false};

  double k_v_p_{2.0};
  double k_v_i_{0.01};
  double k_v_d_{0.10};
  double k_w_p_{1.0};
  double k_w_i_{0.01};
  double k_w_d_{0.10};
  double k_theta_{0.5};
  double model_k_v_{1.0};
  double model_k_w_{1.0};
  double l_{0.2};
  double cruise_speed_{0.51};
  double angular_speed_reserve_{0.10};

  bool enable_start_heading_alignment_{true};
  double start_heading_enter_tolerance_{0.6};
  double start_heading_exit_tolerance_{0.15};
  double start_heading_kp_{1.0};
  bool start_heading_check_pending_{false};
  bool start_heading_alignment_active_{false};
  double start_heading_target_{0.0};

  double kp_yaw_{1.0};
  double kp_x_{1.0};
  double kp_y_{1.0};
  double max_xy_speed_{0.20};
  double control_gap_threshold_{0.50};
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
  bool rotation_pending_{false};
  std::size_t pending_rotation_segment_index_{0};
  bool rotation_ack_candidate_{false};

  double e_v_{0.0};
  double e_w_{0.0};
  double i_v_{0.0};
  double i_w_{0.0};

  double d_t_{1.0 / 15.0};
  double goal_x_{std::numeric_limits<double>::max()};
  double goal_y_{std::numeric_limits<double>::max()};
  double goal_theta_{0.0};

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  std::shared_ptr<const PreparedPath> prepared_path_;
  std::unique_ptr<PathProgressTracker> path_progress_tracker_;
  std::string controller_plugin_name;
  bool use_model_{true};

  TrackingStatistics tracking_statistics_;
  std::size_t plan_revision_{0};
  double tracking_start_time_seconds_{0.0};
  double last_diagnostics_log_time_seconds_{0.0};
  bool tracking_start_time_valid_{false};
  bool force_diagnostics_log_{true};

  rclcpp::Logger logger_{rclcpp::get_logger("PidTrackingController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    carrot_pub_;
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_
