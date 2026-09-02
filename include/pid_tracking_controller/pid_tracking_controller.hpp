#ifndef PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_
#define PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_

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

class PidTrackingController : public nav2_core::Controller
{
public:
  PidTrackingController() = default;
  ~PidTrackingController() override = default;

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
  std::vector<geometry_msgs::msg::PoseStamped> prune(
    const geometry_msgs::msg::PoseStamped robot_pose_global);
  double getLookAheadDistance(double velocity);
  void getLookAheadPoint(
    double lookahead_dist,
    geometry_msgs::msg::PoseStamped robot_pose_global,
    const std::vector<geometry_msgs::msg::PoseStamped> & prune_plan,
    geometry_msgs::msg::PointStamped & point,
    double & theta,
    double & kappa);

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
    const std::vector<geometry_msgs::msg::PoseStamped> & prune_plan,
    double lookahead_distance,
    double theta_trj,
    double kappa,
    const geometry_msgs::msg::PointStamped & carrot);

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
  double k_{1.0};
  double l_{0.2};

  bool enable_start_heading_alignment_{true};
  double start_heading_enter_tolerance_{0.6};
  double start_heading_exit_tolerance_{0.15};
  double start_heading_kp_{1.0};
  bool start_heading_check_pending_{false};
  bool start_heading_alignment_active_{false};
  double start_heading_target_{0.0};

  double e_v_{0.0};
  double e_w_{0.0};
  double i_v_{0.0};
  double i_w_{0.0};

  double d_t_{1.0 / 15.0};
  double goal_x_{std::numeric_limits<double>::max()};
  double goal_y_{std::numeric_limits<double>::max()};
  double goal_theta_{0.0};

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
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
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    carrot_pub_;
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__PID_TRACKING_CONTROLLER_HPP_
