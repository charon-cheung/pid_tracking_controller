#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pid_tracking_controller/pid_tracking_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging.h"
#include "tf2_ros/buffer.h"

namespace pid_tracking_controller
{
namespace
{

using namespace std::chrono_literals;

std::string * captured_log_output = nullptr;

void captureLoggingOutput(
  const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
  const char * format, va_list * arguments)
{
  std::array<char, 4096> buffer{};
  va_list copied_arguments;
  va_copy(copied_arguments, *arguments);
  const int written = std::vsnprintf(
    buffer.data(), buffer.size(), format, copied_arguments);
  va_end(copied_arguments);
  if (captured_log_output != nullptr && written > 0) {
    captured_log_output->append(buffer.data());
    captured_log_output->push_back('\n');
  }
}

class ScopedLogCapture
{
public:
  ScopedLogCapture()
  : previous_handler_(rcutils_logging_get_output_handler())
  {
    captured_log_output = &output_;
    rcutils_logging_set_output_handler(captureLoggingOutput);
  }

  ~ScopedLogCapture()
  {
    rcutils_logging_set_output_handler(previous_handler_);
    captured_log_output = nullptr;
  }

  const std::string & output() const {return output_;}

private:
  std::string output_;
  rcutils_logging_output_handler_t previous_handler_;
};

nav_msgs::msg::Path makeLifecyclePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const double x : {0.0, 0.3, 0.6}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path makeAngularLimitedLifecyclePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const double y : {0.0, 0.3, 0.6}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped makePose(double x, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  pose.pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

nav_msgs::msg::Path makeRotationPendingLifecyclePath()
{
  constexpr double kDegreesToRadians = M_PI / 180.0;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {
    makePose(0.0, 179.0 * kDegreesToRadians),
    makePose(0.3, 179.0 * kDegreesToRadians),
    makePose(0.3, -179.0 * kDegreesToRadians),
    makePose(0.6, -179.0 * kDegreesToRadians)};
  return path;
}

nav_msgs::msg::Path makeInitialRotationLifecyclePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {makePose(0.0, 0.0), makePose(0.0, 0.2)};
  return path;
}

nav_msgs::msg::Path makeLegacyBaselineLifecyclePath()
{
  constexpr double kDegreesToRadians = M_PI / 180.0;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {
    makePose(0.0, 179.0 * kDegreesToRadians),
    makePose(0.3, 179.0 * kDegreesToRadians),
    makePose(0.6, -179.0 * kDegreesToRadians)};
  return path;
}

double loggedDouble(const std::string & text, const std::string & field)
{
  const std::size_t position = text.find(field);
  if (position == std::string::npos) {
    throw std::runtime_error("required numeric log field is absent: " + field);
  }
  return std::stod(text.substr(position + field.size()));
}

TEST(LifecycleSmoke, publishesCarrotAndLogsTrackingDiagnosticsWithoutDiagnosticTopic)
{
  rclcpp::init(0, nullptr);

  bool carrot_received = false;
  geometry_msgs::msg::PointStamped received_carrot;
  geometry_msgs::msg::TwistStamped legacy_command;
  ScopedLogCapture log_capture;

  {
    rclcpp::NodeOptions parent_options;
    parent_options.append_parameter_override("PidTrackingController.max_v", 0.8);
    parent_options.append_parameter_override("PidTrackingController.max_w", 0.8);
    parent_options.append_parameter_override("PidTrackingController.model_k_v", 1.9);
    parent_options.append_parameter_override("PidTrackingController.model_k_w", 2.1);
    parent_options.append_parameter_override("PidTrackingController.cruise_speed", 0.8);
    parent_options.append_parameter_override(
      "PidTrackingController.enable_start_heading_alignment", false);
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "pid_tracking_controller_test_node", parent_options);
    auto observer = std::make_shared<rclcpp::Node>(
      "pid_tracking_controller_test_observer");
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "pid_tracking_controller_test_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    const bool costmap_configured =
      costmap->get_current_state().label() == "inactive";
    EXPECT_TRUE(costmap_configured);

    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    PidTrackingController controller;
    if (costmap_configured) {
      controller.configure(parent, "PidTrackingController", tf, costmap);
      controller.activate();
    }

    auto carrot_subscription = observer->create_subscription<geometry_msgs::msg::PointStamped>(
      "/pid_tracking_controller/carrot", rclcpp::QoS(10).reliable(),
      [&](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        received_carrot = *message;
        carrot_received = true;
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(parent->get_node_base_interface());
    executor.add_node(observer);

    const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < discovery_deadline &&
      observer->count_publishers("/pid_tracking_controller/carrot") == 0u)
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    if (costmap_configured) {
      controller.setPlan(makeLifecyclePath());
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.0;
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = 0.2995;
      legacy_command = controller.computeVelocityCommands(pose, velocity, nullptr);
    }

    const auto message_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < message_deadline && !carrot_received)
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(carrot_received);
    EXPECT_EQ(received_carrot.header.frame_id, "map");
    EXPECT_NEAR(received_carrot.point.x, 0.3, 1e-12);
    EXPECT_DOUBLE_EQ(received_carrot.point.y, 0.0);
    EXPECT_DOUBLE_EQ(received_carrot.point.z, 0.0);
    EXPECT_NEAR(legacy_command.twist.linear.x, 0.7995, 1e-12);
    EXPECT_DOUBLE_EQ(legacy_command.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(legacy_command.twist.angular.z, 0.0);
    if (costmap_configured) {
      controller.setPlan(makeAngularLimitedLifecyclePath());
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.0;
      controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    }
    EXPECT_EQ(
      observer->count_publishers("/pid_tracking_controller/tracking_diagnostics"), 0u);
    EXPECT_NE(
      log_capture.output().find("PID_LOG effective_parameters"), std::string::npos);
    EXPECT_NE(log_capture.output().find("max_v=0.800000"), std::string::npos);
    EXPECT_NE(log_capture.output().find("max_w=0.800000"), std::string::npos);
    EXPECT_NE(log_capture.output().find("cruise_speed=0.800000"), std::string::npos);
    EXPECT_NE(log_capture.output().find("kp_yaw=1.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("rotation_yaw_tolerance=0.050000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("rotation_omega_tolerance=0.050000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("rotation_xy_speed_tolerance=0.050000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("rotation_settle_time=0.300000"),
      std::string::npos);
    EXPECT_NE(log_capture.output().find("PID_LOG plan_revision=1"), std::string::npos);
    EXPECT_NE(log_capture.output().find("angular_saturation_ratio="), std::string::npos);
    EXPECT_NE(log_capture.output().find("phase=TRACKING"), std::string::npos);
    EXPECT_NE(log_capture.output().find("tracking_sample_count=1"), std::string::npos);
    EXPECT_NE(log_capture.output().find("alignment_sample_count=0"), std::string::npos);
    EXPECT_NE(log_capture.output().find("raw_linear_command="), std::string::npos);
    EXPECT_NE(log_capture.output().find("raw_angular_command="), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("tracking_raw_angular_saturation_ratio="),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("planner_limit_active_ratio=1.000000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("linear_speed_tracking_error=0.500000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("tracking_linear_speed_error_rms=0.500000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("tracking_linear_saturation_ratio=1.000000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("tracking_lateral_error_rms="), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("tracking_lateral_error_p95_abs="), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("tracking_lateral_error_max_abs="), std::string::npos);
    EXPECT_NE(log_capture.output().find("carrot_alpha="), std::string::npos);
    EXPECT_NE(log_capture.output().find("carrot_alpha_deg="), std::string::npos);
    EXPECT_NE(log_capture.output().find("carrot_sin_alpha="), std::string::npos);
    EXPECT_NE(log_capture.output().find("carrot_cos_alpha="), std::string::npos);
    EXPECT_NE(log_capture.output().find("steering_gain_ratio="), std::string::npos);
    EXPECT_NE(log_capture.output().find("cruise_speed="), std::string::npos);
    EXPECT_NE(log_capture.output().find("angular_planning_limit="), std::string::npos);
    EXPECT_NE(log_capture.output().find("angular_limited_speed="), std::string::npos);
    EXPECT_NE(log_capture.output().find("reference_speed="), std::string::npos);
    EXPECT_NE(log_capture.output().find("speed_limit_reason=cruise"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("required_angular_at_cruise="), std::string::npos);
    EXPECT_NE(log_capture.output().find("kappa="), std::string::npos);
    EXPECT_NE(log_capture.output().find("progress_arc_length="), std::string::npos);
    EXPECT_NE(log_capture.output().find("carrot_arc_length="), std::string::npos);
    EXPECT_NE(log_capture.output().find("segment_index="), std::string::npos);
    EXPECT_NE(log_capture.output().find("segment_fraction="), std::string::npos);
    EXPECT_NE(log_capture.output().find("remaining_path_length="), std::string::npos);
    EXPECT_NE(log_capture.output().find("progress_budget="), std::string::npos);
    EXPECT_NE(log_capture.output().find("reference_mode=translation"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("current_segment_index=0"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("current_segment_fraction=0.000000"),
      std::string::npos);
    EXPECT_NE(log_capture.output().find("current_ref_x=0.000000"), std::string::npos);
    EXPECT_NE(log_capture.output().find("current_ref_y=0.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("current_path_tangent=0.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("current_target_yaw=0.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("current_target_yaw_rate_per_meter=0.000000"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("preview_segment_index=0"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("preview_segment_fraction=1.000000"),
      std::string::npos);
    EXPECT_NE(log_capture.output().find("preview_ref_x=0.300000"), std::string::npos);
    EXPECT_NE(log_capture.output().find("preview_ref_y=0.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("preview_path_tangent=0.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("preview_target_yaw=0.000000"), std::string::npos);
    EXPECT_NE(
      log_capture.output().find("preview_clamped_by_rotation=false"),
      std::string::npos);
    EXPECT_NE(
      log_capture.output().find("rotation_target_yaw=0.000000"), std::string::npos);

    if (costmap_configured) {
      controller.deactivate();
      controller.cleanup();
      costmap->cleanup();
    }
    executor.remove_node(observer);
    executor.remove_node(parent->get_node_base_interface());
  }

  rclcpp::shutdown();
}

TEST(LifecycleSmoke, rotationPendingShadowUsesMeasuredSettleWithoutChangingLegacyCommand)
{
  rclcpp::init(0, nullptr);
  ScopedLogCapture log_capture;

  {
    rclcpp::NodeOptions parent_options;
    parent_options.append_parameter_override(
      "PidTrackingController.enable_start_heading_alignment", false);
    parent_options.append_parameter_override("PidTrackingController.kp_yaw", 2.0);
    parent_options.append_parameter_override(
      "PidTrackingController.rotation_settle_time", 0.12);
    parent_options.append_parameter_override(
      "LegacyBaseline.enable_start_heading_alignment", false);
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "pid_tracking_controller_shadow_test_node", parent_options);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "pid_tracking_controller_shadow_test_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().label(), "inactive");

    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    PidTrackingController legacy_baseline;
    legacy_baseline.configure(parent, "LegacyBaseline", tf, costmap);
    legacy_baseline.activate();
    legacy_baseline.setPlan(makeLegacyBaselineLifecyclePath());

    constexpr double kRobotYaw = 179.0 * M_PI / 180.0;
    geometry_msgs::msg::PoseStamped pose = makePose(0.3, kRobotYaw);
    geometry_msgs::msg::Twist stopped_velocity;
    const auto baseline_command = legacy_baseline.computeVelocityCommands(
      pose, stopped_velocity, nullptr);

    const std::size_t first_observation_begin = log_capture.output().size();
    PidTrackingController controller;
    controller.configure(parent, "PidTrackingController", tf, costmap);
    controller.activate();
    controller.setPlan(makeRotationPendingLifecyclePath());
    const auto first_command = controller.computeVelocityCommands(
      pose, stopped_velocity, nullptr);
    const std::string first_observation =
      log_capture.output().substr(first_observation_begin);

    EXPECT_DOUBLE_EQ(first_command.twist.linear.x, baseline_command.twist.linear.x);
    EXPECT_DOUBLE_EQ(first_command.twist.linear.y, baseline_command.twist.linear.y);
    EXPECT_DOUBLE_EQ(first_command.twist.angular.z, baseline_command.twist.angular.z);
    EXPECT_DOUBLE_EQ(first_command.twist.linear.y, 0.0);
    EXPECT_NE(
      first_observation.find("rotation_settle_time=0.120000"),
      std::string::npos);
    EXPECT_NE(
      first_observation.find("pending_rotation_segment_index=1"),
      std::string::npos);
    EXPECT_DOUBLE_EQ(
      loggedDouble(first_observation, "shadow_settled_duration="), 0.0);
    EXPECT_NE(
      first_observation.find("rotation_ack_candidate=false"),
      std::string::npos);
    EXPECT_NE(
      first_observation.find("shadow_raw_vx=0.000000"), std::string::npos);
    EXPECT_DOUBLE_EQ(loggedDouble(first_observation, "shadow_raw_vy="), 0.0);
    EXPECT_NE(
      first_observation.find("shadow_yaw_error=0.034907"), std::string::npos);
    EXPECT_NE(
      first_observation.find("shadow_raw_omega=0.069813"), std::string::npos);

    const std::size_t settled_log_begin = log_capture.output().size();
    std::this_thread::sleep_for(140ms);
    const auto settled_command = controller.computeVelocityCommands(
      pose, stopped_velocity, nullptr);
    const std::string settled_log = log_capture.output().substr(settled_log_begin);

    EXPECT_DOUBLE_EQ(settled_command.twist.linear.x, baseline_command.twist.linear.x);
    EXPECT_DOUBLE_EQ(settled_command.twist.linear.y, baseline_command.twist.linear.y);
    EXPECT_DOUBLE_EQ(settled_command.twist.angular.z, baseline_command.twist.angular.z);
    EXPECT_DOUBLE_EQ(settled_command.twist.linear.y, 0.0);
    EXPECT_GE(loggedDouble(settled_log, "shadow_settled_duration="), 0.12);
    EXPECT_NE(
      settled_log.find("rotation_ack_candidate=true"), std::string::npos);
    EXPECT_NE(
      settled_log.find("pending_rotation_segment_index=1"), std::string::npos);

    geometry_msgs::msg::Twist moving_velocity;
    moving_velocity.linear.x = 0.06;
    moving_velocity.angular.z = 0.10;
    const std::size_t violation_log_begin = log_capture.output().size();
    controller.computeVelocityCommands(pose, moving_velocity, nullptr);
    const std::string violation_log =
      log_capture.output().substr(violation_log_begin);

    EXPECT_NE(
      violation_log.find("pending_rotation_segment_index=1"), std::string::npos);
    EXPECT_DOUBLE_EQ(
      loggedDouble(violation_log, "shadow_settled_duration="), 0.0);
    EXPECT_NE(
      violation_log.find("shadow_measured_vx=0.060000"), std::string::npos);
    EXPECT_NE(
      violation_log.find("shadow_measured_vy=0.000000"), std::string::npos);
    EXPECT_NE(
      violation_log.find("shadow_measured_omega=0.100000"), std::string::npos);
    EXPECT_NE(
      violation_log.find("rotation_ack_candidate=false"), std::string::npos);

    const std::size_t restored_log_begin = log_capture.output().size();
    std::this_thread::sleep_for(140ms);
    controller.computeVelocityCommands(pose, stopped_velocity, nullptr);
    const std::string restored_log = log_capture.output().substr(restored_log_begin);
    EXPECT_GE(loggedDouble(restored_log, "shadow_settled_duration="), 0.12);
    EXPECT_NE(
      restored_log.find("rotation_ack_candidate=true"), std::string::npos);

    const std::size_t equivalent_plan_log_begin = log_capture.output().size();
    controller.setPlan(makeRotationPendingLifecyclePath());
    controller.computeVelocityCommands(pose, stopped_velocity, nullptr);
    const std::string equivalent_plan_log =
      log_capture.output().substr(equivalent_plan_log_begin);

    EXPECT_NE(
      equivalent_plan_log.find("reference_mode=rotation_pending"),
      std::string::npos);
    EXPECT_NE(
      equivalent_plan_log.find("pending_rotation_segment_index=1"),
      std::string::npos);
    EXPECT_DOUBLE_EQ(
      loggedDouble(equivalent_plan_log, "shadow_settled_duration="), 0.0);
    EXPECT_NE(
      equivalent_plan_log.find("rotation_ack_candidate=false"),
      std::string::npos);

    const std::size_t equivalent_restored_log_begin = log_capture.output().size();
    std::this_thread::sleep_for(140ms);
    controller.computeVelocityCommands(
      pose, stopped_velocity, nullptr);
    const std::string equivalent_restored_log =
      log_capture.output().substr(equivalent_restored_log_begin);
    EXPECT_GE(
      loggedDouble(equivalent_restored_log, "shadow_settled_duration="), 0.12);
    EXPECT_NE(
      equivalent_restored_log.find("rotation_ack_candidate=true"),
      std::string::npos);

    const std::size_t index_change_log_begin = log_capture.output().size();
    controller.setPlan(makeInitialRotationLifecyclePath());
    controller.computeVelocityCommands(
      makePose(0.0, 0.2), geometry_msgs::msg::Twist(), nullptr);
    const std::string index_change_log =
      log_capture.output().substr(index_change_log_begin);

    EXPECT_NE(
      index_change_log.find("reference_mode=rotation_pending"),
      std::string::npos);
    EXPECT_NE(
      index_change_log.find("pending_rotation_segment_index=0"),
      std::string::npos);
    EXPECT_DOUBLE_EQ(
      loggedDouble(index_change_log, "shadow_settled_duration="), 0.0);
    EXPECT_NE(
      index_change_log.find("rotation_ack_candidate=false"), std::string::npos);

    const std::size_t index_restored_log_begin = log_capture.output().size();
    std::this_thread::sleep_for(140ms);
    controller.computeVelocityCommands(
      makePose(0.0, 0.2), geometry_msgs::msg::Twist(), nullptr);
    const std::string index_restored_log =
      log_capture.output().substr(index_restored_log_begin);
    EXPECT_GE(
      loggedDouble(index_restored_log, "shadow_settled_duration="), 0.12);
    EXPECT_NE(
      index_restored_log.find("rotation_ack_candidate=true"),
      std::string::npos);

    const std::size_t mode_change_log_begin = log_capture.output().size();
    controller.setPlan(makeLifecyclePath());
    controller.computeVelocityCommands(
      makePose(0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
    const std::string mode_change_log =
      log_capture.output().substr(mode_change_log_begin);

    EXPECT_NE(
      mode_change_log.find("reference_mode=translation"), std::string::npos);
    EXPECT_NE(
      mode_change_log.find("shadow_rotation_available=false"),
      std::string::npos);
    EXPECT_NE(
      mode_change_log.find("rotation_ack_candidate=false"),
      std::string::npos);

    controller.deactivate();
    controller.cleanup();
    legacy_baseline.deactivate();
    legacy_baseline.cleanup();
    costmap->cleanup();
  }

  rclcpp::shutdown();
}

TEST(LifecycleSmoke, invalidShadowParameterFailsBeforeRuntimeCanBeUsed)
{
  rclcpp::init(0, nullptr);

  {
    rclcpp::NodeOptions parent_options;
    parent_options.append_parameter_override("PidTrackingController.kp_yaw", -0.01);
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "pid_tracking_controller_invalid_shadow_test_node", parent_options);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "pid_tracking_controller_invalid_shadow_test_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().label(), "inactive");

    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    PidTrackingController controller;
    EXPECT_THROW(
      controller.configure(parent, "PidTrackingController", tf, costmap),
      std::invalid_argument);
    EXPECT_EQ(parent->count_publishers("/pid_tracking_controller/carrot"), 0u);

    ASSERT_TRUE(
      parent->set_parameter(rclcpp::Parameter("PidTrackingController.kp_yaw", 1.0)).
      successful);
    EXPECT_NO_THROW(
      controller.configure(parent, "PidTrackingController", tf, costmap));
    EXPECT_EQ(parent->count_publishers("/pid_tracking_controller/carrot"), 1u);

    controller.cleanup();
    costmap->cleanup();
  }

  rclcpp::shutdown();
}

}  // namespace
}  // namespace pid_tracking_controller
