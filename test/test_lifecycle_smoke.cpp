#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
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

}  // namespace
}  // namespace pid_tracking_controller
