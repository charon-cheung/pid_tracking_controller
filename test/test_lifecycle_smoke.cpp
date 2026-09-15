#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <limits>
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
#include "controller_test_access.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "rcl/time.h"

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




double loggedDouble(const std::string & text, const std::string & field)
{
  const std::size_t position = text.find(field);
  if (position == std::string::npos) {
    throw std::runtime_error("required numeric log field is absent: " + field);
  }
  return std::stod(text.substr(position + field.size()));
}

TEST(LifecycleSmoke, publishesCarrotAndCompactActiveDiagnosticsWithoutDiagnosticTopic)
{
  rclcpp::init(0, nullptr);

  bool carrot_received = false;
  geometry_msgs::msg::PointStamped received_carrot;
  geometry_msgs::msg::TwistStamped active_command;
  ScopedLogCapture log_capture;
  auto control_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

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
      auto clock = control_clock;
      ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
      ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL), RCL_RET_OK);
      PidTrackingControllerTestAccess::setClock(controller, clock);
      controller.computeVelocityCommands(pose, velocity, nullptr);
      ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1100000000LL), RCL_RET_OK);
      active_command = controller.computeVelocityCommands(pose, velocity, nullptr);
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
    EXPECT_NEAR(active_command.twist.linear.x, 0.02, 1e-12);
    EXPECT_DOUBLE_EQ(active_command.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(active_command.twist.angular.z, 0.0);
    if (costmap_configured) {
      ASSERT_EQ(rcl_set_ros_time_override(control_clock->get_clock_handle(), 1200000000LL), RCL_RET_OK);
      controller.setPlan(makeAngularLimitedLifecyclePath());
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.0;
      controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    }
    EXPECT_EQ(
      observer->count_publishers("/pid_tracking_controller/tracking_diagnostics"), 0u);
    EXPECT_NE(log_capture.output().find("PID_LOG holonomic_parameters"), std::string::npos);
    EXPECT_NE(log_capture.output().find("nominal=0.200"), std::string::npos);
    EXPECT_NE(log_capture.output().find("PID_LOG holonomic plan_revision=1"), std::string::npos);
    EXPECT_NE(log_capture.output().find("mode=translation"), std::string::npos);
    EXPECT_NE(log_capture.output().find("planner_step=true"), std::string::npos);
    EXPECT_NE(log_capture.output().find("limited_vy="), std::string::npos);
    EXPECT_EQ(log_capture.output().find("phase=TRACKING"), std::string::npos);
    EXPECT_EQ(log_capture.output().find("start_heading_alignment=entered"), std::string::npos);

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

TEST(LifecycleSmoke, configuredLimitsAndGainsReachActiveOutput)
{
  rclcpp::init(0, nullptr);
  ScopedLogCapture logs;
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("Controller.nominal_tracking_speed", 0.1);
    options.append_parameter_override("Controller.max_xy_speed", 0.4);
    options.append_parameter_override("Controller.max_vx", 0.4);
    options.append_parameter_override("Controller.max_vy", 0.4);
    options.append_parameter_override("Controller.kp_y", 0.5);
    // Legacy activation constraints must no longer gate the active holonomic chain.
    options.append_parameter_override("Controller.cruise_speed", 0.8);
    options.append_parameter_override("Controller.max_v", 0.1);
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("active_parent", options);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("active_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().label(), "inactive");
    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    PidTrackingController controller;
    controller.configure(parent, "Controller", tf, costmap);
    controller.activate();
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL), RCL_RET_OK);
    PidTrackingControllerTestAccess::setClock(controller, clock);
    controller.setPlan(makeLifecyclePath());
    controller.setSpeedLimit(50.0, true);
    geometry_msgs::msg::Twist measured;
    measured.linear.y = 0.3;
    const auto seed = controller.computeVelocityCommands(makePose(0.0, 0.0), measured, nullptr);
    EXPECT_NEAR(seed.twist.linear.y, 0.2, 1e-12);  // max_xy 0.4, not nominal 0.1.
    controller.deactivate();
    controller.activate();
    controller.setSpeedLimit(0.0, false);
    auto pose = makePose(0.0, 0.0);
    pose.pose.position.y = 0.1;
    controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1100000000LL), RCL_RET_OK);
    controller.setPlan(makeLifecyclePath());
    const auto begin = logs.output().size();
    const auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    EXPECT_LT(command.twist.linear.y, 0.0);
    EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
    EXPECT_NEAR(loggedDouble(logs.output().substr(begin), "raw_vy="), -0.05, 1e-12);
    controller.deactivate();
    controller.cleanup();
    controller.configure(parent, "Controller", tf, costmap);
    EXPECT_THROW(controller.computeVelocityCommands(pose, measured, nullptr), nav2_core::InvalidPath);
    controller.cleanup();
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
      "Controller.kp_y", std::numeric_limits<double>::max())).successful);
    controller.configure(parent, "Controller", tf, costmap);
    controller.setPlan(makeLifecyclePath());
    pose.pose.position.y = 2.0;  // Finite input whose P calculation overflows.
    EXPECT_THROW(controller.computeVelocityCommands(pose, measured, nullptr), nav2_core::NoValidControl);
    pose.pose.position.y = 0.0;
    const auto recovered = controller.computeVelocityCommands(
      pose, geometry_msgs::msg::Twist(), nullptr);
    EXPECT_DOUBLE_EQ(recovered.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(recovered.twist.linear.y, 0.0);
    controller.cleanup();
    costmap->cleanup();
  }
  rclcpp::shutdown();
}

TEST(LifecycleSmoke, invalidHolonomicParameterFailsBeforeRuntimeCanBeUsed)
{
  rclcpp::init(0, nullptr);

  {
    rclcpp::NodeOptions parent_options;
    parent_options.append_parameter_override("PidTrackingController.kp_yaw", -0.01);
    parent_options.append_parameter_override("PidTrackingController.max_alpha", 0.0);
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "pid_tracking_controller_invalid_holonomic_test_node", parent_options);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "pid_tracking_controller_invalid_holonomic_test_costmap");
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
    EXPECT_THROW(
      controller.configure(parent, "PidTrackingController", tf, costmap), std::invalid_argument);
    EXPECT_EQ(parent->count_publishers("/pid_tracking_controller/carrot"), 0u);
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
      "PidTrackingController.max_alpha", 0.5)).successful);
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
