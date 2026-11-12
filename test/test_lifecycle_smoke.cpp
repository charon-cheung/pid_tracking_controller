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
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "direct_costmap_fixture.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pid_tracking_controller/pid_tracking_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging.h"
#include "tf2_ros/buffer.h"
#include "controller_test_access.hpp"
#include "motion_observation.hpp"
#include "pid_tracking_controller/msg/tracking_progress.hpp"
#include "pid_tracking_controller/msg/path_risk_state.hpp"
#include "pid_tracking_controller/path_identity.hpp"
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




TEST(LifecycleSmoke, publishesCarrotRawCommandAndProgressWithoutPidLogs)
{
  rclcpp::init(0, nullptr);

  bool carrot_received = false;
  msg::TrackingProgress received_progress;
  geometry_msgs::msg::PointStamped received_carrot;
  geometry_msgs::msg::TwistStamped active_command;
  geometry_msgs::msg::TwistStamped received_raw;
  ScopedLogCapture log_capture;
  auto control_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  {
    rclcpp::NodeOptions parent_options;
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "pid_tracking_controller_test_node", parent_options);
    parent->declare_parameter("controller_frequency", 15.0);
    parent->declare_parameter("PidTrackingController.publish_tracking_progress", true);
    parent->declare_parameter("failure_tolerance", 0.0);
    parent->declare_parameter("publish_zero_velocity", true);
    parent->declare_parameter("PidTrackingController.plugin",
      "pid_tracking_controller::PidTrackingController");
    parent->declare_parameter("PidTrackingController.fixed_heading_yaw_enter_error",
      0.03490658503988659);
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

    auto progress_subscription = observer->create_subscription<msg::TrackingProgress>(
      "/pid_tracking_controller/tracking_progress", rclcpp::QoS(10).reliable(),
      [&](msg::TrackingProgress::ConstSharedPtr message) {received_progress = *message;});
    auto carrot_subscription = observer->create_subscription<geometry_msgs::msg::PointStamped>(
      "/pid_tracking_controller/carrot", rclcpp::QoS(10).reliable(),
      [&](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        received_carrot = *message;
        carrot_received = true;
      });
    auto raw_subscription = observer->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/tracking_cmd_vel_raw", rclcpp::QoS(10).reliable(),
      [&](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        received_raw = *message;
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(parent->get_node_base_interface());
    executor.add_node(observer);

    const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < discovery_deadline &&
      (observer->count_publishers("/pid_tracking_controller/carrot") == 0u ||
      raw_subscription->get_publisher_count() == 0u || progress_subscription->get_publisher_count() == 0u))
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
    while (std::chrono::steady_clock::now() < message_deadline && (!carrot_received || !received_progress.valid))
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(carrot_received);
    ASSERT_TRUE(received_progress.valid);
    EXPECT_FALSE(received_progress.controller_instance.empty());
    EXPECT_EQ(received_progress.path_header, makeLifecyclePath().header);
    EXPECT_EQ(received_progress.path_fingerprint, path_identity::fingerprintPath(makeLifecyclePath()));
    EXPECT_EQ(received_progress.plan_revision, 1u);
    EXPECT_EQ(received_progress.segment_index, 0u);
    EXPECT_DOUBLE_EQ(received_progress.segment_fraction, 0.0);
    const auto instance = received_progress.controller_instance;
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
      pose.pose.position.x = 0.1;
      pose.pose.orientation.z = std::sin(0.1);
      pose.pose.orientation.w = std::cos(0.1);
      active_command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    }
    const auto raw_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < raw_deadline &&
      received_raw.header.stamp != active_command.header.stamp)
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_EQ(received_raw.header, active_command.header);
    EXPECT_EQ(received_raw.header.frame_id, "base_link");
    // The offset requires negative-x correction; acceleration limiting delays it.
    EXPECT_LT(received_raw.twist.linear.x, -0.05);
    EXPECT_GT(std::abs(received_raw.twist.linear.x), std::abs(active_command.twist.linear.x));
    EXPECT_GT(received_raw.twist.linear.y, 0.0);
    EXPECT_NEAR(received_raw.twist.angular.z, -0.2, 1e-12);
    EXPECT_EQ(
      observer->count_publishers("/pid_tracking_controller/tracking_diagnostics"), 0u);
    EXPECT_NE(log_capture.output().find("PID_LOG parameter_snapshot_begin"), std::string::npos);
    EXPECT_NE(log_capture.output().find("PID_LOG parameter_snapshot_end"), std::string::npos);
    EXPECT_NE(log_capture.output().find("PID_LOG holonomic plan_revision="), std::string::npos);
    EXPECT_NE(log_capture.output().find("external_speed_limit="), std::string::npos);

    if (costmap_configured) {
      controller.reset();
      for (int i = 0; i < 20 && received_progress.valid; ++i) {
        executor.spin_some(); std::this_thread::sleep_for(10ms);
      }
      EXPECT_FALSE(received_progress.valid);
      EXPECT_EQ(received_progress.controller_instance, instance);
      // Reloading identical geometry is a new execution revision, not a new hash.
      controller.setPlan(makeLifecyclePath());
      controller.computeVelocityCommands(makePose(0, 0), geometry_msgs::msg::Twist(), nullptr);
      for (int i = 0; i < 20 && (!received_progress.valid || received_progress.plan_revision != 3u); ++i) {
        executor.spin_some(); std::this_thread::sleep_for(10ms);
      }
      EXPECT_TRUE(received_progress.valid);
      EXPECT_EQ(received_progress.plan_revision, 3u);
      EXPECT_EQ(received_progress.path_fingerprint, path_identity::fingerprintPath(makeLifecyclePath()));
      auto rotation_path = makeLifecyclePath();
      rotation_path.poses = {makePose(0, 0), makePose(0, 0),
        makePose(0, M_PI_2), makePose(0, M_PI_2)};
      controller.setPlan(rotation_path);
      ASSERT_EQ(rcl_set_ros_time_override(control_clock->get_clock_handle(), 2000000000LL), RCL_RET_OK);
      controller.computeVelocityCommands(makePose(0, 0), geometry_msgs::msg::Twist(), nullptr);
      for (int i = 0; i < 30 && (!received_progress.valid || received_progress.plan_revision != 4u); ++i) {
        executor.spin_some(); std::this_thread::sleep_for(10ms);
      }
      EXPECT_EQ(received_progress.segment_index, 1u) << "duplicate prefix before pure rotation";
      for (int i = 1; i <= 15; ++i) {
        ASSERT_EQ(rcl_set_ros_time_override(control_clock->get_clock_handle(),
          2000000000LL + i * 100000000LL), RCL_RET_OK);
        controller.computeVelocityCommands(makePose(0, M_PI_2), geometry_msgs::msg::Twist(), nullptr);
        executor.spin_some(); std::this_thread::sleep_for(10ms);
      }
      for (int i = 0; i < 10; ++i) {executor.spin_some(); std::this_thread::sleep_for(10ms);}
      EXPECT_FALSE(received_progress.rotating);
      EXPECT_EQ(received_progress.segment_index, 3u) << "consumed rotation must publish terminal original index";
      controller.deactivate();
      for (int i = 0; i < 20 && received_progress.valid; ++i) {
        executor.spin_some(); std::this_thread::sleep_for(10ms);
      }
      EXPECT_FALSE(received_progress.valid);
      controller.cleanup();
      costmap->cleanup();
    }
    executor.remove_node(observer);
    executor.remove_node(parent->get_node_base_interface());
  }

  rclcpp::shutdown();
}

TEST(LifecycleSmoke, unavailableObservationDoesNotChangeCommands)
{
  rclcpp::init(0, nullptr);
  {
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("observation_control_map");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().label(), "inactive");
    // Declare an observational source without starting a map-update thread.
    costmap->declare_parameter("test_layer.enabled", true);
    costmap->declare_parameter("test_layer.observation_sources", "scan");
    costmap->declare_parameter("test_layer.scan.topic", "/absent_observation_scan");
    costmap->declare_parameter("test_layer.scan.data_type", "LaserScan");
    costmap->declare_parameter("test_layer.scan.expected_update_rate", 0.);
    costmap->declare_parameter("test_layer.scan.marking", true);
    costmap->declare_parameter("test_layer.scan.clearing", true);
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"test_layer"}));
    std::array<std::vector<geometry_msgs::msg::Twist>, 2> commands;
    for (int enabled = 0; enabled < 2; ++enabled) {
      auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("observation_control_parent");
      parent->declare_parameter("Controller.observe_execution_inputs", bool(enabled));
      parent->declare_parameter("Controller.observed_smoothed_velocity_topic", "/absent_smoother");
      // The same observation path must also work when angular filtering is disabled.
      parent->declare_parameter("Controller.omega_feedback_filter_tau", 0.0);
      parent->declare_parameter("odom_topic", "/motion_test_odom");
      auto odom_pub = parent->create_publisher<nav_msgs::msg::Odometry>("/motion_test_odom", 1);
      odom_pub->on_activate();
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(parent->get_node_base_interface());
      auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
      auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
      PidTrackingController controller;
      controller.configure(parent, "Controller", tf, costmap); controller.activate();
      PidTrackingControllerTestAccess::setClock(controller, clock);
      controller.setPlan(makeLifecyclePath());
      for (int i = 0; i < 60; ++i) {
        if (i == 20) {
          nav_msgs::msg::Path rotation_path;
          rotation_path.header.frame_id = "map";
          rotation_path.poses = {makePose(0., 0.), makePose(0., 0.2), makePose(0.6, 0.2)};
          controller.setPlan(rotation_path);
        } else if (i == 40) {
          controller.setPlan(makeLifecyclePath());
        }
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL + i * 66666667LL), RCL_RET_OK);
        commands[enabled].push_back(controller.computeVelocityCommands(
            makePose(0., 0.), geometry_msgs::msg::Twist(), nullptr).twist);
        if (enabled && i == 39) {
          const auto motion = PidTrackingControllerTestAccess::motionObservation(controller);
          ASSERT_NE(motion, nullptr);
          const auto sample = motion->snapshot(parent->now().nanoseconds())[MotionObservation::Candidate].sample;
          EXPECT_DOUBLE_EQ(sample.omega, commands[enabled].back().angular.z);
          EXPECT_EQ(sample.revision, 2u);
        }
        nav_msgs::msg::Odometry odom; odom.header.stamp = parent->now();
        odom.header.frame_id = "odom"; odom.child_frame_id = "base_link";
        odom.twist.twist.linear.y = -.12; odom.twist.twist.angular.z = .08;
        odom_pub->publish(odom); executor.spin_some();
        std::this_thread::sleep_for(10ms);
      }
      auto motion = PidTrackingControllerTestAccess::motionObservation(controller);
      if (enabled) {
        ASSERT_NE(motion, nullptr);
        const auto end = std::chrono::steady_clock::now() + 2s;
        while (!motion->snapshot(parent->now().nanoseconds())[3].sample.count &&
          std::chrono::steady_clock::now() < end) {executor.spin_some(); std::this_thread::sleep_for(10ms);}
        const auto evidence = motion->snapshot(parent->now().nanoseconds());
        EXPECT_EQ(odom_pub->get_subscription_count(), 1u);
        EXPECT_STREQ(evidence[3].state, "valid");
        EXPECT_DOUBLE_EQ(evidence[3].sample.vy, -.12); EXPECT_DOUBLE_EQ(evidence[3].sample.omega, .08);
        EXPECT_EQ(evidence[0].sample.vx, commands[enabled].back().linear.x);
        EXPECT_EQ(evidence[0].sample.omega, commands[enabled].back().angular.z);
        EXPECT_EQ(evidence[0].sample.revision, 3u);
      }
      controller.deactivate(); controller.cleanup();
      if (motion) {EXPECT_STREQ(motion->snapshot(parent->now().nanoseconds())[0].state, "missing");}
    }
    EXPECT_GT(commands[1][39].angular.z, 0.);  // The comparison includes admitted rotation.
    EXPECT_DOUBLE_EQ(commands[1][39].linear.x, 0.);
    EXPECT_GT(commands[1].back().linear.x, 0.);
    EXPECT_EQ(commands[0], commands[1]);
    costmap->cleanup();
  }
  rclcpp::shutdown();
}

TEST(LifecycleSmoke, rotationParametersAffectTurnsWithoutChangingTranslationYaw)
{
  rclcpp::init(0, nullptr);
  {
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("rotation_parameters_map");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().label(), "inactive");
    std::vector<double> turns, translations;
    for (const auto & settings : {std::pair<double, double>{0.5, 0.0}, {2.5, 0.0}, {2.5, 0.6}}) {
      auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("rotation_parameters_parent");
      parent->declare_parameter("Controller.rotation_kp_yaw", settings.first);
      parent->declare_parameter("Controller.rotation_response_delay", settings.second);
      parent->declare_parameter("Controller.kp_yaw", 0.5);
      parent->declare_parameter("Controller.omega_feedback_filter_tau", 0.0);
      auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
      auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
      PidTrackingController controller;
      controller.configure(parent, "Controller", tf, costmap); controller.activate();
      PidTrackingControllerTestAccess::setClock(controller, clock);
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.poses = {makePose(0., 0.), makePose(0., 0.15)};
      controller.setPlan(path);
      for (int i = 0; i < 30; ++i) {
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL + i * 66666667LL), RCL_RET_OK);
        const auto output = controller.computeVelocityCommands(makePose(0., 0.), geometry_msgs::msg::Twist(), nullptr);
        EXPECT_DOUBLE_EQ(output.twist.linear.x, 0.0);
        if (i == 29) {turns.push_back(output.twist.angular.z);}
      }
      controller.reset(); controller.setPlan(makeLifecyclePath());
      for (int i = 0; i < 20; ++i) {
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 4000000000LL + i * 66666667LL), RCL_RET_OK);
        const auto output = controller.computeVelocityCommands(makePose(0., 0.2), geometry_msgs::msg::Twist(), nullptr);
        if (i == 19) {translations.push_back(output.twist.angular.z);}
      }
      controller.deactivate(); controller.cleanup();
    }
    EXPECT_GT(turns[1], turns[0]);
    EXPECT_LT(turns[2], turns[1]);
    for (const auto omega : translations) {EXPECT_NEAR(omega, -0.1, 1e-12);}
    costmap->cleanup();
  }
  rclcpp::shutdown();
}

TEST(LifecycleSmoke, enabledSafetyRejectsMotionWithoutInputsIncludingReseedAndNewTask)
{
  rclcpp::init(0, nullptr);
  {
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("safety_no_inputs");
    parent->declare_parameter("Controller.execution_safety.enabled", true);
    parent->declare_parameter("controller_frequency", 15.0);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("safety_no_inputs_map");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
    PidTrackingController controller;
    controller.configure(parent, "Controller", tf, costmap);
    controller.activate();
    PidTrackingControllerTestAccess::setClock(controller, clock);
    geometry_msgs::msg::Twist measured;
    measured.linear.x = .4; measured.linear.y = -.2; measured.angular.z = .1;
    for (int task = 0; task < 2; ++task) {
      controller.setPlan(makeLifecyclePath());
      for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(),
            1000000000LL + (task * 10 + i) * 66666667LL), RCL_RET_OK);
        const auto cmd = controller.computeVelocityCommands(makePose(0., 0.), measured, nullptr);
        EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.);
        EXPECT_DOUBLE_EQ(cmd.twist.linear.y, 0.);
        EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.);
      }
      controller.reset();
    }
    controller.deactivate(); controller.cleanup(); costmap->cleanup();
  }
  rclcpp::shutdown();
}

TEST(LifecycleSmoke, safetyInputsRecoverRotationEntryStopOnCollisionAndResumeAfterRemoval)
{
  rclcpp::init(0, nullptr);
  {
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("gate_live_inputs");
    parent->declare_parameter("Controller.execution_safety.enabled", true);
    parent->declare_parameter("Controller.omega_feedback_filter_tau", 0.0);
    parent->declare_parameter("controller_frequency", 15.0);
    parent->declare_parameter("odom_topic", "/gate_live_odom");
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("gate_live_map");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
    costmap->set_parameter(rclcpp::Parameter("robot_base_frame", "base_link"));
    costmap->configure();
    costmap->declare_parameter("test_layer.enabled", true);
    costmap->declare_parameter("test_layer.observation_sources", "scan");
    costmap->declare_parameter("test_layer.scan.topic", "gate_scan");
    costmap->declare_parameter("test_layer.scan.data_type", "LaserScan");
    costmap->declare_parameter("test_layer.scan.expected_update_rate", 0.0);
    costmap->declare_parameter("test_layer.scan.marking", true);
    costmap->declare_parameter("test_layer.scan.clearing", true);
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"test_layer"}));
    auto publisher = std::make_shared<rclcpp::Node>("gate_live_publisher");
    const auto topic = [&](const char * name) {
        return costmap->get_node_topics_interface()->resolve_topic_name(name);
      };
    auto * live = costmap->getCostmap();
    live->resizeMap(200, 200, .05, -5., -5.);
    costmap->getLayeredCostmap()->addPlugin(std::make_shared<TestCostmapLayer>());
    costmap->getLayeredCostmap()->updateMap(0., 0., 0.);
    auto footprint_pub = publisher->create_publisher<geometry_msgs::msg::PolygonStamped>(
        topic("published_footprint"), 1);
    auto scan_pub = publisher->create_publisher<sensor_msgs::msg::LaserScan>(
        topic("gate_scan"), rclcpp::SensorDataQoS());
    auto odom_pub = publisher->create_publisher<nav_msgs::msg::Odometry>("/gate_live_odom", 1);
    auto risk_pub = publisher->create_publisher<msg::PathRiskState>("path_risk/control_state", 1);
    uint8_t risk_state = msg::PathRiskState::CLEARANCE_LOW;
    builtin_interfaces::msg::Time map_stamp;
    auto progress_sub = parent->create_subscription<msg::TrackingProgress>(
      "pid_tracking_controller/tracking_progress", 1, [&](msg::TrackingProgress::ConstSharedPtr p) {
        if (!p->valid) {return;}
        msg::PathRiskState risk;
        risk.header = p->header; risk.header.stamp = parent->now(); risk.path_header = p->path_header;
        risk.controller_instance = p->controller_instance; risk.plan_revision = p->plan_revision;
        risk.path_fingerprint = p->path_fingerprint; risk.progress_stamp = p->header.stamp;
        risk.map_stamp = map_stamp; risk.state = risk_state; risk_pub->publish(risk);
      });
    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(parent->get_node_base_interface());
    PidTrackingController controller;
    controller.configure(parent, "Controller", tf, costmap); controller.activate();
    nav_msgs::msg::Path path; path.header.frame_id = "map";
    path.poses = {makePose(0., 0.), makePose(0., .2), makePose(.6, .2)};
    controller.setPlan(path);
    auto pose = makePose(0., 0.); pose.header.stamp = parent->now();
    auto command = controller.computeVelocityCommands(pose, {}, nullptr);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.);  // Startup has no input evidence.
    const auto run = [&](unsigned char cost, double yaw, bool want_motion, bool translation = false) {
        bool observed = false;
        risk_state = cost == 254 ? msg::PathRiskState::COLLISION_AHEAD : msg::PathRiskState::CLEARANCE_LOW;
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline && !observed) {
          pose = makePose(0., yaw); pose.header.stamp = parent->now();
          geometry_msgs::msg::TransformStamped transform;
          transform.header = pose.header; transform.child_frame_id = "base_link";
          transform.transform.rotation = pose.pose.orientation; tf->setTransform(transform, "test", false);
          map_stamp = pose.header.stamp;
          {
            std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
            std::fill_n(live->getCharMap(), 40000, cost);
          }
          geometry_msgs::msg::PolygonStamped footprint; footprint.header = pose.header;
          for (auto xy : {std::pair<float, float>{-.1, -.1}, {.1, -.1}, {.1, .1}, {-.1, .1}}) {
            geometry_msgs::msg::Point32 p;
            p.x = xy.first * std::cos(yaw) - xy.second * std::sin(yaw);
            p.y = xy.first * std::sin(yaw) + xy.second * std::cos(yaw);
            footprint.polygon.points.push_back(p);
          }
          footprint_pub->publish(footprint);
          sensor_msgs::msg::LaserScan scan; scan.header = pose.header;
          scan.angle_max = .1; scan.angle_increment = .1; scan.range_min = .1; scan.range_max = 10.;
          scan.ranges = {1., 2.}; scan_pub->publish(scan);
          nav_msgs::msg::Odometry odom; odom.header = pose.header; odom.child_frame_id = "base_link";
          odom_pub->publish(odom);
          std::this_thread::sleep_for(30ms);
          const auto odom_deadline = std::chrono::steady_clock::now() + 100ms;
          const auto motion = PidTrackingControllerTestAccess::motionObservation(controller);
          bool anchored = false;
          while (!anchored && std::chrono::steady_clock::now() < odom_deadline) {
            executor.spin_some();
            anchored = motion->snapshot(parent->now().nanoseconds())[MotionObservation::Odometry].sample.stamp_ns ==
                rclcpp::Time(pose.header.stamp).nanoseconds();
            if (!anchored) {std::this_thread::sleep_for(1ms);}
          }
          EXPECT_TRUE(anchored) << "Fixture must deliver the pose-time odometry before control";
          if (!anchored) {return false;}
          command = controller.computeVelocityCommands(pose, {}, nullptr);
          const bool moving = std::hypot(command.twist.linear.x, command.twist.linear.y) > 1e-5 ||
              std::abs(command.twist.angular.z) > 1e-5;
          observed = translation ? std::hypot(command.twist.linear.x, command.twist.linear.y) > 1e-5 :
              moving == want_motion;
        }
        return observed;
      };
    ASSERT_TRUE(run(0, 0., true));
    EXPECT_GT(command.twist.angular.z, 0.);
    ASSERT_TRUE(run(254, 0., false));
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.);
    // Full footprint still blocked: repeated cycles must not produce stop/go pulses.
    for (int i = 0; i < 5; ++i) {ASSERT_TRUE(run(254, 0., false));}
    ASSERT_TRUE(run(0, 0., true));
    // Actual rotation completion must advance even if a previous gate stop occurred.
    ASSERT_TRUE(run(0, .2, true, true));
    controller.deactivate(); controller.activate(); controller.setPlan(path);
    pose.header.stamp = parent->now();
    command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.);
    ASSERT_TRUE(run(0, 0., true));
    controller.deactivate(); controller.cleanup(); costmap->cleanup();
  }
  rclcpp::shutdown();
}

TEST(LifecycleSmoke, configuredLimitsAndGainsReachActiveOutput)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("Controller.nominal_tracking_speed", 0.1);
    options.append_parameter_override("Controller.max_xy_speed", 0.4);
    options.append_parameter_override("Controller.max_vx", 0.4);
    options.append_parameter_override("Controller.max_vy", 0.4);
    options.append_parameter_override("Controller.kp_y", 0.5);
    options.append_parameter_override("Controller.fixed_heading_yaw_enter_error", 0.1);
    options.append_parameter_override("Controller.fixed_heading_yaw_exit_error", 0.05);
    // Legacy activation constraints must no longer gate the active holonomic chain.
    options.append_parameter_override("Controller.cruise_speed", 0.8);
    options.append_parameter_override("Controller.max_v", 0.1);
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("active_parent", options);
    geometry_msgs::msg::TwistStamped received_raw;
    auto raw_subscription = parent->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/tracking_cmd_vel_raw", rclcpp::QoS(10).reliable(),
      [&](const geometry_msgs::msg::TwistStamped::SharedPtr message) {received_raw = *message;});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(parent->get_node_base_interface());
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
    // 0.06 rad exceeds the default 2 degrees but is below the configured 0.1.
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1050000000LL), RCL_RET_OK);
    const auto held = controller.computeVelocityCommands(
      makePose(0.0, 0.06), geometry_msgs::msg::Twist(), nullptr);
    EXPECT_DOUBLE_EQ(held.twist.angular.z, 0.0);
    controller.deactivate();
    controller.activate();
    controller.setSpeedLimit(0.0, false);
    auto pose = makePose(0.0, 0.0);
    pose.pose.position.y = 0.1;
    controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1100000000LL), RCL_RET_OK);
    controller.setPlan(makeLifecyclePath());
    const auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
    EXPECT_LT(command.twist.linear.y, 0.0);
    EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (received_raw.header.stamp != command.header.stamp &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_EQ(received_raw.header.stamp, command.header.stamp);
    EXPECT_NEAR(received_raw.twist.linear.y, -0.05, 1e-12);
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
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter("Controller.kp_y", 0.5)).successful);
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
      "Controller.fixed_heading_yaw_hold", false)).successful);
    controller.configure(parent, "Controller", tf, costmap);
    PidTrackingControllerTestAccess::setClock(controller, clock);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1200000000LL), RCL_RET_OK);
    controller.setPlan(makeLifecyclePath());
    controller.computeVelocityCommands(makePose(0.0, 0.01), geometry_msgs::msg::Twist(), nullptr);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1300000000LL), RCL_RET_OK);
    const auto unfiltered = controller.computeVelocityCommands(
      makePose(0.0, 0.01), geometry_msgs::msg::Twist(), nullptr);
    EXPECT_NEAR(unfiltered.twist.angular.z, -0.01, 1e-12);
    controller.cleanup();
    costmap->cleanup();
  }
  rclcpp::shutdown();
}

TEST(LifecycleSmoke, odometryCallbackFiltersReseedingAndFallsBackAcrossLifecycle)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("odom_topic", "/omega_feedback_test_odom");
    options.append_parameter_override("Controller.base_frame", "rbs_base_link");
    auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("omega_feedback_parent", options);
    auto publisher_node = std::make_shared<rclcpp::Node>("omega_feedback_publisher");
    auto publisher = publisher_node->create_publisher<nav_msgs::msg::Odometry>(
      "/omega_feedback_test_odom", rclcpp::QoS(10));
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("omega_feedback_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>()));
    costmap->configure();
    ASSERT_EQ(costmap->get_current_state().label(), "inactive");
    auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
    PidTrackingController controller;
    controller.configure(parent, "Controller", tf, costmap);
    controller.activate();
    PidTrackingControllerTestAccess::setClock(controller, clock);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(parent->get_node_base_interface());
    executor.add_node(publisher_node);
    const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
    while (publisher->get_subscription_count() == 0u &&
      std::chrono::steady_clock::now() < discovery_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);
    const auto send_and_check = [&](double omega, int64_t stamp, const std::string & frame,
        double expected, bool valid) {
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(stamp);
        message.child_frame_id = frame;
        message.twist.twist.angular.z = omega;
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), stamp), RCL_RET_OK);
        controller.reset();
        controller.setPlan(makeLifecyclePath());
        publisher->publish(message);
        geometry_msgs::msg::Twist measured;
        measured.linear.x = 0.1;
        // Distinguish fallback from the received sample to verify callback consumption.
        measured.angular.z = valid ? omega + 0.1 : omega;
        bool matched = false;
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < deadline && !matched) {
          executor.spin_some();
          const auto command = controller.computeVelocityCommands(makePose(0.0, 0.0), measured, nullptr);
          matched = std::abs(command.twist.angular.z - expected) < 1e-10;
          EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.1);
          if (!matched) {
            std::this_thread::sleep_for(5ms);
          }
        }
        EXPECT_TRUE(matched) << "omega=" << omega << " expected=" << expected;
      };
    send_and_check(0.2, 1000000000LL, "rbs_base_link", 0.2, true);
    send_and_check(0.1, 1010000000LL, "rbs_base_link", 0.1818730753078, true);
    send_and_check(-0.2, 1020000000LL, "wrong_frame", -0.2, false);
    send_and_check(0.3, 1030000000LL, "rbs_base_link", 0.3, true);
    controller.deactivate();
    controller.activate();
    send_and_check(-0.1, 1040000000LL, "rbs_base_link", -0.1, true);
    controller.deactivate();
    controller.cleanup();
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter("Controller.omega_feedback_filter_tau", 0.0)).successful);
    controller.configure(parent, "Controller", tf, costmap);
    controller.activate();
    PidTrackingControllerTestAccess::setClock(controller, clock);
    send_and_check(0.1, 1050000000LL, "rbs_base_link", 0.1, false);
    controller.deactivate();
    controller.cleanup();
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter("Controller.omega_feedback_filter_tau", -0.1)).successful);
    EXPECT_THROW(controller.configure(parent, "Controller", tf, costmap), std::invalid_argument);
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
    // The failed configuration declared this default from the then-zero max_alpha.
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
      "PidTrackingController.rotation_max_alpha", 0.5)).successful);
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
      "PidTrackingController.fixed_heading_yaw_enter_error", 0.01)).successful);
    EXPECT_THROW(
      controller.configure(parent, "PidTrackingController", tf, costmap), std::invalid_argument);
    ASSERT_TRUE(parent->set_parameter(rclcpp::Parameter(
      "PidTrackingController.fixed_heading_yaw_enter_error", 0.03490658503988659)).successful);
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
