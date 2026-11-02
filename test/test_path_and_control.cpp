#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "controller_test_access.hpp"
#include "angular_velocity_feedback_filter.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "rclcpp/clock.hpp"
#include "rcl/time.h"

namespace pid_tracking_controller
{
namespace
{

nav_msgs::msg::Path makeStraightPath(std::initializer_list<double> x_coordinates)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const double x : x_coordinates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped makeRobotPose(double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

geometry_msgs::msg::PoseStamped makeRobotPoseAt(
  double x, double y, double yaw = 0.0)
{
  auto pose = makeRobotPose(yaw);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

nav_msgs::msg::Path makeDenseHairpinPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const auto append = [&path](double x, double y) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    };

  for (int index = 0; index <= 20; ++index) {
    append(0.05 * static_cast<double>(index), 0.0);
  }
  append(1.0, 0.05);
  append(1.0, 0.10);
  for (int index = 19; index >= 0; --index) {
    append(0.05 * static_cast<double>(index), 0.10);
  }
  return path;
}

class CommandContractTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    PidTrackingControllerTestAccess::setCostmap(controller_, &costmap_);
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    PidTrackingControllerTestAccess::setClock(controller_, clock_);
    speed_.nominal_tracking_speed = 0.2;
    speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.2;
    speed_.max_omega = 0.3;
    speed_.max_ax = speed_.max_ay = speed_.a_ref_up = speed_.a_ref_down = 0.2;
    speed_.yaw_feedback_reserve = 0.05;
    limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.2;
    limits_.max_omega = 0.3;
    limits_.max_ax = limits_.max_ay = 0.2;
    limits_.max_decel_x = limits_.max_decel_y = 0.2;
    limits_.max_alpha = 0.5;
    initialize();
    controller_.setPlan(makeStraightPath({0.0, 1.0, 3.0}));
  }

  void initialize()
  {
    PidTrackingControllerTestAccess::initializeHolonomic(
      controller_, speed_, limits_, {0.01, 0.01, 0.01, 0.3});
  }

  geometry_msgs::msg::TwistStamped step(
    const geometry_msgs::msg::PoseStamped & pose = makeRobotPose(),
    const geometry_msgs::msg::Twist & measured = geometry_msgs::msg::Twist(), double dt = 0.1)
  {
    now_ns_ += static_cast<int64_t>(std::llround(dt * 1e9));
    EXPECT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), now_ns_), RCL_RET_OK);
    return controller_.computeVelocityCommands(pose, measured, nullptr);
  }

  void setRotationPath(bool append_translation)
  {
    auto path = makeStraightPath({0.0});
    path.poses.push_back(makeRobotPose(0.4));
    path.poses.push_back(makeRobotPose(0.2));
    if (append_translation) {
      path.poses.push_back(makeRobotPoseAt(1.0, 0.0, 0.2));
      path.poses.push_back(makeRobotPoseAt(2.0, 0.0, 0.2));
    }
    controller_.setPlan(path);
  }

  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  int64_t now_ns_{1000000000LL};
  HolonomicSpeedPlannerConfig speed_;
  HolonomicCommandLimiterConfig limits_;
  nav2_costmap_2d::Costmap2D costmap_{100, 100, 0.05, -2.5, -2.5};
  PidTrackingController controller_;
};

TEST_F(CommandContractTest, filtersOnlyAngularReseedingAndKeepsHistoryAcrossTasks)
{
  auto filter = std::make_shared<AngularVelocityFeedbackFilter>(0.05);
  PidTrackingControllerTestAccess::setOmegaFeedbackFilter(controller_, filter);
  filter->update(0.0, now_ns_ + 90000000LL);
  filter->update(0.1, now_ns_ + 100000000LL);
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.1;
  measured.linear.y = -0.05;
  measured.angular.z = 0.1;
  const auto first = step(makeRobotPose(), measured).twist;
  EXPECT_NEAR(first.angular.z, 0.0181269246922, 1e-12);
  EXPECT_DOUBLE_EQ(first.linear.x, measured.linear.x);
  EXPECT_DOUBLE_EQ(first.linear.y, measured.linear.y);
  controller_.reset();
  controller_.setPlan(makeStraightPath({0.0, 1.0}));
  EXPECT_NEAR(step(makeRobotPose(), measured, 0.01).twist.angular.z, first.angular.z, 1e-12);
  // A control gap with no fresh odometry must use the original feedback.
  EXPECT_DOUBLE_EQ(step(makeRobotPose(), measured, 0.6).twist.angular.z, measured.angular.z);
}

TEST_F(CommandContractTest, filteredNearZeroCannotAdmitRotationWhileRawFeedbackMoves)
{
  auto filter = std::make_shared<AngularVelocityFeedbackFilter>(0.05);
  PidTrackingControllerTestAccess::setOmegaFeedbackFilter(controller_, filter);
  setRotationPath(true);
  geometry_msgs::msg::Twist moving;
  moving.angular.z = 0.02;  // Above this fixture's 0.01 rad/s raw stop threshold.
  for (int i = 0; i < 15; ++i) {
    filter->update(0.0, now_ns_ + 99000000LL);
    filter->update(moving.angular.z, now_ns_ + 100000000LL);
    const auto output = step(makeRobotPose(), moving).twist;
    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_LT(std::abs(output.angular.z), 0.01);
  }
}

TEST_F(CommandContractTest, filteredNearZeroCannotCompleteRotationWhileRawFeedbackMoves)
{
  setRotationPath(true);
  const auto aligned = makeRobotPose(0.2);
  for (int i = 0; i < 4; ++i) {
    step(aligned);  // Admit rotation with genuine stopped measurements.
  }
  auto filter = std::make_shared<AngularVelocityFeedbackFilter>(0.05);
  PidTrackingControllerTestAccess::setOmegaFeedbackFilter(controller_, filter);
  geometry_msgs::msg::Twist moving;
  moving.angular.z = 0.02;
  for (int i = 0; i < 10; ++i) {
    filter->update(0.0, now_ns_ + 99000000LL);
    filter->update(moving.angular.z, now_ns_ + 100000000LL);
    EXPECT_DOUBLE_EQ(step(aligned, moving).twist.linear.x, 0.0);
  }
  for (int i = 0; i < 5; ++i) {
    step(aligned);
  }
  EXPECT_GT(step(aligned).twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, growingOutwardTurnErrorHoldsAccelerationButPreservesCorrection)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 60; ++i) {
    path.poses.push_back(makeRobotPoseAt(2.0 * std::sin(0.05 * i),
      2.0 * (1.0 - std::cos(0.05 * i))));
  }
  const double tangent = 0.275;  // Outgoing chord at theta=0.25.
  const double tx = std::cos(tangent), ty = std::sin(tangent);
  const double nx = -ty, ny = tx;
  for (const double side : {-1.0, 1.0}) {
    const auto run = [&](bool guard_enabled) {
        speed_.nominal_tracking_speed = speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.8;
        speed_.max_ax = speed_.max_ay = 1.1;
        speed_.a_ref_up = 0.2;
        speed_.a_ref_down = speed_.stopping_deceleration = 0.4;
        speed_.tracking_guard.enter_error = guard_enabled ? 0.03 : 10.0;
        speed_.tracking_guard.exit_error = guard_enabled ? 0.015 : 5.0;
        limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.8;
        limits_.max_ax = limits_.max_ay = 1.1;
        initialize();
        controller_.setPlan(path);
        geometry_msgs::msg::Twist measured;
        measured.linear.x = 0.2 * tx + side * 0.1 * nx;
        measured.linear.y = 0.2 * ty + side * 0.1 * ny;
        geometry_msgs::msg::Twist result;
        for (int i = 0; i < 12; ++i) {
          const double error = side * (0.04 + 0.003 * i);
          result = step(makeRobotPoseAt(path.poses[5].pose.position.x + error * nx,
            path.poses[5].pose.position.y + error * ny), measured).twist;
        }
        return result;
      };
    const auto guarded = run(true);
    const auto baseline = run(false);
    // The inside offset projects onto the incoming chord (0.225 rad),
    // while the outside offset keeps the outgoing chord (0.275 rad).
    const double accepted_tangent = side > 0.0 ? 0.225 : 0.275;
    const double along_x = std::cos(accepted_tangent), along_y = std::sin(accepted_tangent);
    EXPECT_LT(along_x * guarded.linear.x + along_y * guarded.linear.y,
      along_x * baseline.linear.x + along_y * baseline.linear.y - 0.05);
    EXPECT_NEAR(-along_y * guarded.linear.x + along_x * guarded.linear.y,
      -along_y * baseline.linear.x + along_x * baseline.linear.y, 1e-9);
    EXPECT_GT(-side * (-along_y * guarded.linear.x + along_x * guarded.linear.y), 0.06);
  }
}

TEST_F(CommandContractTest, fixedHeadingTranslationSuppressesSmallYawWithHysteresis)
{
  const double degree = std::acos(-1.0) / 180.0;
  const auto settled = [&](double yaw) {
      geometry_msgs::msg::Twist result;
      for (int cycle = 0; cycle < 10; ++cycle) {
        result = step(makeRobotPoseAt(0.0, -0.03, yaw)).twist;
      }
      return result;
    };
  const auto holding = settled(1.5 * degree);
  EXPECT_DOUBLE_EQ(holding.angular.z, 0.0);
  // Suppressing yaw must preserve both translation components and position correction.
  EXPECT_GT(holding.linear.x, 0.0);
  EXPECT_GT(holding.linear.y, 0.0);
  EXPECT_NEAR(settled(2.5 * degree).angular.z, -2.5 * degree, 1e-12);
  EXPECT_NEAR(settled(1.5 * degree).angular.z, -1.5 * degree, 1e-12);
  EXPECT_DOUBLE_EQ(settled(0.5 * degree).angular.z, 0.0);
  EXPECT_DOUBLE_EQ(settled(-1.5 * degree).angular.z, 0.0);
}

TEST_F(CommandContractTest, yawHoldResetOnNewPathAndControlGap)
{
  const double degree = std::acos(-1.0) / 180.0;
  step(makeRobotPose(3.0 * degree));
  EXPECT_LT(step(makeRobotPose(3.0 * degree)).twist.angular.z, -0.03);
  controller_.setPlan(makeStraightPath({0.0, 1.0, 3.0}));
  for (int cycle = 0; cycle < 3; ++cycle) {
    step(makeRobotPose(1.5 * degree));
  }
  EXPECT_DOUBLE_EQ(step(makeRobotPose(1.5 * degree)).twist.angular.z, 0.0);
  step(makeRobotPose(3.0 * degree));
  step(makeRobotPose(3.0 * degree));
  step(makeRobotPose(1.5 * degree), geometry_msgs::msg::Twist(), 1.0);
  EXPECT_DOUBLE_EQ(step(makeRobotPose(1.5 * degree)).twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, yawHoldExitRetainsAngularDecelerationLimit)
{
  step(makeRobotPose(0.2));
  geometry_msgs::msg::Twist output;
  for (int cycle = 0; cycle < 10; ++cycle) {
    output = step(makeRobotPose(0.2)).twist;
  }
  EXPECT_NEAR(output.angular.z, -0.2, 1e-12);
  const auto stopping = step(makeRobotPose(0.005)).twist;
  EXPECT_LT(stopping.angular.z, 0.0);
  EXPECT_LE(std::abs(stopping.angular.z - output.angular.z), limits_.max_alpha * 0.1 + 1e-12);
  for (int cycle = 0; cycle < 10; ++cycle) {
    output = step(makeRobotPose(0.005)).twist;
  }
  EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST_F(CommandContractTest, saturatedFeedforwardPreservesNormalPositionCorrection)
{
  speed_.nominal_tracking_speed = speed_.max_vx = speed_.max_xy_speed = 0.8;
  speed_.max_vy = 0.4;
  speed_.max_ax = speed_.max_ay = speed_.a_ref_up = speed_.a_ref_down = 0.8;
  limits_.max_vx = limits_.max_xy_speed = 0.8;
  limits_.max_vy = 0.4;
  limits_.max_ax = limits_.max_ay = 0.8;
  initialize();
  auto path = makeStraightPath({0.0});
  path.poses.push_back(makeRobotPoseAt(12.0, 16.0));
  controller_.setPlan(path);
  const auto pose = makeRobotPoseAt(0.08, -0.06);
  geometry_msgs::msg::Twist output;
  for (int cycle = 0; cycle < 20; ++cycle) {
    output = step(pose).twist;
  }
  // t=(0.6,0.8), feedback=(-0.08,0.06): v must fall from 0.5 to 0.425.
  EXPECT_NEAR(output.linear.x, 0.175, 1e-6);
  EXPECT_NEAR(output.linear.y, 0.4, 1e-6);
  EXPECT_NEAR(-0.8 * output.linear.x + 0.6 * output.linear.y, 0.1, 1e-6);
}

TEST_F(CommandContractTest, combinedSpeedCapPreservesLateralPositionCorrection)
{
  speed_.nominal_tracking_speed = speed_.max_xy_speed = 0.5;
  speed_.max_vx = speed_.max_vy = 1.0;
  limits_.max_xy_speed = 0.5;
  limits_.max_vx = limits_.max_vy = 1.0;
  initialize();
  auto path = makeStraightPath({0.0, 20.0});
  path.poses.back().pose.orientation = makeRobotPose(1.0).pose.orientation;
  controller_.setPlan(path);
  geometry_msgs::msg::Twist output;
  for (int cycle = 0; cycle < 40; ++cycle) {
    output = step(makeRobotPoseAt(0.0, -0.2)).twist;
  }
  EXPECT_NEAR(output.linear.x, std::sqrt(0.21), 1e-6);
  EXPECT_NEAR(output.linear.y, 0.2, 1e-6);
  EXPECT_LE(std::hypot(output.linear.x, output.linear.y), 0.5 + 1e-12);
  // Yaw FF uses the reduced path speed (1 rad / 20 m), not the original 0.5 m/s.
  EXPECT_NEAR(output.angular.z, 0.05 * std::sqrt(0.21), 1e-6);
}

TEST_F(CommandContractTest, infeasibleFeedbackStillProducesBoundedCorrection)
{
  controller_.setPlan(makeStraightPath({0.0, 20.0}));
  geometry_msgs::msg::Twist output;
  for (int cycle = 0; cycle < 30; ++cycle) {
    const auto previous = output;
    output = step(makeRobotPoseAt(0.0, -0.5)).twist;
    EXPECT_LE(std::abs(output.linear.y - previous.linear.y), limits_.max_ay * 0.1 + 1e-12);
    EXPECT_LE(std::hypot(output.linear.x, output.linear.y), limits_.max_xy_speed + 1e-12);
  }
  EXPECT_NEAR(output.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(output.linear.y, 0.2, 1e-12);
}


TEST_F(CommandContractTest, curvedTranslationKeepsAccelerationBoundsAndMakesProgress)
{
  speed_.nominal_tracking_speed = speed_.max_vx = speed_.max_xy_speed = 0.8;
  speed_.max_vy = 0.4;
  speed_.max_ax = speed_.max_ay = 1.0;
  speed_.a_ref_up = 0.8;
  speed_.a_ref_down = speed_.stopping_deceleration = 0.4;
  limits_.max_vx = limits_.max_xy_speed = 0.8;
  limits_.max_vy = 0.4;
  limits_.max_ax = limits_.max_ay = 1.0;
  limits_.max_decel_x = limits_.max_decel_y = 0.6;
  initialize();
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int index = 0; index <= 240; ++index) {
    const double x = index * 0.025;
    const double y = x >= 1.0 && x <= 5.0 ?
      0.25 * (1.0 - std::cos(3.14159265358979323846 * (x - 1.0))) : 0.0;
    path.poses.push_back(makeRobotPoseAt(x, y));
  }
  controller_.setPlan(path);
  auto pose = makeRobotPoseAt(0.0, 0.0);
  geometry_msgs::msg::Twist previous;
  constexpr double dt = 1.0 / 15.0;
  double max_distance = 0.0;
  for (int cycle = 0; cycle < 600 && pose.pose.position.x < 5.5; ++cycle) {
    const auto command = step(pose, previous, dt).twist;
    ASSERT_TRUE(std::isfinite(command.linear.x));
    ASSERT_TRUE(std::isfinite(command.linear.y));
    EXPECT_LE(std::abs(command.linear.x - previous.linear.x), dt + 1e-6);
    EXPECT_LE(std::abs(command.linear.y - previous.linear.y), dt + 1e-6);
    EXPECT_LE(std::hypot(command.linear.x, command.linear.y), 0.8 + 1e-6);
    // Ideal command-following plant: isolates controller/limiter behavior.
    pose.pose.position.x += command.linear.x * dt;
    pose.pose.position.y += command.linear.y * dt;
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < path.poses.size(); ++index) {
      const auto & start = path.poses[index - 1u].pose.position;
      const auto & end = path.poses[index].pose.position;
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double u = std::clamp(
        ((pose.pose.position.x - start.x) * dx +
        (pose.pose.position.y - start.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
      nearest = std::min(nearest, std::hypot(
        pose.pose.position.x - start.x - u * dx,
        pose.pose.position.y - start.y - u * dy));
    }
    max_distance = std::max(max_distance, nearest);
    previous = command;
  }
  EXPECT_GT(pose.pose.position.x, 5.5);
  EXPECT_LT(max_distance, 0.08);
  RecordProperty("max_path_distance_m", std::to_string(max_distance));
}

TEST(Lookahead, lookaheadKeepsSpeedTimeClamp)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setLookaheadParameters(
    controller, 0.5, 0.3, 0.9);

  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(0.0), 0.3);
  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(0.8), 0.4);
  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(3.0), 0.9);
}

TEST(PathValidation, nonFinitePathClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));

  auto invalid = makeStraightPath({0.0, 0.3});
  invalid.poses.back().pose.position.x =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(controller.setPlan(invalid), nav2_core::InvalidPath);
  EXPECT_FALSE(PidTrackingControllerTestAccess::hasPreparedPath(controller));
}

TEST(PathValidation, emptyPathClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));

  EXPECT_THROW(controller.setPlan(nav_msgs::msg::Path()), nav2_core::InvalidPath);
  EXPECT_FALSE(PidTrackingControllerTestAccess::hasPreparedPath(controller));
}

TEST(PathValidation, zeroNormQuaternionClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));

  auto invalid = makeStraightPath({0.0, 0.3});
  auto & quaternion = invalid.poses.back().pose.orientation;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;
  quaternion.w = 0.0;

  EXPECT_THROW(controller.setPlan(invalid), nav2_core::InvalidPath);
  EXPECT_FALSE(PidTrackingControllerTestAccess::hasPreparedPath(controller));
}

TEST_F(CommandContractTest, lateralOffsetCommandsTranslationWithoutYawCorrection)
{
  const auto pose = makeRobotPoseAt(0.0, 0.1);
  step(pose);
  const auto command = step(pose);
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.linear.y, 0.0);
  EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
  EXPECT_EQ(command.header.frame_id, "base_link");
}

TEST_F(CommandContractTest, bodyDirectionIsIndependentOfTargetHeading)
{
  for (const double yaw : {M_PI_2, M_PI}) {
    auto path = makeStraightPath({0.0, 1.0, 2.0});
    for (auto & pose : path.poses) {
      pose.pose.orientation = makeRobotPose(yaw).pose.orientation;
    }
    initialize();
    controller_.setPlan(path);
    step(makeRobotPose(yaw));
    const auto command = step(makeRobotPose(yaw));
    EXPECT_NEAR(command.twist.linear.x, 0.02 * std::cos(yaw), 1e-12);
    EXPECT_NEAR(command.twist.linear.y, -0.02 * std::sin(yaw), 1e-12);
    EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
  }
}

TEST_F(CommandContractTest, realDtAndCommandHistoryDriveSlew)
{
  step();
  const auto first = step();
  EXPECT_NEAR(first.twist.linear.x, 0.02, 1e-12);
  const auto second = step(makeRobotPose(), geometry_msgs::msg::Twist(), 0.2);
  EXPECT_NEAR(second.twist.linear.x, 0.06, 1e-12);
  // Measured velocity is still zero: a normal cycle must not reseed from it.
  EXPECT_GT(second.twist.linear.x, first.twist.linear.x);
}

TEST_F(CommandContractTest, firstAndDiscontinuousCyclesProjectMeasuredVelocity)
{
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.3;
  measured.linear.y = -0.4;
  measured.angular.z = 0.9;
  const auto initial = step(makeRobotPose(), measured);
  EXPECT_LE(std::hypot(initial.twist.linear.x, initial.twist.linear.y), 0.2 + 1e-12);
  EXPECT_NEAR(initial.twist.angular.z, 0.3, 1e-12);
  for (const double dt : {0.0, -0.1, 1.0}) {
    measured.linear.x = 0.04;
    measured.linear.y = 0.03;
    measured.angular.z = 0.02;
    const auto seed = step(makeRobotPose(), measured, dt);
    EXPECT_EQ(seed.twist, measured);
  }
}

TEST_F(CommandContractTest, lateralMeasuredSpeedSeedsPathReference)
{
  auto path = makeStraightPath({0.0, 1.0, 2.0});
  for (auto & pose : path.poses) {
    pose.pose.orientation = makeRobotPose(M_PI_2).pose.orientation;
  }
  controller_.setPlan(path);
  geometry_msgs::msg::Twist measured;
  measured.linear.y = -0.08;
  step(makeRobotPose(M_PI_2), measured);
  const auto command = step(makeRobotPose(M_PI_2), measured);
  EXPECT_NEAR(command.twist.linear.y, -0.1, 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
}

TEST_F(CommandContractTest, speedLimitIsImmediateAndInvalidUpdatesPreserveLastLimit)
{
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.2;
  step(makeRobotPose(), measured);
  controller_.setSpeedLimit(0.03, false);
  EXPECT_NEAR(step().twist.linear.x, 0.03, 1e-12);
  for (const double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()})
  {
    controller_.setSpeedLimit(invalid, false);
    EXPECT_LE(step().twist.linear.x, 0.03 + 1e-12);
  }
  controller_.setSpeedLimit(101.0, true);
  EXPECT_LE(step().twist.linear.x, 0.03 + 1e-12);
  controller_.setSpeedLimit(0.0, true);  // Nav2 zero sentinel releases, it is not stop.
  const auto released = step();
  EXPECT_GT(released.twist.linear.x, 0.03);
  EXPECT_LE(released.twist.linear.x, 0.05 + 1e-12);
}

TEST_F(CommandContractTest, percentageUsesMaximumXyInsteadOfNominalReference)
{
  speed_.nominal_tracking_speed = 0.2;
  speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.8;
  limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.8;
  initialize();
  controller_.setPlan(makeStraightPath({0.0, 1.0, 2.0}));
  controller_.setSpeedLimit(25.0, true);
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.5;
  EXPECT_NEAR(step(makeRobotPose(), measured).twist.linear.x, 0.2, 1e-12);
  EXPECT_NEAR(step(makeRobotPose(), measured).twist.linear.x, 0.2, 1e-12);
}

TEST_F(CommandContractTest, externalCapAlsoScalesTheSameYawFeedforward)
{
  auto path = makeStraightPath({0.0, 1.0, 2.0});
  path.poses[1].pose.orientation = makeRobotPose(0.5).pose.orientation;
  path.poses[2].pose.orientation = makeRobotPose(1.0).pose.orientation;
  controller_.setPlan(path);
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.2;
  step(makeRobotPose(), measured);
  controller_.setSpeedLimit(0.04, false);
  const auto command = step();
  EXPECT_NEAR(command.twist.linear.x, 0.04, 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.5 * command.twist.linear.x, 1e-12);
}

TEST_F(CommandContractTest, conflictingPathFramesRejectAndInvalidateOldPlan)
{
  auto path = makeStraightPath({0.0, 1.0});
  path.poses.back().header.frame_id = "odom";
  EXPECT_THROW(controller_.setPlan(path), nav2_core::InvalidPath);
  EXPECT_THROW(step(), nav2_core::InvalidPath);
  path.header.frame_id.clear();
  EXPECT_THROW(controller_.setPlan(path), nav2_core::InvalidPath);
}

TEST_F(CommandContractTest, transformsPositionAndYawIntoPathFrame)
{
  auto tf = std::make_shared<tf2_ros::Buffer>(clock_);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.rotation = makeRobotPose(0.5 * std::acos(-1.0)).pose.orientation;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  PidTrackingControllerTestAccess::setTf(controller_, tf);
  // R(+90 deg) * (-2, 1) + (1, 2) = (0, 0), with map yaw zero.
  auto pose = makeRobotPoseAt(-2.0, 1.0, -0.5 * std::acos(-1.0));
  pose.header.frame_id = "odom";
  pose.header.stamp.sec = 1;
  step(pose);
  const auto command = step(pose);
  EXPECT_EQ(command.header.frame_id, "base_link");
  EXPECT_NEAR(command.twist.linear.x, 0.02, 1e-12);
  EXPECT_NEAR(command.twist.linear.y, 0.0, 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
}

TEST_F(CommandContractTest, missingPoseFrameOrTransformThrowsControllerTfError)
{
  auto pose = makeRobotPose();
  pose.header.frame_id.clear();
  EXPECT_THROW(step(pose), nav2_core::ControllerTFError);
  pose.header.frame_id = "odom";
  EXPECT_THROW(step(pose), nav2_core::ControllerTFError);
  PidTrackingControllerTestAccess::setTf(
    controller_, std::make_shared<tf2_ros::Buffer>(clock_));
  EXPECT_THROW(step(pose), nav2_core::ControllerTFError);
}

TEST_F(CommandContractTest, taskResetReseedsFromMeasuredVelocityAndPreservesSpeedLimit)
{
  step();
  step();
  ASSERT_NEAR(step().twist.linear.x, 0.04, 1e-12);
  controller_.setSpeedLimit(0.03, false);
  controller_.reset();
  controller_.setPlan(makeStraightPath({0.0, 1.0, 3.0}));
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.01;
  measured.linear.y = -0.01;
  measured.angular.z = 0.04;
  const auto command = step(makeRobotPose(), measured);
  EXPECT_NEAR(command.twist.linear.x, 0.01, 1e-12);
  EXPECT_NEAR(command.twist.linear.y, -0.01, 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.04, 1e-12);
  for (int index = 0; index < 5; ++index) {
    const auto next = step();
    EXPECT_LE(std::hypot(next.twist.linear.x, next.twist.linear.y), 0.03 + 1e-12);
  }
}

TEST_F(CommandContractTest, taskResetRequiresANewPlan)
{
  step();
  controller_.reset();
  EXPECT_THROW(step(), nav2_core::InvalidPath);
}

TEST_F(CommandContractTest, sameGoalReplanChangesGeometryWithoutResettingSlew)
{
  step();
  step();
  EXPECT_NEAR(step().twist.linear.x, 0.04, 1e-12);
  auto replanned = makeStraightPath({0.0, 1.0, 3.0});
  replanned.poses[1].pose.orientation = makeRobotPose(0.1).pose.orientation;
  controller_.setPlan(replanned);
  const auto command = step();
  EXPECT_NEAR(command.twist.linear.x, 0.06, 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.1 * command.twist.linear.x, 1e-12);
}

TEST_F(CommandContractTest, staleReplanPrefixNeverPullsRobotBack)
{
  for (const bool initial_rotation : {false, true}) {
    SCOPED_TRACE(initial_rotation);
    controller_.reset();
    controller_.setPlan(makeStraightPath({4.0, 2.0, 0.5}));
    geometry_msgs::msg::Twist measured;
    measured.linear.x = -0.15;
    step(makeRobotPoseAt(3.5, 0.0), measured);
    step(makeRobotPoseAt(2.05, 0.0), measured);

    auto path = makeStraightPath({3.5});
    if (initial_rotation) {
      path.poses.front().pose.orientation = makeRobotPose(0.2).pose.orientation;
      path.poses.push_back(makeRobotPoseAt(3.5, 0.0));
    }
    for (int i = 1; i <= 60; ++i) {
      path.poses.push_back(makeRobotPoseAt(3.5 - 0.05 * i, 0.0));
    }
    auto tf = std::make_shared<tf2_ros::Buffer>(clock_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.translation.x = 1.0;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf->setTransform(transform, "test", true));
    PidTrackingControllerTestAccess::setTf(controller_, tf);
    controller_.setPlan(path);
    // The first control pose is newer than both the planning and last control poses.
    for (int i = 0; i < 30; ++i) {
      if (i > 0 && i % 5 == 0) {
        controller_.setPlan(path);
      }
      auto pose = makeRobotPoseAt(0.925 - 0.01 * i, 0.0);
      pose.header.frame_id = "odom";
      const auto command = step(pose, measured);
      EXPECT_LT(command.twist.linear.x, 0.0);
      EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
      measured = command.twist;
    }
  }
}

TEST_F(CommandContractTest, ambiguousHandoffNeverFallsBackToTrackingAndCanRetry)
{
  step();
  controller_.setPlan(makeDenseHairpinPath());
  for (int i = 0; i < 3; ++i) {
    EXPECT_THROW(step(makeRobotPoseAt(0.4, 0.049)), nav2_core::NoValidControl);
  }
  // A valid observation retries the same revision, rather than tracking from s=0.
  EXPECT_NO_THROW(step(makeRobotPoseAt(0.4, 0.02)));
  EXPECT_GT(step(makeRobotPoseAt(0.4, 0.02)).twist.linear.x, 0.0);
  // Once initialized, normal budget protection owns later observations.
  const auto nearby = step(makeRobotPoseAt(0.4, 0.08));
  EXPECT_GT(nearby.twist.linear.x, 0.0);
  EXPECT_LT(nearby.twist.linear.y, 0.0);
}

TEST_F(CommandContractTest, zeroReferenceDoesNotUsePreviewAsPositionFeedback)
{
  speed_.nominal_tracking_speed = 0.0;
  initialize();
  controller_.setPlan(makeStraightPath({0.0, 1.0, 2.0}));
  for (int index = 0; index < 10; ++index) {
    const auto command = step();
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  }
}

TEST_F(CommandContractTest, rotationUsesIndependentDeceleration)
{
  speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.8;
  limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.8;
  limits_.max_decel_x = limits_.max_decel_y = 0.6;
  for (const auto & moving : std::vector<geometry_msgs::msg::Twist>{
      [] {geometry_msgs::msg::Twist t; t.linear.x = 0.44; return t;}(),
      [] {geometry_msgs::msg::Twist t; t.linear.x = -0.44; return t;}(),
      [] {geometry_msgs::msg::Twist t; t.linear.y = 0.44; return t;}()})
  {
    initialize();
    controller_.setPlan(makeStraightPath({0.0, 1.0}));
    step(makeRobotPose(), moving);
    setRotationPath(true);
    const auto command = step(makeRobotPose(), moving, 1.0 / 15.0);
    EXPECT_NEAR(std::hypot(command.twist.linear.x, command.twist.linear.y), 0.40, 1e-9);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  }
}

TEST_F(CommandContractTest, rotationBrakesDiagonalWithoutChangingDirection)
{
  speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.8;
  limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.8;
  limits_.max_decel_x = limits_.max_decel_y = 0.6;
  initialize();
  controller_.setPlan(makeStraightPath({0.0, 1.0}));
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.44;
  measured.linear.y = -0.33;
  step(makeRobotPose(), measured);
  setRotationPath(true);
  const auto command = step(makeRobotPose(), measured, 1.0 / 15.0);
  EXPECT_NEAR(command.twist.linear.x, 0.40, 1e-9);
  EXPECT_NEAR(command.twist.linear.y, -0.30, 1e-9);
}

TEST_F(CommandContractTest, rotationBrakesBeforeTurningAndWaitsForMeasuredStop)
{
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.2;
  step(makeRobotPose(), moving);
  setRotationPath(true);
  // Even after the command reaches zero, moving odometry must prevent rotation.
  for (int index = 0; index < 15; ++index) {
    const auto command = step(makeRobotPose(), moving);
    EXPECT_NEAR(command.twist.linear.x, std::max(0.0, 0.18 - 0.02 * index), 1e-12);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  }
  EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
  for (int index = 0; index < 4; ++index) {
    step();
  }
  EXPECT_GT(step().twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, rotationAccelerationOverrideRequiresConfirmedStop)
{
  limits_.rotation_max_alpha = 0.8;
  initialize();
  controller_.setPlan(makeStraightPath({0.0, 1.0}));
  geometry_msgs::msg::Twist moving;
  moving.angular.z = 0.3;
  step(makeRobotPose(), moving);
  setRotationPath(true);
  for (int index = 0; index < 8; ++index) {
    EXPECT_NEAR(step(makeRobotPose(), moving).twist.angular.z,
      std::max(0.0, 0.25 - 0.05 * index), 1e-12);
  }
  for (int index = 0; index < 3; ++index) {
    EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
  }
  EXPECT_NEAR(step().twist.angular.z, 0.08, 1e-12);
  EXPECT_NEAR(step().twist.angular.z, 0.16, 1e-12);
}

TEST_F(CommandContractTest, rotationAccelerationOverrideDoesNotAffectTranslationYaw)
{
  limits_.rotation_max_alpha = 0.8;
  initialize();
  controller_.setPlan(makeStraightPath({0.0, 1.0, 3.0}));
  const auto pose = makeRobotPose(0.2);
  step(pose);
  const auto output = step(pose).twist;
  EXPECT_GT(output.linear.x, 0.0);
  EXPECT_NEAR(output.angular.z, -0.05, 1e-12);
}

TEST_F(CommandContractTest, entryBrakesExistingAngularMotionWithoutHardStopping)
{
  geometry_msgs::msg::Twist moving;
  moving.angular.z = 0.3;
  step(makeRobotPose(), moving);
  setRotationPath(true);
  for (int index = 0; index < 8; ++index) {
    const auto command = step(makeRobotPose(), moving);
    EXPECT_NEAR(command.twist.angular.z, std::max(0.0, 0.25 - 0.05 * index), 1e-12);
  }
}

TEST_F(CommandContractTest, brakingConfirmationSurvivesReplansButNotControlGaps)
{
  setRotationPath(true);
  step();
  step();
  setRotationPath(true);
  EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
  EXPECT_GT(step().twist.angular.z, 0.0);
  // Stationary samples after gaps or nonpositive dt cannot reuse the old admission.
  for (const double dt : {2.0, 0.0, -0.1}) {
    EXPECT_DOUBLE_EQ(
      step(makeRobotPose(), geometry_msgs::msg::Twist(), dt).twist.angular.z, 0.0);
    EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
    EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
    EXPECT_GT(step().twist.angular.z, 0.0);
  }
}

TEST_F(CommandContractTest, continuousReplansCannotHideBrakingTimeout)
{
  geometry_msgs::msg::Twist moving;
  moving.linear.y = 0.1;
  setRotationPath(true);
  step(makeRobotPose(), moving);
  bool timed_out = false;
  for (int index = 0; index < 85 && !timed_out; ++index) {
    setRotationPath(true);
    try {
      EXPECT_DOUBLE_EQ(step(makeRobotPose(), moving).twist.angular.z, 0.0);
    } catch (const nav2_core::NoValidControl &) {
      timed_out = true;
    }
  }
  EXPECT_TRUE(timed_out);
  // Error recovery seeds from measured velocity and cannot reuse stop evidence.
  EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(step().twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, rotationRunAcknowledgesFinalYawAndResumesTranslation)
{
  setRotationPath(true);
  const auto aligned = makeRobotPose(0.2);
  // 0.2 s stop admission, then independent 0.3 s rotation completion.
  for (int index = 0; index < 7; ++index) {
    const auto command = step(aligned);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
  }
  EXPECT_GT(step(aligned).twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, nearStartHandoffRotatesBeforeTranslatingDespitePositiveProjection)
{
  setRotationPath(true);
  const auto unaligned = makeRobotPoseAt(0.005, 0.001);
  geometry_msgs::msg::Twist measured;
  for (int index = 0; index < 12; ++index) {
    const auto command = step(unaligned, measured);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
    measured = command.twist;
  }
  EXPECT_GT(measured.angular.z, 0.0);
  const auto aligned = makeRobotPoseAt(0.005, 0.001, 0.2);
  measured = geometry_msgs::msg::Twist();
  // Existing completion checks and angular rate limiting govern release.
  geometry_msgs::msg::TwistStamped command;
  for (int index = 0; index < 15; ++index) {
    command = step(aligned, measured);
    measured = command.twist;
  }
  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, finalRotationRunTransitionsToTerminalPositionFeedback)
{
  setRotationPath(false);
  const auto aligned = makeRobotPoseAt(0.0, 0.1, 0.2);
  for (int index = 0; index < 7; ++index) {
    EXPECT_DOUBLE_EQ(step(aligned).twist.linear.y, 0.0);
  }
  const auto terminal = step(aligned);
  EXPECT_LT(terminal.twist.linear.y, 0.0);
  EXPECT_NEAR(terminal.twist.angular.z, 0.0, 1e-12);
}

TEST_F(CommandContractTest, rotationRequiresAllMeasuredConditionsAndContinuousSettle)
{
  setRotationPath(true);
  const auto aligned = makeRobotPose(0.2);
  for (int index = 0; index < 3; ++index) {
    step(aligned);  // Complete stop admission before exercising yaw completion.
  }
  step(aligned);
  step(aligned);
  geometry_msgs::msg::Twist drifting;
  drifting.linear.y = 0.02;
  step(aligned, drifting);  // Break XY condition.
  step(aligned);
  step(aligned);
  drifting.linear.y = 0.0;
  drifting.angular.z = 0.02;
  step(aligned, drifting);  // Break omega condition.
  step(aligned);
  step(makeRobotPose(0.0));  // Break yaw condition.
  for (int index = 0; index < 4; ++index) {
    EXPECT_DOUBLE_EQ(step(aligned).twist.linear.x, 0.0);
  }
  EXPECT_GT(step(aligned).twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, gapAndSamePlanResetRotationSettle)
{
  for (const bool replan : {false, true}) {
    initialize();
    setRotationPath(true);
    const auto aligned = makeRobotPose(0.2);
    for (int index = 0; index < 3; ++index) {
      step(aligned);
    }
    step(aligned);
    step(aligned);
    step(aligned);
    if (replan) {
      setRotationPath(true);
      step(aligned);  // The first observation in a new plan counts zero.
    } else {
      step(aligned, geometry_msgs::msg::Twist(), 2.0);  // Do not count a gap as settled time.
    }
    for (int index = 0; index < 6; ++index) {
      EXPECT_DOUBLE_EQ(step(aligned).twist.linear.x, 0.0);
    }
    EXPECT_GT(step(aligned).twist.linear.x, 0.0);
  }
}

TEST_F(CommandContractTest, singlePoseTerminalKeepsPositionAndYawFeedback)
{
  controller_.setPlan(makeStraightPath({0.0}));
  const auto pose = makeRobotPoseAt(-0.1, 0.1, 0.1);
  step(pose);
  const auto command = step(pose);
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.linear.y, 0.0);
  EXPECT_LT(command.twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, nearbyFutureBranchDoesNotTakeOverTracking)
{
  controller_.setPlan(makeDenseHairpinPath());
  for (int index = 0; index <= 8; ++index) {
    step(makeRobotPoseAt(0.05 * static_cast<double>(index), 0.0));
  }
  const auto command = step(makeRobotPoseAt(0.40, 0.08));
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.linear.y, 0.0);  // Correct toward current y=0, not future y=0.1.
  EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
}

TEST_F(CommandContractTest, invalidInputRejectsInsteadOfFallingBackAndRecoveryReseeds)
{
  step();
  step();
  auto pose = makeRobotPose();
  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(step(pose), nav2_core::NoValidControl);
  EXPECT_DOUBLE_EQ(step().twist.linear.x, 0.0);
  pose = makeRobotPose();
  pose.pose.orientation.w = 0.0;
  EXPECT_THROW(step(pose), nav2_core::NoValidControl);
  geometry_msgs::msg::Twist measured;
  measured.linear.y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(step(makeRobotPose(), measured), nav2_core::NoValidControl);
  EXPECT_THROW(controller_.setPlan(nav_msgs::msg::Path()), nav2_core::InvalidPath);
  EXPECT_THROW(step(), nav2_core::InvalidPath);
  controller_.setPlan(makeStraightPath({0.0, 1.0, 2.0}));
  EXPECT_DOUBLE_EQ(step().twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, failedActiveCalculationCannotAcknowledgeRotation)
{
  setRotationPath(true);
  const auto aligned = makeRobotPose(0.2);
  for (int index = 0; index < 3; ++index) {
    step(aligned);
  }
  step(aligned);
  step(aligned);
  step(aligned);
  auto invalid = aligned;
  invalid.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(step(invalid), nav2_core::NoValidControl);
  for (int index = 0; index < 7; ++index) {
    EXPECT_DOUBLE_EQ(step(aligned).twist.linear.x, 0.0);
  }
  EXPECT_GT(step(aligned).twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, closedLoopTranslationAndFinalRotationConvergeWithinEnvelope)
{
  auto path = makeStraightPath({0.0, 1.0, 2.0});
  for (auto & point : path.poses) {
    point.pose.orientation = makeRobotPose(M_PI_2).pose.orientation;
  }
  path.poses.push_back(makeRobotPoseAt(2.0, 0.0, 0.0));
  controller_.setPlan(path);
  double x = -0.1;
  double y = 0.2;
  double yaw = M_PI_2;
  geometry_msgs::msg::Twist measured;
  // Ideal body-velocity plant: protects controller composition, not real robot dynamics.
  for (int index = 0; index < 600; ++index) {
    const auto command = step(makeRobotPoseAt(x, y, yaw), measured).twist;
    EXPECT_LE(std::hypot(command.linear.x, command.linear.y), 0.2 + 1e-12);
    EXPECT_LE(std::abs(command.angular.z), 0.3 + 1e-12);
    EXPECT_LE(std::abs(command.linear.x - measured.linear.x), 0.02 + 1e-12);
    EXPECT_LE(std::abs(command.linear.y - measured.linear.y), 0.02 + 1e-12);
    EXPECT_LE(std::abs(command.angular.z - measured.angular.z), 0.05 + 1e-12);
    x += 0.1 * (std::cos(yaw) * command.linear.x - std::sin(yaw) * command.linear.y);
    y += 0.1 * (std::sin(yaw) * command.linear.x + std::cos(yaw) * command.linear.y);
    yaw += 0.1 * command.angular.z;
    measured = command;
  }
  EXPECT_NEAR(x, 2.0, 0.01);
  EXPECT_NEAR(y, 0.0, 0.01);
  EXPECT_NEAR(yaw, 0.0, 0.01);
  EXPECT_LT(std::hypot(measured.linear.x, measured.linear.y), 0.01);
  EXPECT_LT(std::abs(measured.angular.z), 0.01);
}

TEST_F(CommandContractTest, closedLoopTranslationCornerReachesOutgoingEndpoint)
{
  auto path = makeStraightPath({0.0, 1.0});
  path.poses.push_back(makeRobotPoseAt(1.0, 1.0));
  controller_.setPlan(path);
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  geometry_msgs::msg::Twist measured;
  bool reached_outgoing_segment = false;
  // Pose advances only from the real controller output through an ideal velocity plant.
  for (int cycle = 0; cycle < 450; ++cycle) {
    const auto command = step(makeRobotPoseAt(x, y, yaw), measured).twist;
    ASSERT_LE(std::hypot(command.linear.x, command.linear.y), 0.2 + 1e-12);
    ASSERT_LE(std::abs(command.linear.x - measured.linear.x), 0.02 + 1e-12);
    ASSERT_LE(std::abs(command.linear.y - measured.linear.y), 0.02 + 1e-12);
    x += 0.1 * (std::cos(yaw) * command.linear.x - std::sin(yaw) * command.linear.y);
    y += 0.1 * (std::sin(yaw) * command.linear.x + std::cos(yaw) * command.linear.y);
    yaw += 0.1 * command.angular.z;
    measured = command;
    reached_outgoing_segment = reached_outgoing_segment || (y > 0.5 && std::abs(x - 1.0) < 0.1);
  }
  EXPECT_TRUE(reached_outgoing_segment);
  EXPECT_NEAR(x, 1.0, 0.01);
  EXPECT_NEAR(y, 1.0, 0.01);
  EXPECT_NEAR(yaw, 0.0, 0.01);
  EXPECT_LT(std::hypot(measured.linear.x, measured.linear.y), 0.01);
}

TEST_F(CommandContractTest, stopsNearEndpointWithPoseAndDriveDelay)
{
  // The plant has actuator lag and the controller observes poses two cycles old.
  // Exercise the actual tracker, planner, feedback law and command limiter together.
  for (const double deceleration : {0.2, 0.6}) {
  for (const double direction : {1.0, -1.0}) {
    for (const bool lateral : {false, true}) {
      SCOPED_TRACE(direction);
      SCOPED_TRACE(lateral);
      speed_.nominal_tracking_speed = 0.8;
      speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.8;
      limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.8;
      limits_.max_decel_x = limits_.max_decel_y = deceleration;
      initialize();
      auto path = makeStraightPath({0.0, direction * 1.5});
      if (lateral) {
        for (auto & pose : path.poses) {
          std::swap(pose.pose.position.x, pose.pose.position.y);
        }
      }
      controller_.setPlan(path);
      constexpr double dt = 1.0 / 15.0;
      constexpr double drive_time_constant = 0.25;
      std::vector<double> positions(3, 0.0);
      double velocity = 0.0;
      double max_overshoot = 0.0;
      for (int cycle = 0; cycle < 450; ++cycle) {
        const double observed_position = positions[positions.size() - 3];
        auto pose = makeRobotPoseAt(
          lateral ? 0.0 : observed_position, lateral ? observed_position : 0.0);
        geometry_msgs::msg::Twist measured;
        if (lateral) {
          measured.linear.y = velocity;
        } else {
          measured.linear.x = velocity;
        }
        const auto output = step(pose, measured, dt);
        const double next_velocity = velocity +
          ((lateral ? output.twist.linear.y : output.twist.linear.x) - velocity) *
          (1.0 - std::exp(-dt / drive_time_constant));
        positions.push_back(positions.back() + 0.5 * (velocity + next_velocity) * dt);
        velocity = next_velocity;
        max_overshoot = std::max(max_overshoot, direction * positions.back() - 1.5);
      }
      EXPECT_LE(max_overshoot, 0.05);
      EXPECT_NEAR(positions.back(), direction * 1.5, 0.05);
      EXPECT_LT(std::abs(velocity), 0.05);
    }
  }
  }
}

TEST_F(CommandContractTest, delayedDriveWithDeadbandReachesTerminalRotation)
{
  speed_.nominal_tracking_speed = 0.8;
  speed_.max_vx = speed_.max_vy = speed_.max_xy_speed = 0.8;
  limits_.max_vx = limits_.max_vy = limits_.max_xy_speed = 0.8;
  limits_.max_decel_x = limits_.max_decel_y = 0.2;
  initialize();
  auto path = makeStraightPath({0.0, 1.5});
  path.poses.push_back(makeRobotPoseAt(1.5, 0.0, 0.4));
  controller_.setPlan(path);
  constexpr double dt = 1.0 / 15.0;
  std::vector<geometry_msgs::msg::PoseStamped> history(3, makeRobotPose());
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double world_vx = 0.0;
  double world_vy = 0.0;
  double omega = 0.0;
  double max_overshoot = 0.0;
  const auto deadband = [](double value) {return std::abs(value) < 0.01 ? 0.0 : value;};
  for (int cycle = 0; cycle < 450; ++cycle) {
    geometry_msgs::msg::Twist measured;
    measured.linear.x = std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
    measured.linear.y = -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
    measured.angular.z = omega;
    const auto output = step(history[history.size() - 3], measured, dt).twist;
    const double body_vx = deadband(output.linear.x);
    const double body_vy = deadband(output.linear.y);
    const double target_vx = std::cos(yaw) * body_vx - std::sin(yaw) * body_vy;
    const double target_vy = std::sin(yaw) * body_vx + std::cos(yaw) * body_vy;
    const double alpha = 1.0 - std::exp(-dt / 0.25);
    const double next_vx = world_vx + alpha * (target_vx - world_vx);
    const double next_vy = world_vy + alpha * (target_vy - world_vy);
    x += 0.5 * (world_vx + next_vx) * dt;
    y += 0.5 * (world_vy + next_vy) * dt;
    world_vx = next_vx;
    world_vy = next_vy;
    omega = output.angular.z;
    yaw += omega * dt;
    history.push_back(makeRobotPoseAt(x, y, yaw));
    max_overshoot = std::max(max_overshoot, x - 1.5);
  }
  EXPECT_LE(max_overshoot, 0.05);
  EXPECT_NEAR(x, 1.5, 0.05);
  EXPECT_NEAR(y, 0.0, 0.05);
  EXPECT_NEAR(yaw, 0.4, 0.02);
  EXPECT_LT(std::hypot(world_vx, world_vy), 0.01);
  EXPECT_LT(std::abs(omega), 0.01);
}

TEST_F(CommandContractTest, axisDeadbandStillAllowsIntermediateRotationAndContinuation)
{
  // Reproduce a drive that discards each small body-axis command independently.
  // Retain the original reproduction and also model the confirmed real-drive thresholds.
  for (const bool real_drive_thresholds : {false, true}) {
    SCOPED_TRACE(real_drive_thresholds);
    const double linear_deadband = real_drive_thresholds ? 0.01 : 0.02;
    const double angular_deadband = real_drive_thresholds ? 0.03 : 0.01;
    // Use the production yaw tolerance for the real drive, retaining stricter stop checks.
    const double yaw_tolerance = real_drive_thresholds ? 0.05 : 0.01;
    for (const double direction : {0.0, M_PI_2, M_PI_4, M_PI, -M_PI_2, -M_PI_4}) {
      SCOPED_TRACE(direction);
      speed_.nominal_tracking_speed = 0.8;
      speed_.max_vx = speed_.max_xy_speed = 0.8;
      speed_.max_vy = 0.4;
      speed_.max_ax = speed_.max_ay = speed_.a_ref_up = 0.8;
      speed_.a_ref_down = speed_.stopping_deceleration = 0.4;
      limits_.max_vx = limits_.max_xy_speed = 0.8;
      limits_.max_vy = 0.4;
      limits_.max_ax = limits_.max_ay = 0.8;
      limits_.max_decel_x = limits_.max_decel_y = 0.6;
      PidTrackingControllerTestAccess::initializeHolonomic(
        controller_, speed_, limits_, {yaw_tolerance, 0.01, 0.01, 0.3});
      const double tx = std::cos(direction);
      const double ty = std::sin(direction);
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.poses = {makeRobotPoseAt(0.0, 0.0), makeRobotPoseAt(0.4 * tx, 0.4 * ty),
        makeRobotPoseAt(0.4 * tx, 0.4 * ty, 0.4), makeRobotPoseAt(tx, ty, 0.4)};
      controller_.setPlan(path);
      constexpr double dt = 1.0 / 15.0;
      double x = -0.012 * ty;
      double y = 0.012 * tx;
      double yaw = 0.0;
      double world_vx = 0.0;
      double world_vy = 0.0;
      double omega = 0.0;
      bool rotation_started = false;
      double max_boundary_overshoot = 0.0;
      double max_goal_overshoot = 0.0;
      std::vector<geometry_msgs::msg::PoseStamped> history(3, makeRobotPoseAt(x, y));
      for (int cycle = 0; cycle < 600; ++cycle) {
        geometry_msgs::msg::Twist measured;
        measured.linear.x = std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
        measured.linear.y = -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
        measured.angular.z = omega;
        const auto output = step(history[history.size() - 3], measured, dt).twist;
        if (!rotation_started && std::abs(output.angular.z) >= angular_deadband) {
          rotation_started = true;
          EXPECT_LT(std::hypot(world_vx, world_vy), 0.01);
          EXPECT_NEAR(x, 0.4 * tx, 0.05);
          EXPECT_NEAR(y, 0.4 * ty, 0.05);
        }
        const double body_vx = std::abs(output.linear.x) < linear_deadband ? 0.0 : output.linear.x;
        const double body_vy = std::abs(output.linear.y) < linear_deadband ? 0.0 : output.linear.y;
        const double target_vx = std::cos(yaw) * body_vx - std::sin(yaw) * body_vy;
        const double target_vy = std::sin(yaw) * body_vx + std::cos(yaw) * body_vy;
        const double alpha = 1.0 - std::exp(-dt / 0.25);
        const double next_vx = world_vx + alpha * (target_vx - world_vx);
        const double next_vy = world_vy + alpha * (target_vy - world_vy);
        x += 0.5 * (world_vx + next_vx) * dt;
        y += 0.5 * (world_vy + next_vy) * dt;
        world_vx = next_vx;
        world_vy = next_vy;
        omega = std::abs(output.angular.z) < angular_deadband ? 0.0 : output.angular.z;
        yaw += omega * dt;
        if (!rotation_started) {
          max_boundary_overshoot = std::max(max_boundary_overshoot, x * tx + y * ty - 0.4);
        }
        max_goal_overshoot = std::max(max_goal_overshoot, x * tx + y * ty - 1.0);
        history.push_back(makeRobotPoseAt(x, y, yaw));
      }
      EXPECT_TRUE(rotation_started);
      EXPECT_LE(max_boundary_overshoot, 0.05);
      EXPECT_LE(max_goal_overshoot, 0.05);
      EXPECT_NEAR(x, tx, 0.05);
      EXPECT_NEAR(y, ty, 0.05);
      EXPECT_NEAR(yaw, 0.4, real_drive_thresholds ? 0.05 : 0.02);
      EXPECT_LT(std::hypot(world_vx, world_vy), 0.01);
      EXPECT_LT(std::abs(omega), 0.01);
    }
  }
}

}  // namespace
}  // namespace pid_tracking_controller
