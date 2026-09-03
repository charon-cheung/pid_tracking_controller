#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "controller_test_access.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "rclcpp/clock.hpp"

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
    PidTrackingControllerTestAccess::setClock(
      controller_, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
    PidTrackingControllerTestAccess::setMaximumVelocityIncrements(
      controller_, 0.1, 0.2);
    controller_.setPlan(makeStraightPath({0.0, 0.3, 0.6}));
  }

  nav2_costmap_2d::Costmap2D costmap_{100, 100, 0.05, -2.5, -2.5};
  PidTrackingController controller_;
};

TEST(PathValidation, nonFinitePathClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));

  auto invalid = makeStraightPath({0.0, 0.3});
  invalid.poses.back().pose.position.x =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(controller.setPlan(invalid), nav2_core::InvalidPath);
  EXPECT_TRUE(PidTrackingControllerTestAccess::plan(controller).empty());
}

TEST(PathValidation, emptyPathClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));

  EXPECT_THROW(controller.setPlan(nav_msgs::msg::Path()), nav2_core::InvalidPath);
  EXPECT_TRUE(PidTrackingControllerTestAccess::plan(controller).empty());
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
  EXPECT_TRUE(PidTrackingControllerTestAccess::plan(controller).empty());
}

TEST_F(CommandContractTest, commandUsesMeasuredVelocityAndKeepsLateralVelocityZero)
{
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.4;

  const auto command = controller_.computeVelocityCommands(
    makeRobotPose(), measured, nullptr);

  EXPECT_NEAR(command.twist.linear.x, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
  EXPECT_EQ(command.header.frame_id, "base_link");
  EXPECT_TRUE(std::isfinite(command.twist.linear.x));
  EXPECT_TRUE(std::isfinite(command.twist.angular.z));
}

TEST_F(CommandContractTest, largeInitialHeadingErrorRotatesBeforeTranslating)
{
  const auto command = controller_.computeVelocityCommands(
    makeRobotPose(M_PI_2), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_LT(command.twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, diagnosticsSeparateAlignmentFromTrackingSamples)
{
  controller_.computeVelocityCommands(
    makeRobotPose(M_PI_2), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_EQ(PidTrackingControllerTestAccess::alignmentSampleCount(controller_), 1u);
  EXPECT_EQ(PidTrackingControllerTestAccess::trackingSampleCount(controller_), 0u);

  controller_.computeVelocityCommands(
    makeRobotPose(), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_EQ(PidTrackingControllerTestAccess::alignmentSampleCount(controller_), 1u);
  EXPECT_EQ(PidTrackingControllerTestAccess::trackingSampleCount(controller_), 1u);
}

TEST_F(CommandContractTest, trackingLateralDiagnosticsExcludeAlignmentSamples)
{
  auto alignment_pose = makeRobotPose(M_PI_2);
  alignment_pose.pose.position.y = 0.1;
  controller_.computeVelocityCommands(
    alignment_pose, geometry_msgs::msg::Twist(), nullptr);

  const double alignment_target_yaw = std::atan2(-0.1, std::sqrt(0.08));
  auto tracking_pose = makeRobotPose(alignment_target_yaw);
  tracking_pose.pose.position.y = 0.02;
  controller_.computeVelocityCommands(
    tracking_pose, geometry_msgs::msg::Twist(), nullptr);

  const auto & tracking_samples =
    PidTrackingControllerTestAccess::trackingAbsoluteLateralErrorSamples(controller_);
  ASSERT_EQ(tracking_samples.size(), 1u);
  EXPECT_NEAR(tracking_samples.front(), 0.02, 1e-12);
  EXPECT_NEAR(
    PidTrackingControllerTestAccess::trackingSumSquaredLateralError(controller_),
    0.0004, 1e-12);
  EXPECT_NEAR(
    PidTrackingControllerTestAccess::trackingMaximumAbsoluteLateralError(controller_),
    0.02, 1e-12);
}

TEST_F(CommandContractTest, headingAlignmentDoesNotReenterDuringSameGoal)
{
  const auto aligning = controller_.computeVelocityCommands(
    makeRobotPose(M_PI_2), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_DOUBLE_EQ(aligning.twist.linear.x, 0.0);

  const auto tracking = controller_.computeVelocityCommands(
    makeRobotPose(), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(tracking.twist.linear.x, 0.0);

  controller_.setPlan(makeStraightPath({0.0, 0.2, 0.6}));

  const auto command = controller_.computeVelocityCommands(
    makeRobotPose(1.0), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, zeroCarrotCommandRemainsStoppedThroughRegularization)
{
  controller_.setPlan(makeStraightPath({0.0}));

  const auto command = controller_.computeVelocityCommands(
    makeRobotPose(), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, newGoalCanStartHeadingAlignmentAgain)
{
  controller_.computeVelocityCommands(
    makeRobotPose(), geometry_msgs::msg::Twist(), nullptr);
  controller_.setPlan(makeStraightPath({0.0, 0.3, 0.9}));

  const auto command = controller_.computeVelocityCommands(
    makeRobotPose(M_PI_2), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.angular.z, 0.0);
}

TEST_F(CommandContractTest, normalTrackingNeverCommandsReverseMotion)
{
  controller_.computeVelocityCommands(
    makeRobotPose(), geometry_msgs::msg::Twist(), nullptr);

  const auto command = controller_.computeVelocityCommands(
    makeRobotPose(M_PI), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
}

TEST_F(CommandContractTest, nearbyFutureBranchDoesNotTakeOverTracking)
{
  controller_.setPlan(makeDenseHairpinPath());
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.30;
  for (int index = 0; index <= 8; ++index) {
    controller_.computeVelocityCommands(
      makeRobotPoseAt(0.05 * static_cast<double>(index), 0.0), measured, nullptr);
  }

  const auto command = controller_.computeVelocityCommands(
    makeRobotPoseAt(0.40, 0.08), measured, nullptr);

  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  const auto & lateral_errors =
    PidTrackingControllerTestAccess::trackingAbsoluteLateralErrorSamples(controller_);
  ASSERT_FALSE(lateral_errors.empty());
  EXPECT_NEAR(lateral_errors.back(), 0.08, 1e-12);
}

TEST_F(CommandContractTest, nonFinitePoseThrowsNoValidControl)
{
  auto pose = makeRobotPose();
  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(
    controller_.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr),
    nav2_core::NoValidControl);
}

TEST_F(CommandContractTest, nonFiniteVelocityThrowsNoValidControl)
{
  geometry_msgs::msg::Twist measured;
  measured.linear.y = std::numeric_limits<double>::infinity();

  EXPECT_THROW(
    controller_.computeVelocityCommands(makeRobotPose(), measured, nullptr),
    nav2_core::NoValidControl);
}

}  // namespace
}  // namespace pid_tracking_controller
