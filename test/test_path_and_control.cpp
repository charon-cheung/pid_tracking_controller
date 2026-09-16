#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "controller_test_access.hpp"
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

}  // namespace
}  // namespace pid_tracking_controller
