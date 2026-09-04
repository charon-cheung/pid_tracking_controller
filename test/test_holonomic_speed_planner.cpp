#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "holonomic_speed_planner.hpp"
#include "holonomic_tracking_reference.hpp"
#include "path_progress_tracker.hpp"
#include "prepared_path.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct PathPose
{
  double x;
  double y;
  double yaw;
};

geometry_msgs::msg::Quaternion makeQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

nav_msgs::msg::Path makePath(std::initializer_list<PathPose> path_poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & path_pose : path_poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = makePoint(path_pose.x, path_pose.y);
    pose.pose.orientation = makeQuaternion(path_pose.yaw);
    path.poses.push_back(pose);
  }
  return path;
}

HolonomicSpeedPlannerConfig makeConfig()
{
  HolonomicSpeedPlannerConfig config;
  config.nominal_tracking_speed = 2.0;
  config.max_vx = 3.0;
  config.max_vy = 3.0;
  config.max_xy_speed = 3.0;
  config.max_omega = 2.0;
  config.max_ax = 4.0;
  config.max_ay = 4.0;
  config.a_ref_up = 4.0;
  config.a_ref_down = 4.0;
  config.yaw_feedback_reserve = 0.0;
  config.configured_min_rate_window = 0.0;
  return config;
}

HolonomicTrackingReference makeReference(
  const PreparedPathPtr & path,
  const geometry_msgs::msg::Point & robot_position,
  double lookahead_distance = 0.5)
{
  PathProgressTracker tracker;
  tracker.setPath(path);
  const auto progress = tracker.update(robot_position, lookahead_distance);
  return buildHolonomicTrackingReference(*path, progress, lookahead_distance);
}

HolonomicSpeedPlan planAt(
  HolonomicSpeedPlanner & planner,
  const PreparedPathPtr & path,
  const geometry_msgs::msg::Point & robot_position,
  double robot_yaw = 0.0,
  double dt = 1.0,
  double external_speed_limit = std::numeric_limits<double>::infinity())
{
  return planner.plan(
    makeReference(path, robot_position), robot_yaw, dt, external_speed_limit);
}

TEST(HolonomicSpeedPlanner, longStraightUsesNominalSpeed)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.8;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.8);

  const auto plan = planAt(planner, path, makePoint(1.0, 0.0));

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kNominal);
  EXPECT_NEAR(plan.reference_velocity_target, 0.8, 1e-12);
  EXPECT_NEAR(plan.reference_velocity, 0.8, 1e-12);
}

TEST(HolonomicSpeedPlanner, bodyAxesAndCombinedSpeedLimitPathDirection)
{
  const double diagonal = std::sqrt(0.5);
  const auto diagonal_path = preparePath(
    makePath({{0.0, 0.0, 0.0}, {10.0 * diagonal, 10.0 * diagonal, 0.0}}));
  auto config = makeConfig();
  config.max_vx = 0.6;
  config.max_vy = 0.3;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(diagonal_path);
  planner.seed(2.0);

  const auto axis_limited = planAt(planner, diagonal_path, makePoint(0.0, 0.0));

  EXPECT_EQ(axis_limited.speed_limit_reason, HolonomicSpeedLimitReason::kBodyAxis);
  EXPECT_NEAR(axis_limited.axis_speed_limit, 0.3 / diagonal, 1e-12);
  EXPECT_NEAR(axis_limited.reference_velocity_target, 0.3 / diagonal, 1e-12);

  const auto straight_path = preparePath(
    makePath({{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}));
  config.max_vx = 3.0;
  config.max_vy = 3.0;
  config.max_xy_speed = 0.35;
  HolonomicSpeedPlanner xy_planner(config);
  xy_planner.setPath(straight_path);
  xy_planner.seed(2.0);

  const auto xy_limited = planAt(xy_planner, straight_path, makePoint(0.0, 0.0));

  EXPECT_EQ(xy_limited.speed_limit_reason, HolonomicSpeedLimitReason::kCombinedSpeed);
  EXPECT_NEAR(xy_limited.axis_speed_limit, 3.0, 1e-12);
  EXPECT_NEAR(xy_limited.reference_velocity_target, 0.35, 1e-12);
}

TEST(HolonomicSpeedPlanner, targetYawSpatialRateReservesFeedbackAuthority)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {4.0, 0.0, 2.0}}));
  auto config = makeConfig();
  config.max_omega = 0.6;
  config.yaw_feedback_reserve = 0.2;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(2.0);

  const auto plan = planAt(planner, path, makePoint(0.0, 0.0));

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kYawRate);
  EXPECT_NEAR(plan.max_abs_target_yaw_rate_per_meter, 0.5, 1e-12);
  EXPECT_NEAR(plan.yaw_speed_limit, 0.8, 1e-12);
  EXPECT_NEAR(plan.reference_velocity_target, 0.8, 1e-12);
}

TEST(HolonomicSpeedPlanner, pathTangentSpatialRateUsesNormalAcceleration)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0}, {1.0, 2.0, 0.0}}));
  auto config = makeConfig();
  config.max_ax = 0.25;
  config.max_ay = 1.0;
  config.a_ref_up = 0.25;
  config.a_ref_down = 0.25;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(2.0);

  const auto plan = planAt(planner, path, makePoint(0.0, 0.0));
  const double expected_rate = 0.5 * kPi / 2.0;

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kTangentRate);
  EXPECT_NEAR(plan.max_abs_path_tangent_rate_per_meter, expected_rate, 1e-12);
  EXPECT_NEAR(plan.tangent_speed_limit, std::sqrt(0.25 / expected_rate), 1e-12);
  EXPECT_NEAR(plan.reference_velocity_target, plan.tangent_speed_limit, 1e-12);
}

TEST(HolonomicSpeedPlanner, propagatesFutureYawConstraintBackByDistance)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {6.0, 0.0, 0.0},
        {7.0, 0.0, 2.0}, {8.0, 0.0, 2.0}}));
  auto config = makeConfig();
  config.max_omega = 0.2;
  config.max_ax = 1.0;
  config.max_ay = 1.0;
  config.a_ref_up = 0.5;
  config.a_ref_down = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(2.0);

  const auto plan = planAt(planner, path, makePoint(0.0, 0.0), 0.0, 100.0);

  EXPECT_NEAR(plan.max_abs_target_yaw_rate_per_meter, 1.0, 1e-12);
  EXPECT_NEAR(plan.yaw_speed_limit, std::sqrt(2.16), 1e-12);
  EXPECT_NEAR(plan.stopping_speed_limit, std::sqrt(8.0), 1e-12);
  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kYawRate);
  EXPECT_NEAR(plan.reference_velocity_target, std::sqrt(2.16), 1e-12);
}

TEST(HolonomicSpeedPlanner, earlierRotationStopOverridesTerminalStop)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0},
        {3.0, 0.0, 0.5 * kPi}, {10.0, 0.0, 0.5 * kPi}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 5.0;
  config.max_vx = 6.0;
  config.max_vy = 6.0;
  config.max_xy_speed = 6.0;
  config.max_ax = 1.0;
  config.max_ay = 1.0;
  config.a_ref_up = 1.0;
  config.a_ref_down = 1.0;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(5.0);

  const auto plan = planAt(planner, path, makePoint(0.0, 0.0), 0.0, 10.0);

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kStoppingDistance);
  EXPECT_NEAR(plan.stopping_speed_limit, std::sqrt(6.0), 1e-12);
  EXPECT_NEAR(plan.reference_velocity_target, std::sqrt(6.0), 1e-12);
}

TEST(HolonomicSpeedPlanner, terminalStopPropagatesToCurrentPosition)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 5.0;
  config.max_vx = 6.0;
  config.max_vy = 6.0;
  config.max_xy_speed = 6.0;
  config.max_ax = 0.5;
  config.max_ay = 0.5;
  config.a_ref_up = 0.5;
  config.a_ref_down = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(5.0);

  const auto plan = planAt(planner, path, makePoint(0.0, 0.0), 0.0, 10.0);

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kStoppingDistance);
  EXPECT_NEAR(plan.stopping_speed_limit, std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(plan.reference_velocity_target, std::sqrt(2.0), 1e-12);
}

TEST(HolonomicSpeedPlanner, shortestAngleWindowsIgnoreNoOpsAndStopAtRotation)
{
  const double yaw_179 = 179.0 * kPi / 180.0;
  const double yaw_minus_179 = -179.0 * kPi / 180.0;
  const auto unwrap_path = preparePath(
    makePath({{0.0, 0.0, yaw_179}, {1.0, 0.0, yaw_minus_179}}));
  auto config = makeConfig();
  HolonomicSpeedPlanner unwrap_planner(config);
  unwrap_planner.setPath(unwrap_path);
  unwrap_planner.seed(0.0);

  const auto unwrap_plan = planAt(
    unwrap_planner, unwrap_path, makePoint(0.0, 0.0), 0.0, 0.1);

  EXPECT_NEAR(
    unwrap_plan.max_abs_target_yaw_rate_per_meter,
    2.0 * kPi / 180.0, 1e-12);

  const auto rotation_path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, kPi}, {0.0, 0.0, kPi}}));
  HolonomicSpeedPlanner rotation_planner(config);
  rotation_planner.setPath(rotation_path);
  rotation_planner.seed(0.0);

  const auto rotation_plan = planAt(
    rotation_planner, rotation_path, makePoint(0.0, 0.0), 0.0, 0.1);

  EXPECT_NEAR(rotation_plan.max_abs_target_yaw_rate_per_meter, 0.0, 1e-12);
  EXPECT_NEAR(rotation_plan.max_abs_path_tangent_rate_per_meter, 0.0, 1e-12);

  const auto no_op_path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}));
  HolonomicSpeedPlanner no_op_planner(config);
  no_op_planner.setPath(no_op_path);
  no_op_planner.seed(0.0);

  const auto no_op_plan = planAt(
    no_op_planner, no_op_path, makePoint(0.0, 0.0), 0.0, 0.1);

  EXPECT_TRUE(std::isfinite(no_op_plan.reference_velocity_target));
  EXPECT_NEAR(no_op_plan.max_abs_target_yaw_rate_per_meter, 0.0, 1e-12);
  EXPECT_NEAR(no_op_plan.max_abs_path_tangent_rate_per_meter, 0.0, 1e-12);
}

TEST(HolonomicSpeedPlanner, referenceVelocityHonorsSeparateRiseAndFallRates)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 1.0;
  config.a_ref_up = 0.2;
  config.a_ref_down = 0.4;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.0);

  const auto rising = planAt(planner, path, makePoint(0.0, 0.0), 0.0, 0.1);
  const auto falling = planAt(planner, path, makePoint(0.0, 0.0), 0.0, 0.1, 0.0);

  EXPECT_NEAR(rising.reference_velocity_target, 1.0, 1e-12);
  EXPECT_NEAR(rising.reference_velocity, 0.02, 1e-12);
  EXPECT_NEAR(falling.reference_velocity_target, 0.0, 1e-12);
  EXPECT_NEAR(falling.reference_velocity, 0.0, 1e-12);
}

TEST(HolonomicSpeedPlanner, rejectsCursorThatIsInconsistentWithInstalledPath)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}));
  HolonomicSpeedPlanner planner(makeConfig());
  planner.setPath(path);
  planner.seed(0.0);

  auto invalid_fraction = makeReference(path, makePoint(0.5, 0.0));
  invalid_fraction.current.cursor.segment_fraction = 1.2;
  EXPECT_THROW(
    planner.plan(invalid_fraction, 0.0, 0.1),
    std::invalid_argument);

  auto inconsistent_arc = makeReference(path, makePoint(0.5, 0.0));
  inconsistent_arc.current.cursor.translation_arc_length = 1.5;
  EXPECT_THROW(
    planner.plan(inconsistent_arc, 0.0, 0.1),
    std::invalid_argument);

  auto inconsistent_position = makeReference(path, makePoint(0.5, 0.0));
  inconsistent_position.current.position.y = 0.1;
  EXPECT_THROW(
    planner.plan(inconsistent_position, 0.0, 0.1),
    std::invalid_argument);
}

TEST(HolonomicSpeedPlanner, extremelyShortEffectiveWindowDoesNotCreateRateSpike)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {1.0, 0.0, 1.0}}));
  HolonomicSpeedPlanner planner(makeConfig());
  planner.setPath(path);
  planner.seed(0.0);

  auto reference = makeReference(path, makePoint(0.0, 0.0));
  reference.mode = HolonomicReferenceMode::kTranslation;
  reference.current.cursor.segment_fraction = 1.0 - 5e-10;
  reference.current.cursor.translation_arc_length = 1.0 - 5e-10;
  reference.current.position.x = 1.0 - 5e-10;

  const auto plan = planner.plan(reference, 0.0, 0.1);

  EXPECT_NEAR(plan.max_abs_target_yaw_rate_per_meter, 0.0, 1e-12);
  EXPECT_NEAR(plan.max_abs_path_tangent_rate_per_meter, 0.0, 1e-12);
  EXPECT_TRUE(std::isfinite(plan.reference_velocity_target));
}

TEST(HolonomicSpeedPlanner, convertsNav2AbsoluteAndPercentageSpeedLimits)
{
  EXPECT_NEAR(convertExternalSpeedLimitToAbsolute(0.3, false, 0.8), 0.3, 1e-12);
  EXPECT_NEAR(convertExternalSpeedLimitToAbsolute(25.0, true, 0.8), 0.2, 1e-12);
  EXPECT_TRUE(std::isinf(convertExternalSpeedLimitToAbsolute(0.0, false, 0.8)));
  EXPECT_TRUE(std::isinf(convertExternalSpeedLimitToAbsolute(0.0, true, 0.8)));
  EXPECT_THROW(
    convertExternalSpeedLimitToAbsolute(101.0, true, 0.8),
    std::invalid_argument);
}

TEST(HolonomicSpeedPlanner, rejectsInvalidConfigurationStateAndInputs)
{
  auto invalid_config = makeConfig();
  invalid_config.a_ref_down = invalid_config.max_ay + 0.1;
  EXPECT_THROW(
    (void)HolonomicSpeedPlanner{invalid_config},
    std::invalid_argument);

  HolonomicSpeedPlanner planner(makeConfig());
  EXPECT_THROW(planner.seed(-0.1), std::domain_error);

  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}));
  planner.setPath(path);
  planner.seed(0.0);
  EXPECT_THROW(
    planner.plan(makeReference(path, makePoint(0.0, 0.0)), 0.0, 0.0),
    std::domain_error);
}

}  // namespace
}  // namespace pid_tracking_controller
