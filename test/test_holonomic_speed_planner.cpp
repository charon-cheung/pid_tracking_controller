#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "holonomic_speed_planner.hpp"
#include "holonomic_acceleration_envelope.hpp"
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
  config.stopping_deceleration = config.a_ref_down = 4.0;
  config.yaw_feedback_reserve = 0.0;
  config.configured_min_rate_window = 0.0;
  config.stopping_delay = 0.0;
  // Existing fixtures use the physical axis budget unless a tracking budget is under test.
  config.max_translation_curve_accel = 4.0;
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

// A unit-radius arc at 0.8 m/s requires 0.64 m/s^2 normal acceleration.
// The tracking budget should slow it in advance while leaving a straight at 0.8.
TEST(HolonomicSpeedPlanner, defaultTrackingBudgetSlowsCurveButKeepsStraightSpeed)
{
  nav_msgs::msg::Path curve;
  curve.header.frame_id = "map";
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position = makePoint(std::sin(0.02 * i), 1.0 - std::cos(0.02 * i));
    pose.pose.orientation.w = 1.0;
    curve.poses.push_back(pose);
  }
  const auto path = preparePath(curve);
  HolonomicSpeedPlannerConfig config;
  config.max_ax = config.max_ay = 1.1;
  config.a_ref_up = 0.8;
  config.a_ref_down = config.stopping_deceleration = 0.4;
  config.stopping_delay = 0.0;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.8);
  const auto turn = planAt(planner, path, curve.poses[5].pose.position, 0.0, 1.0 / 15.0);
  EXPECT_LT(turn.reference_velocity_target, 0.62);
  EXPECT_GT(turn.reference_velocity_target, 0.4);
  EXPECT_GE(turn.reference_velocity, 0.8 - 0.4 / 15.0 - 1e-12);
  const auto straight = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  planner.setPath(straight);
  planner.seed(0.8);
  EXPECT_NEAR(planAt(planner, straight, makePoint(1.0, 0.0)).reference_velocity, 0.8, 1e-12);
}

TEST(HolonomicSpeedPlanner, smoothTurnBudgetAvoidsPerVertexSamplingSlowdown)
{
  // Radius 2 m at 0.8 m/s needs 0.32 m/s^2, below the 0.35 tracking budget.
  for (const double angle_step : {0.01, 0.05}) {
    nav_msgs::msg::Path curve;
    curve.header.frame_id = "map";
    for (int i = 0; i <= static_cast<int>(2.0 / angle_step); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = makePoint(2.0 * std::sin(angle_step * i),
        2.0 * (1.0 - std::cos(angle_step * i)));
      pose.pose.orientation.w = 1.0;
      curve.poses.push_back(pose);
    }
    const auto path = preparePath(curve);
    HolonomicSpeedPlannerConfig config;
    config.max_ax = config.max_ay = 1.1;
    config.a_ref_up = 0.8;
    config.a_ref_down = config.stopping_deceleration = 0.4;
    config.stopping_delay = 0.0;
    HolonomicSpeedPlanner planner(config);
    planner.setPath(path);
    planner.seed(0.8);
    const auto result = planAt(planner, path,
      makePoint(2.0 * std::sin(0.2), 2.0 * (1.0 - std::cos(0.2))), 0.0, 1.0 / 15.0);
    // Windowed piecewise curvature is conservative (up to 0.625 /m here),
    // but must not introduce the per-vertex cap of roughly 0.55 m/s.
    EXPECT_GT(result.reference_velocity_target, 0.74);
  }
}

TEST(HolonomicSpeedPlanner, trackingHoldPreservesCorrectionAndCannotBeOverriddenByReachability)
{
  for (const bool curved : {false, true}) {
    for (const double yaw : {0.0, 1.2}) {
      for (const double side : {-1.0, 1.0}) {
        nav_msgs::msg::Path curve;
        curve.header.frame_id = "map";
        for (int i = 0; i <= 60; ++i) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position = curved ? makePoint(2.0 * std::sin(0.05 * i),
            2.0 * (1.0 - std::cos(0.05 * i))) : makePoint(0.1 * i, 0.0);
          pose.pose.orientation = makeQuaternion(yaw);
          curve.poses.push_back(pose);
        }
        const auto path = preparePath(curve);
        HolonomicSpeedPlannerConfig config;
        config.max_ax = config.max_ay = 1.1;
        config.a_ref_up = 0.8;
        config.a_ref_down = config.stopping_deceleration = 0.4;
        config.stopping_delay = 0.0;
        HolonomicSpeedPlanner planner(config);
        planner.setPath(path);
        planner.seed(0.1);
        const auto reference = makeReference(path, curve.poses[5].pose.position);
        const double tangent = reference.current.path_tangent - yaw;
        const double tx = std::cos(tangent), ty = std::sin(tangent);
        const double nx = -ty, ny = tx;
        const auto observation = [&](double error) {
            // Straight cases reproduce growing position error with inward odometry;
            // curve cases retain the existing outward-velocity entry coverage.
            const double normal_speed = side * (curved ? 0.1 : -0.01);
            return HolonomicTrackingObservation{side * error,
            0.2 * tx + normal_speed * nx, 0.2 * ty + normal_speed * ny};
          };
        for (const double error : {0.04, 0.05, 0.06}) {
          EXPECT_FALSE(planner.plan(reference, yaw, 0.1, 0.8, 0.2,
            {-side * error * nx, -side * error * ny}, std::nullopt,
            observation(error)).tracking_acceleration_held);
        }
        // The physical rectangle demands a higher FF speed than the held 0.34.
        // It must report infeasibility, never undo the tracking acceleration hold.
        const auto held = planner.plan(reference, yaw, 0.1, 0.8, 0.2,
            {-side * 0.07 * nx, -side * 0.07 * ny},
          HolonomicBodyCommand{0.7 * tx, 0.7 * ty, 0.0}, observation(0.07));
        EXPECT_TRUE(held.tracking_acceleration_held);
        EXPECT_TRUE(held.acceleration_unreachable);
        EXPECT_NEAR(held.reference_velocity, 0.34, 1e-10);
        EXPECT_EQ(held.speed_limit_reason, HolonomicSpeedLimitReason::kTrackingResponse);
        // Holding FF leaves the entire requested normal correction intact.
        EXPECT_NEAR(nx * (held.reference_velocity * tx - side * 0.07 * nx) +
          ny * (held.reference_velocity * ty - side * 0.07 * ny), -side * 0.07, 1e-12);
        const auto capped = planner.plan(reference, yaw, 0.1, 0.1, 0.2, {}, std::nullopt,
          observation(0.07));
        EXPECT_TRUE(capped.tracking_acceleration_held);
        EXPECT_LE(capped.reference_velocity, 0.1);
        EXPECT_LE(planner.plan(reference, yaw, 0.1, 0.8, 0.2, {}, std::nullopt,
          observation(0.07)).reference_velocity, 0.1);
        // Sustained small error releases the hold with the normal acceleration ramp.
        for (int i = 0; i < 4; ++i) {
          EXPECT_TRUE(planner.plan(reference, yaw, 0.1, 0.8, 0.2, {}, std::nullopt,
            observation(0.01)).tracking_acceleration_held);
        }
        const auto released = planner.plan(reference, yaw, 0.1, 0.8, 0.2, {}, std::nullopt,
          observation(0.01));
        EXPECT_FALSE(released.tracking_acceleration_held);
        EXPECT_NEAR(released.reference_velocity, 0.18, 1e-10);
      }
    }
  }
}

TEST(HolonomicSpeedPlanner, trackingHoldResetsAtPathSeedRotationTerminalAndMissingObservation)
{
  const auto path = preparePath(makePath({{0, 0, 0}, {1, 0, 0}, {1.2, 0.1, 0}, {5, 3, 0}}));
  for (int reset = 0; reset < 5; ++reset) {
    auto config = makeConfig();
    HolonomicSpeedPlanner planner(config);
    planner.setPath(path);
    auto reference = makeReference(path, makePoint(0.9, 0.0));
    for (int i = 0; i < 5; ++i) {
      planner.plan(reference, 0.0, 0.1, 0.8, 0.2, {}, std::nullopt,
        HolonomicTrackingObservation{0.04 + 0.01 * i, 0.2, 0.1});
    }
    ASSERT_TRUE(planner.plan(reference, 0.0, 0.1, 0.8, 0.2, {}, std::nullopt,
      HolonomicTrackingObservation{0.08, 0.2, 0.1}).tracking_acceleration_held);
    PathProgressTracker approach_tracker;
    approach_tracker.setPath(path);
    const auto approach_progress = approach_tracker.initializePathHandoff(makePoint(4.999, 2.999), 0.5);
    const auto approaching = buildHolonomicTrackingReference(*path, approach_progress, 0.5);
    ASSERT_EQ(approaching.current.cursor.segment_index, path->segments.size() - 1u);
    const auto stopping = planner.plan(approaching, 0.0, 0.1, 0.8, 0.2, {}, std::nullopt,
      HolonomicTrackingObservation{0.08, 0.2, 0.1});
    EXPECT_TRUE(stopping.tracking_acceleration_held);
    EXPECT_LE(stopping.reference_velocity, stopping.stopping_speed_limit);
    EXPECT_LT(stopping.reference_velocity, 0.12);
    if (reset == 0) {
      planner.setPath(path);
    } else if (reset == 1) {
      planner.seed(0.2);
    } else if (reset == 2 || reset == 3) {
      auto stopped = reference;
      stopped.mode = reset == 2 ? HolonomicReferenceMode::kRotationPending :
        HolonomicReferenceMode::kTerminal;
      EXPECT_DOUBLE_EQ(planner.plan(stopped, 0.0, 0.1).reference_velocity, 0.0);
    } else {
      planner.plan(reference, 0.0, 0.1);
    }
    EXPECT_FALSE(planner.plan(reference, 0.0, 0.1, 0.8, 0.2, {}, std::nullopt,
      HolonomicTrackingObservation{0.08, 0.2, 0.1}).tracking_acceleration_held);
  }
}

TEST(HolonomicSpeedPlanner, feedbackReservesEitherBodyAxisInBothDirections)
{
  for (const bool swap_axes : {false, true}) {
    for (const double sx : {-1.0, 1.0}) {
      for (const double sy : {-1.0, 1.0}) {
        for (const double yaw : {0.0, 1.0}) {
          SCOPED_TRACE(::testing::Message() << swap_axes << ", " << sx << ", " << sy << ", " << yaw);
          const double tx = sx * (swap_axes ? 0.8 : 0.6);
          const double ty = sy * (swap_axes ? 0.6 : 0.8);
          const HolonomicTranslationFeedback feedback{
            sx * (swap_axes ? 0.06 : -0.08), sy * (swap_axes ? -0.08 : 0.06)};
          const auto path = preparePath(makePath({{0.0, 0.0, yaw},
            {20.0 * (std::cos(yaw) * tx - std::sin(yaw) * ty),
              20.0 * (std::sin(yaw) * tx + std::cos(yaw) * ty), yaw}}));
          auto config = makeConfig();
          config.max_vx = swap_axes ? 0.4 : 0.8;
          config.max_vy = swap_axes ? 0.8 : 0.4;
          HolonomicSpeedPlanner planner(config);
          planner.setPath(path);
          const auto result = planner.plan(
            makeReference(path, makePoint(0.0, 0.0)), yaw, 1.0, 0.8, 0.0, feedback);
          EXPECT_NEAR(result.reference_velocity, 0.425, 1e-12);
          EXPECT_EQ(result.speed_limit_reason, HolonomicSpeedLimitReason::kFeedbackBudget);
          EXPECT_FALSE(result.feedback_unreachable);
          EXPECT_LE(std::abs(tx * result.reference_velocity + feedback.vx), config.max_vx + 1e-12);
          EXPECT_LE(std::abs(ty * result.reference_velocity + feedback.vy), config.max_vy + 1e-12);
        }
      }
    }
  }
}

TEST(HolonomicSpeedPlanner, feedbackRespectsCombinedExternalAndStoppingCaps)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  // Each case has a 0.5 m/s combined budget, imposed by a different source.
  for (const int source : {0, 1, 2}) {
    SCOPED_TRACE(source);
    auto config = makeConfig();
    config.max_xy_speed = source == 0 ? 0.5 : 1.0;
    config.stopping_deceleration = 0.25;
    HolonomicSpeedPlanner planner(config);
    planner.setPath(path);
    const auto result = planner.plan(
      makeReference(path, makePoint(source == 2 ? 19.5 : 0.0, 0.0)),
      0.0, 1.0, source == 1 ? 0.5 : 1.0, 0.0, {0.0, 0.3});
    EXPECT_NEAR(result.reference_velocity, 0.4, 1e-12);
    EXPECT_NEAR(std::hypot(result.reference_velocity, 0.3), 0.5, 1e-12);
    EXPECT_FALSE(result.feedback_unreachable);
  }
}

TEST(HolonomicSpeedPlanner, oppositeFeedbackCanBeCancelledWithoutIncreasingOriginalSpeed)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.max_xy_speed = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  for (const double feedback : {-0.2, -0.8}) {
    const auto result = planner.plan(
      makeReference(path, makePoint(0.0, 0.0)), 0.0, 1.0, 1.0, 0.0, {feedback, 0.0});
    EXPECT_NEAR(result.reference_velocity, 0.5, 1e-12);
    EXPECT_FALSE(result.feedback_unreachable);
    EXPECT_LE(std::abs(result.reference_velocity + feedback), 0.5);
  }
}

TEST(HolonomicSpeedPlanner, positiveFeasibilityMinimumDoesNotBypassReferenceAcceleration)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.max_xy_speed = 0.5;
  config.a_ref_up = 0.2;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  const auto reference = makeReference(path, makePoint(0.0, 0.0));
  // FF must reach 0.3 to cancel enough feedback, but must not jump there.
  const auto starting = planner.plan(reference, 0.0, 0.1, 1.0, 0.0, {-0.8, 0.0});
  EXPECT_NEAR(starting.reference_velocity, 0.02, 1e-12);
  EXPECT_TRUE(starting.feedback_unreachable);
  const auto later = planner.plan(reference, 0.0, 2.0, 1.0, 0.0, {-0.8, 0.0});
  EXPECT_NEAR(later.reference_velocity, 0.42, 1e-12);
  EXPECT_FALSE(later.feedback_unreachable);
}

TEST(HolonomicSpeedPlanner, infeasibleFeedbackRemovesOnlyFeedforward)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.max_vy = config.max_xy_speed = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  const auto reference = makeReference(path, makePoint(0.0, 0.0));
  // Perpendicular excess, forward excess, and cancellation requiring excessive FF.
  for (const auto feedback : {HolonomicTranslationFeedback{0.0, 0.6},
    HolonomicTranslationFeedback{0.6, 0.0}, HolonomicTranslationFeedback{-1.1, 0.0}})
  {
    planner.seed(0.5);
    const auto result = planner.plan(reference, 0.0, 0.01, 1.0, 0.0, feedback);
    EXPECT_DOUBLE_EQ(result.reference_velocity, 0.0);
    EXPECT_TRUE(result.feedback_unreachable);
  }
}

TEST(HolonomicSpeedPlanner, feedbackBudgetBecomesRecoveryStateWithoutForcingMinimumSpeed)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.max_xy_speed = 0.5;
  config.a_ref_up = config.a_ref_down = 0.2;
  config.stopping_min_speed = 0.06;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.5);
  const auto reference = makeReference(path, makePoint(0.0, 0.0));
  const auto limited = planner.plan(reference, 0.0, 0.1, 1.0, 0.0, {0.0, 0.3});
  EXPECT_NEAR(limited.reference_velocity, 0.4, 1e-12);
  const auto recovered = planner.plan(reference, 0.0, 0.1);
  EXPECT_NEAR(recovered.reference_velocity, 0.42, 1e-12);

  const auto near_goal = makeReference(path, makePoint(19.9999, 0.0));
  const auto approaching = planner.plan(near_goal, 0.0, 0.1, 1.0, 0.0, {0.0, 0.05});
  EXPECT_NEAR(approaching.stopping_speed_limit, 0.06, 1e-12);
  EXPECT_NEAR(approaching.reference_velocity, std::sqrt(0.0011), 1e-12);
  EXPECT_NEAR(std::hypot(approaching.reference_velocity, 0.05), 0.06, 1e-12);
}

TEST(HolonomicSpeedPlanner, nonbindingFeedbackPreservesExistingReferenceDeceleration)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.5;
  config.a_ref_down = 0.2;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  for (const double lateral_feedback : {0.0, 0.01}) {
    planner.seed(0.8);
    const auto result = planner.plan(
      makeReference(path, makePoint(0.0, 0.0)), 0.0, 0.1, 1.0, 0.0, {0.0, lateral_feedback});
    EXPECT_NEAR(result.reference_velocity, 0.78, 1e-12);
    EXPECT_EQ(result.speed_limit_reason, HolonomicSpeedLimitReason::kNominal);
  }
}

TEST(HolonomicSpeedPlanner, feedbackCannotCreateFeedforwardDuringRotationOrTerminal)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  HolonomicSpeedPlanner planner(makeConfig());
  planner.setPath(path);
  auto reference = makeReference(path, makePoint(0.0, 0.0));
  for (const auto mode : {HolonomicReferenceMode::kRotationPending, HolonomicReferenceMode::kTerminal}) {
    reference.mode = mode;
    planner.seed(0.8);
    EXPECT_DOUBLE_EQ(planner.plan(reference, 0.0, 0.1, 1.0, 0.0, {-0.8, 0.3}).reference_velocity, 0.0);
    reference.mode = HolonomicReferenceMode::kTranslation;
    EXPECT_NEAR(planner.plan(reference, 0.0, 0.1).reference_velocity, 0.4, 1e-12);
  }
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(planner.plan(reference, 0.0, 0.1, 1.0, 0.0, {invalid, 0.0}), std::domain_error);
    EXPECT_THROW(planner.plan(reference, 0.0, 0.1, 1.0, 0.0, {0.0, invalid}), std::domain_error);
  }
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
  config.stopping_deceleration = config.a_ref_down = 0.25;
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


TEST(HolonomicSpeedPlanner, shortOpposingTurnBrakesBeforeTheDirectionJump)
{
  const double angle = 0.7;
  const double next_x = 1.0 + 0.05 * std::cos(angle);
  const double next_y = 0.05 * std::sin(angle);
  const auto path = preparePath(makePath({
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {next_x, next_y, 0.0},
    {next_x + 1.0, next_y, 0.0}, {next_x + 2.0, next_y, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.8;
  config.max_ax = config.max_ay = 1.0;
  config.a_ref_up = 0.8;
  config.a_ref_down = config.stopping_deceleration = 0.4;
  config.configured_min_rate_window = 0.3;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  const double dt = 1.0 / 15.0;
  const auto result = planAt(planner, path, makePoint(0.99, 0.0), 0.0, dt);
  // Only 1 cm remains before a 0.7 rad tangent jump. Its y command must
  // change by v*sin(0.7) within a cycle, even though the next turn cancels it.
  const double corner_speed = config.max_ay * dt / std::sin(angle);
  EXPECT_LE(result.reference_velocity_target,
    std::sqrt(corner_speed * corner_speed + 2.0 * config.a_ref_down * 0.01) + 1e-9);
}

TEST(HolonomicSpeedPlanner, curveBudgetLeavesAccelerationForNormalCorrection)
{
  nav_msgs::msg::Path curve;
  curve.header.frame_id = "map";
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position = makePoint(std::sin(0.02 * i), 1.0 - std::cos(0.02 * i));
    pose.pose.orientation.w = 1.0;
    curve.poses.push_back(pose);
  }
  const auto path = preparePath(curve);
  constexpr double dt = 1.0 / 15.0;
  for (const double yaw : {0.0, 1.2}) {
    HolonomicSpeedPlannerConfig config;
    config.max_ax = config.max_ay = 1.1;
    config.a_ref_up = 0.8;
    config.a_ref_down = config.stopping_deceleration = 0.4;
    config.stopping_delay = 0.0;
    HolonomicSpeedPlanner planner(config);
    planner.setPath(path);
    planner.seed(0.8);
    const auto reference = makeReference(path, curve.poses[5].pose.position);
    const double speed = planner.plan(reference, yaw, dt).reference_velocity_target;
    EXPECT_LT(speed, 0.62);
    // A unit-radius turn advances its tangent by speed*dt each cycle.
    const double before = reference.current.path_tangent - yaw;
    const double after = before + speed * dt;
    const double nx = -std::sin(after), ny = std::cos(after);
    HolonomicCommandLimiterConfig limits;
    limits.max_ax = limits.max_ay = 1.1;
    HolonomicCommandLimiter limiter(limits);
    const HolonomicBodyCommand previous{speed * std::cos(before), speed * std::sin(before), 0.0};
    limiter.seed(previous);
    const auto result = limiter.limit(
      {speed * std::cos(after) + 0.03 * nx, speed * std::sin(after) + 0.03 * ny, 0.0},
      previous, dt, 0.8);
    EXPECT_LE(std::abs(result.command.vx - previous.vx), 1.1 * dt + 1e-12);
    EXPECT_LE(std::abs(result.command.vy - previous.vy), 1.1 * dt + 1e-12);
    EXPECT_NEAR(nx * result.command.vx + ny * result.command.vy, 0.03, 1e-10);
  }
}

TEST(HolonomicSpeedPlanner, curveBudgetKeepsStraightAccelerationAndStopBudgets)
{
  const auto straight = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  for (const double budget : {1.0, 0.35, 0.2}) {
    SCOPED_TRACE(budget);
    auto config = makeConfig();
    config.nominal_tracking_speed = 0.8;
    config.max_ax = config.max_ay = 1.0;
    config.a_ref_up = 0.8;
    config.a_ref_down = config.stopping_deceleration = 0.4;
    config.max_translation_curve_accel = budget;
    HolonomicSpeedPlanner planner(config);
    planner.setPath(straight);
    planner.seed(0.4);
    EXPECT_NEAR(planAt(planner, straight, makePoint(1.0, 0.0), 0.0, 0.1).
      reference_velocity, 0.48, 1e-12);
    planner.seed(0.8);
    EXPECT_NEAR(planAt(planner, straight, makePoint(1.0, 0.0), 0.0, 0.1).
      reference_velocity, 0.8, 1e-12);
    const auto stop = planAt(planner, straight, makePoint(19.9, 0.0), 0.0, 0.1);
    EXPECT_NEAR(stop.stopping_speed_limit, std::sqrt(0.08), 1e-12);
    EXPECT_LE(stop.reference_velocity, std::sqrt(0.08) + 1e-12);
  }
}

TEST(HolonomicSpeedPlanner, rejectsInvalidCurveBudget)
{
  for (const double budget : {-0.01, 0.0,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    auto config = makeConfig();
    config.max_translation_curve_accel = budget;
    EXPECT_THROW(HolonomicSpeedPlanner{config}, std::invalid_argument);
  }
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
  config.stopping_deceleration = config.a_ref_down = 0.5;
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
  config.stopping_deceleration = config.a_ref_down = 1.0;
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
  config.stopping_deceleration = config.a_ref_down = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(5.0);

  const auto plan = planAt(planner, path, makePoint(0.0, 0.0), 0.0, 10.0);

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kStoppingDistance);
  EXPECT_NEAR(plan.stopping_speed_limit, std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(plan.reference_velocity_target, std::sqrt(2.0), 1e-12);
}

TEST(HolonomicSpeedPlanner, rotationPendingImmediatelyClearsReferenceState)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}}));
  HolonomicSpeedPlanner planner(makeConfig());
  planner.setPath(path);
  planner.seed(0.6);
  const auto reference = makeReference(path, makePoint(1.0, 0.0));
  ASSERT_EQ(reference.mode, HolonomicReferenceMode::kRotationPending);

  const auto plan = planner.plan(reference, 0.0, 0.1);

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kRotation);
  EXPECT_DOUBLE_EQ(plan.reference_velocity_target, 0.0);
  EXPECT_DOUBLE_EQ(plan.reference_velocity, 0.0);
}

TEST(HolonomicSpeedPlanner, terminalSentinelImmediatelyClearsReferenceState)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}));
  HolonomicSpeedPlanner planner(makeConfig());
  planner.setPath(path);
  planner.seed(0.6);
  const auto reference = makeReference(path, makePoint(1.0, 0.0));
  ASSERT_EQ(reference.mode, HolonomicReferenceMode::kTerminal);

  const auto plan = planner.plan(reference, 0.0, 0.1);

  EXPECT_EQ(plan.speed_limit_reason, HolonomicSpeedLimitReason::kTerminal);
  EXPECT_DOUBLE_EQ(plan.reference_velocity_target, 0.0);
  EXPECT_DOUBLE_EQ(plan.reference_velocity, 0.0);
}

TEST(HolonomicSpeedPlanner, instantaneousHardLimitsOverrideTemporalRamp)
{
  struct HardLimitCase
  {
    double max_vx;
    double max_xy_speed;
    double external_speed_limit;
    HolonomicSpeedLimitReason reason;
  };
  const HardLimitCase cases[] = {
    {3.0, 3.0, 0.2, HolonomicSpeedLimitReason::kExternal},
    {0.2, 3.0, std::numeric_limits<double>::infinity(),
      HolonomicSpeedLimitReason::kBodyAxis},
    {3.0, 0.2, std::numeric_limits<double>::infinity(),
      HolonomicSpeedLimitReason::kCombinedSpeed}};
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}));

  for (const auto & test_case : cases) {
    SCOPED_TRACE(static_cast<int>(test_case.reason));
    auto config = makeConfig();
    config.nominal_tracking_speed = 1.0;
    config.max_vx = test_case.max_vx;
    config.max_xy_speed = test_case.max_xy_speed;
    config.a_ref_up = 0.1;
    config.stopping_deceleration = config.a_ref_down = 0.1;
    HolonomicSpeedPlanner planner(config);
    planner.setPath(path);
    planner.seed(0.6);

    const auto plan = planAt(
      planner, path, makePoint(0.0, 0.0), 0.0, 0.1,
      test_case.external_speed_limit);

    EXPECT_EQ(plan.speed_limit_reason, test_case.reason);
    EXPECT_NEAR(plan.reference_velocity_target, 0.2, 1e-12);
    EXPECT_LE(plan.reference_velocity, 0.2);

    if (test_case.reason == HolonomicSpeedLimitReason::kExternal) {
      const auto recovery = planAt(
        planner, path, makePoint(0.0, 0.0), 0.0, 0.1);
      EXPECT_NEAR(recovery.reference_velocity, 0.21, 1e-12);
    }
  }
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
  config.stopping_deceleration = config.a_ref_down = 0.4;
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

TEST(HolonomicSpeedPlanner, currentSegmentYawFeedforwardRemainsFeasibleAfterRamping)
{
  for (const double final_yaw : {0.0, 0.1}) {
    const auto path = preparePath(makePath({
      {0.0, 0.0, 0.0}, {0.01, 0.0, 0.1}, {0.02, 0.0, final_yaw},
      {1.02, 0.0, final_yaw}, {2.02, 0.0, final_yaw}}));
    auto config = makeConfig();
    config.nominal_tracking_speed = 0.2;
    config.max_omega = 0.3;
    config.yaw_feedback_reserve = 0.05;
    config.max_ax = config.max_ay = 0.2;
    config.stopping_deceleration = config.a_ref_up = config.a_ref_down = 0.2;
    HolonomicSpeedPlanner planner(config);
    planner.setPath(path);
    planner.seed(0.2);
    const auto reference = makeReference(path, makePoint(0.0, 0.0));
    const auto result = planner.plan(reference, 0.0, 0.1);
    EXPECT_LE(
      std::abs(reference.current.target_yaw_rate_per_meter * result.reference_velocity),
      0.25 + 1e-12);
  }
}

TEST(HolonomicSpeedPlanner, futureShortYawSegmentBrakesBeforeItsBoundary)
{
  const auto path = preparePath(makePath({
    {0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.11, 0.0, 0.1},
    {1.11, 0.0, 0.1}, {2.11, 0.0, 0.1}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.8;
  config.max_omega = 0.3;
  config.yaw_feedback_reserve = 0.05;
  config.max_ax = config.max_ay = 0.2;
  config.stopping_deceleration = config.a_ref_up = config.a_ref_down = 0.2;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  const auto result = planAt(planner, path, makePoint(0.0, 0.0));
  // 0.025 m/s at the boundary, 0.1 m away, with 0.2 m/s^2 deceleration.
  EXPECT_LE(result.reference_velocity_target, std::sqrt(0.000625 + 0.04) + 1e-12);
}


TEST(HolonomicSpeedPlanner, actualCommandBudgetPreservesCorrectionAcrossAxesAndFrames)
{
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      for (double yaw : {0.0, 1.2}) {
        SCOPED_TRACE(::testing::Message() << sx << ", " << sy << ", " << yaw);
        const double tx = sx * 0.6;
        const double ty = sy * 0.8;
        const auto path = preparePath(makePath({
          {0.0, 0.0, yaw},
          {20.0 * (std::cos(yaw) * tx - std::sin(yaw) * ty),
            20.0 * (std::sin(yaw) * tx + std::cos(yaw) * ty), yaw}}));
        auto config = makeConfig();
        config.max_translation_curve_accel = 0.35;
        config.nominal_tracking_speed = config.max_xy_speed = 0.8;
        config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 0.8;
        config.stopping_deceleration = 0.4;
        HolonomicSpeedPlanner planner(config);
        planner.setPath(path);
        planner.seed(0.5);
        const HolonomicTranslationFeedback feedback{-sx * 0.08, sy * 0.02};
        const HolonomicBodyCommand previous{sx * 0.3, sy * 0.4, 0.0};
        const auto result = planner.plan(
          makeReference(path, makePoint(0.0, 0.0)), yaw, 0.1, 0.8, 0.5, feedback, previous);
        ASSERT_FALSE(result.acceleration_unreachable);
        const double vx = tx * result.reference_velocity + feedback.vx;
        const double vy = ty * result.reference_velocity + feedback.vy;
        EXPECT_LE(std::abs(vx - previous.vx), 0.08 + 1e-12);
        EXPECT_LE(std::abs(vy - previous.vy), 0.08 + 1e-12);
        EXPECT_NEAR(result.reference_velocity, 0.575, 1e-12);
        EXPECT_NEAR(-ty * vx + tx * vy, -ty * feedback.vx + tx * feedback.vy, 1e-12);
      }
    }
  }
}

TEST(HolonomicSpeedPlanner, reachableInwardCorrectionSurvivesFinalAccelerationLimiting)
{
  for (const bool swap_axes : {false, true}) {
    for (const double sign : {-1.0, 1.0}) {
      SCOPED_TRACE(::testing::Message() << swap_axes << ", " << sign);
      const auto path = preparePath(makePath({{0.0, 0.0, 0.0},
        {swap_axes ? 0.0 : 20.0, swap_axes ? 20.0 : 0.0, 0.0}}));
      auto config = makeConfig();
      config.nominal_tracking_speed = config.max_xy_speed = 0.8;
      config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 0.8;
      config.stopping_deceleration = 0.4;
      HolonomicCommandLimiterConfig limits;
      limits.max_ax = limits.max_ay = 0.8;
      const HolonomicBodyCommand previous{
        swap_axes ? -sign * 0.03 : 0.4, swap_axes ? 0.4 : -sign * 0.03, 0.0};
      const HolonomicTranslationFeedback feedback{
        swap_axes ? sign * 0.02 : 0.06, swap_axes ? 0.06 : sign * 0.02};
      const auto reference = makeReference(path, makePoint(0.0, 0.0));
      for (const bool coordinate_acceleration : {false, true}) {
        HolonomicSpeedPlanner planner(config);
        planner.setPath(path);
        planner.seed(0.4);
        HolonomicCommandLimiter limiter(limits);
        limiter.seed(previous);
        const auto planned = planner.plan(reference, 0.0, 0.1, 0.8, 0.4, feedback,
          coordinate_acceleration ? limiter.previousCommand() : std::nullopt);
        const HolonomicBodyCommand raw{
          (swap_axes ? 0.0 : planned.reference_velocity) + feedback.vx,
          (swap_axes ? planned.reference_velocity : 0.0) + feedback.vy, 0.0};
        const auto result = limiter.limit(raw, previous, 0.1, 0.8);
        const double inward_command = sign * (swap_axes ? result.command.vx : result.command.vy);
        EXPECT_LE(std::abs(result.command.vx - previous.vx), 0.08 + 1e-12);
        EXPECT_LE(std::abs(result.command.vy - previous.vy), 0.08 + 1e-12);
        if (coordinate_acceleration) {
          EXPECT_FALSE(planned.acceleration_unreachable);
          EXPECT_NEAR(inward_command, 0.02, 1e-12);
          EXPECT_NEAR(planned.reference_velocity, 0.42, 1e-12);
        } else {
          // Without coordination, tangential acceleration also throttles the
          // normal correction, leaving the final command outward despite raw.
          EXPECT_LT(inward_command, 0.0);
          EXPECT_NEAR(planned.reference_velocity, 0.48, 1e-12);
        }
      }
    }
  }
}

TEST(HolonomicSpeedPlanner, emptyAccelerationIntervalDoesNotForceAnUnreachableZero)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {0.0, 20.0, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.8;
  config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 0.8;
  config.stopping_deceleration = 0.4;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.5);
  const auto reference = makeReference(path, makePoint(0.0, 0.0));
  const auto result = planner.plan(reference, 0.0, 0.1, 0.8, 0.5, {},
      HolonomicBodyCommand{0.5, 0.0, 0.0});
  EXPECT_TRUE(result.acceleration_unreachable);
  EXPECT_GT(result.reference_velocity, 0.0);
  EXPECT_LE(result.reference_velocity, 0.58 + 1e-12);
  EXPECT_THROW(planner.plan(reference, 0.0, 0.1, 0.8, 0.0, {},
      HolonomicBodyCommand{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}),
    std::domain_error);
}

TEST(HolonomicSpeedPlanner, positiveAccelerationLowerBoundAndHardCapRemainBinding)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.8;
  config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 0.8;
  config.stopping_deceleration = 0.4;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.5);
  const auto reference = makeReference(path, makePoint(0.0, 0.0));
  const auto feasible = planner.plan(reference, 0.0, 0.1, 0.45, 0.5, {},
      HolonomicBodyCommand{0.5, 0.0, 0.0});
  EXPECT_FALSE(feasible.acceleration_unreachable);
  EXPECT_NEAR(feasible.reference_velocity, 0.45, 1e-12);
  planner.seed(0.5);
  const auto capped = planner.plan(reference, 0.0, 0.1, 0.2, 0.5, {},
      HolonomicBodyCommand{0.5, 0.0, 0.0});
  EXPECT_TRUE(capped.acceleration_unreachable);
  EXPECT_LE(capped.reference_velocity, 0.2);
}

TEST(HolonomicSpeedPlanner, brakingAndRotationDoNotUseOrdinaryAccelerationRectangle)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}));
  auto config = makeConfig();
  config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 0.8;
  config.stopping_deceleration = 0.4;
  config.stopping_delay = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.8);
  auto reference = makeReference(path, makePoint(0.99, 0.0));
  const auto braking = planner.plan(reference, 0.0, 0.1, 0.8, 0.8, {},
      HolonomicBodyCommand{0.8, 0.0, 0.0});
  EXPECT_FALSE(braking.acceleration_unreachable);
  EXPECT_LE(braking.reference_velocity, config.stopping_min_speed);
  reference.mode = HolonomicReferenceMode::kRotationPending;
  const auto rotation = planner.plan(reference, 0.0, 0.1, 0.8, 0.8, {},
      HolonomicBodyCommand{0.8, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(rotation.reference_velocity, 0.0);
  EXPECT_FALSE(rotation.acceleration_unreachable);
}

TEST(HolonomicSpeedPlanner, accelerationPreviewAccountsForBodyHeadingChanges)
{
  auto config = makeConfig();
  config.nominal_tracking_speed = config.max_xy_speed = 0.8;
  config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 1.0;
  config.stopping_deceleration = 0.4;
  double caps[2]{};
  for (int follows = 0; follows < 2; ++follows) {
    nav_msgs::msg::Path message;
    message.header.frame_id = "map";
    for (int index = 0; index <= 100; ++index) {
      const double angle = 0.02 * index;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position = makePoint(0.5 * std::sin(angle), 0.5 * (1.0 - std::cos(angle)));
      pose.pose.orientation = makeQuaternion(follows ? angle : 0.0);
      message.poses.push_back(pose);
    }
    const auto path = preparePath(message);
    const auto reference = makeReference(path, message.poses[20].pose.position);
    caps[follows] = translationAccelerationSpeedLimit(
      *path, reference, follows ? 0.4 : 0.0, 1.0 / 15.0, 0.8, config);
    EXPECT_TRUE(std::isfinite(caps[follows]));
    EXPECT_GT(caps[follows], 0.1);
  }
  // The same map-space bend requires more body-axis acceleration when the
  // chassis keeps a fixed heading than when its heading follows the tangent.
  EXPECT_LT(caps[0], caps[1] - 0.02);
  EXPECT_NEAR(caps[1], 0.8, 1e-9);
}

TEST(HolonomicSpeedPlanner, previewIsFiniteAcrossWrappedAnglesAndRepeatedPoints)
{
  const auto path = preparePath(makePath({
    {0.0, 0.0, 3.13}, {-1.0, 0.01, 3.13}, {-1.0, 0.01, 3.13},
    {-2.0, 0.0, -3.13}, {-3.0, 0.01, -3.13}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.8;
  config.max_ax = config.max_ay = config.a_ref_up = config.a_ref_down = 0.8;
  config.stopping_deceleration = 0.4;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  const auto result = planAt(planner, path, makePoint(-0.99, 0.0099), 3.13, 0.1);
  EXPECT_TRUE(std::isfinite(result.acceleration_speed_limit));
  EXPECT_GT(result.acceleration_speed_limit, 0.5);
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

TEST(HolonomicSpeedPlanner, stoppingEnvelopeIsNotRaisedByReferenceSlew)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {1.5, 0.0, 0.0}}));
  auto config = makeConfig();
  config.stopping_deceleration = config.max_ax = config.max_ay = config.a_ref_down = 0.2;
  config.a_ref_up = 0.2;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.4);

  const auto plan = planAt(planner, path, makePoint(1.45, 0.0), 0.0, 1.0 / 15.0);

  // A fast incoming reference must not override the distance-based stop target.
  EXPECT_LE(plan.reference_velocity, plan.stopping_speed_limit + 1e-12);
  EXPECT_LT(plan.reference_velocity, 0.15);
}

TEST(HolonomicSpeedPlanner, observedSpeedReservesDistanceForDelayedBraking)
{
  const auto path = preparePath(makePath({{0.0, 0.0, 0.0}, {1.5, 0.0, 0.0}}));
  auto config = makeConfig();
  config.stopping_deceleration = 0.12;
  config.stopping_delay = 0.5;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  const auto reference = makeReference(path, makePoint(1.0, 0.0));
  planner.seed(0.0);
  const auto stationary = planner.plan(reference, 0.0, 0.1);
  planner.seed(0.0);
  const auto moving = planner.plan(
    reference, 0.0, 0.1, std::numeric_limits<double>::infinity(), 0.6);
  // 0.5 m remaining, reserving 0.3 m of travel during the delay.
  EXPECT_NEAR(moving.stopping_speed_limit, std::sqrt(0.048), 1e-12);
  EXPECT_LT(moving.reference_velocity, stationary.reference_velocity);
  EXPECT_THROW(planner.plan(reference, 0.0, 0.1, 1.0, -0.1), std::domain_error);
  EXPECT_THROW(planner.plan(reference, 0.0, 0.1, 1.0,
    std::numeric_limits<double>::quiet_NaN()), std::domain_error);

  config.stopping_delay = -0.1;
  EXPECT_THROW(HolonomicSpeedPlanner{config}, std::invalid_argument);
  config.stopping_delay = 0.5;
  config.stopping_deceleration = 0.0;
  EXPECT_THROW(HolonomicSpeedPlanner{config}, std::invalid_argument);
}


TEST(HolonomicSpeedPlanner, obstacleEnvelopeReservesDelayWithoutApproachFloor)
{
  const auto path = preparePath(makePath({{0, 0, 0}, {20, 0, 0}}));
  const auto ref = makeReference(path, makePoint(0, 0));
  auto config = makeConfig();
  config.stopping_deceleration = .4;
  config.stopping_delay = .5;
  config.stopping_min_speed = .06;
  HolonomicSpeedPlanner planner(config); planner.setPath(path);
  double previous_cap = std::numeric_limits<double>::infinity();
  for (double distance : {2.0, 1.21, 1.19, .401, .4, .1, 0.0}) {
    planner.seed(.8);
    const auto result = planner.plan(ref, 0, .1, config.max_xy_speed, .8, {},
      HolonomicBodyCommand{.8, 0, 0}, std::nullopt, distance);
    EXPECT_NEAR(result.obstacle_speed_limit, std::sqrt(.8 * std::max(0.0, distance - .4)), 1e-12);
    EXPECT_LE(result.obstacle_speed_limit, previous_cap);
    EXPECT_LE(result.reference_velocity, result.obstacle_speed_limit);
    EXPECT_EQ(result.obstacle_stop_unreachable, distance < 1.2 - 1e-12);
    previous_cap = result.obstacle_speed_limit;
    if (distance <= .4) {EXPECT_DOUBLE_EQ(result.stopping_speed_limit, 0.0);}
    if (distance == .401) {EXPECT_LT(result.stopping_speed_limit, config.stopping_min_speed);}
  }
  // Last actual output must reserve reaction distance even if measured speed is low.
  planner.seed(0);
  const auto previous_output = planner.plan(ref, 0, .1, config.max_xy_speed, 0, {},
    HolonomicBodyCommand{.8, 0, 0}, std::nullopt, .4);
  EXPECT_DOUBLE_EQ(previous_output.obstacle_speed_limit, 0);
  EXPECT_TRUE(previous_output.obstacle_stop_unreachable);
  // An observed speed above the command cap is not by itself an unreachable stop.
  config.max_xy_speed = .5;
  HolonomicSpeedPlanner overspeed(config); overspeed.setPath(path); overspeed.seed(.5);
  const auto distant = overspeed.plan(ref, 0, .1, config.max_xy_speed, .8, {},
    std::nullopt, std::nullopt, 10.0);
  EXPECT_LE(distant.obstacle_speed_limit, config.max_xy_speed);
  EXPECT_FALSE(distant.obstacle_stop_unreachable);
}

TEST(HolonomicSpeedPlanner, obstacleInputValidationAndSafetyHistoryRecovery)
{
  const auto path = preparePath(makePath({{0, 0, 0}, {20, 0, 0}}));
  const auto ref = makeReference(path, makePoint(0, 0));
  auto config = makeConfig(); config.a_ref_up = .4;
  HolonomicSpeedPlanner planner(config); planner.setPath(path); planner.seed(.8);
  for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(planner.plan(ref, 0, .1, config.max_xy_speed, 0, {},
      std::nullopt, std::nullopt, invalid), std::domain_error);
    EXPECT_THROW(planner.commitSafetyOutput(invalid), std::domain_error);
  }
  // Rejected inputs do not corrupt the existing reference history.
  EXPECT_NEAR(planner.plan(ref, 0, .1).reference_velocity, .84, 1e-12);
  planner.commitSafetyOutput(0);
  const auto resumed = planner.plan(ref, 0, .1);
  EXPECT_NEAR(resumed.reference_velocity, .04, 1e-12);
  EXPECT_TRUE(std::isinf(resumed.obstacle_speed_limit));
  EXPECT_FALSE(resumed.obstacle_stop_unreachable);
}

TEST(HolonomicSpeedPlanner, stopHoldAndResumeAcrossPlannerAndLimiter)
{
  const auto path = preparePath(makePath({{0, 0, 0}, {20, 0, 0}}));
  const auto ref = makeReference(path, makePoint(0, 0));
  auto config = makeConfig(); config.stopping_deceleration = .4;
  config.a_ref_up = .4; config.stopping_min_speed = .06;
  HolonomicSpeedPlanner planner(config); planner.setPath(path); planner.seed(.6);
  HolonomicCommandLimiterConfig limits; limits.max_decel_x = .4;
  HolonomicCommandLimiter limiter(limits);
  HolonomicBodyCommand actual{.6, .2, .3}; limiter.seed(actual);
  double distance = 1.0;
  bool stopped = false;
  for (int cycle = 0; cycle < 80; ++cycle) {
    const auto plan = planner.plan(ref, 0, .1, limits.max_xy_speed,
      std::hypot(actual.vx, actual.vy), {}, actual, std::nullopt, distance);
    const auto before = actual;
    // Synthetic safety request; the real obstacle trigger belongs to stage 1E.
    if (cycle >= 10) {
      actual = limiter.brake(actual, .1, before).command;
      planner.commitSafetyOutput(std::max(0.0, actual.vx));
    } else {
      actual = limiter.limit({plan.reference_velocity, 0, 0}, actual, .1,
        limits.max_xy_speed, plan.stopping_speed_limit).command;
    }
    const double x_rate = cycle >= 10 ? limits.max_decel_x :
      std::max(limits.max_ax, limits.max_decel_x);
    EXPECT_LE(std::abs(actual.vx - before.vx), x_rate * .1 + 1e-12);
    EXPECT_LE(std::abs(actual.vy - before.vy), limits.max_decel_y * .1 + 1e-12);
    EXPECT_LE(std::abs(actual.omega - before.omega), limits.max_alpha * .1 + 1e-12);
    EXPECT_GE(actual.vx, 0); EXPECT_GE(actual.vy, 0); EXPECT_GE(actual.omega, 0);
    distance = std::max(0.0, distance - actual.vx * .1);
    if (actual.vx == 0 && actual.vy == 0 && actual.omega == 0) {stopped = true;}
    if (stopped) {EXPECT_DOUBLE_EQ(actual.vx + actual.vy + actual.omega, 0);}
  }
  ASSERT_TRUE(stopped);
  const auto plan = planner.plan(ref, 0, .1, limits.max_xy_speed, 0, {}, actual);
  actual = limiter.limit({plan.reference_velocity, 0, 0}, actual, .1).command;
  EXPECT_GT(actual.vx, 0); EXPECT_LE(actual.vx, config.a_ref_up * .1 + 1e-12);
}

}  // namespace
}  // namespace pid_tracking_controller
