#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "execution_safety.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace pid_tracking_controller
{
namespace
{
nav2_costmap_2d::Footprint rectangle(double x = 0.3, double y = 0.2)
{
  nav2_costmap_2d::Footprint body(4);
  body[0].x = body[3].x = -x;
  body[1].x = body[2].x = x;
  body[0].y = body[1].y = -y;
  body[2].y = body[3].y = y;
  return body;
}

ExecutionSafetyOptions options()
{
  ExecutionSafetyOptions value;
  value.budget_seconds = 1.0;  // Functional tests do not depend on host scheduling.
  return value;
}

CommandSweepInput input(HolonomicBodyCommand velocity = {})
{
  return {{0.0, 0.0, 0.0}, velocity, 0.0, {0.5, 0.5, 1.0}};
}

void obstacle(nav2_costmap_2d::Costmap2D & map, double x, double y,
  unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
{
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my));
  map.setCost(mx, my, cost);
}

TEST(ExecutionSafety, stationaryBodyIncludesInteriorNotOnlyEdges)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  ExecutionSafety safety(options());
  EXPECT_EQ(safety.checkCommand(input(), map, rectangle()).status, SweepStatus::kClear);
  obstacle(map, .09, .07);
  const auto hit = safety.checkCommand(input(), map, rectangle());
  EXPECT_EQ(hit.status, SweepStatus::kCollision);
  ASSERT_TRUE(hit.collision_cell);
  EXPECT_NEAR(hit.collision_cell->x, .09, .021);
  EXPECT_NEAR(hit.distance, 0.0, 1e-12);
  EXPECT_NEAR(hit.time, 0.0, 1e-12);
}

TEST(ExecutionSafety, brakingIncludesHoldDistanceAndFullStop)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto request = input({1.0, 0.0, 0.0});
  request.hold_time = .3;
  ExecutionSafety safety(options());
  const auto clear = safety.checkCommand(request, map, rectangle());
  ASSERT_EQ(clear.status, SweepStatus::kClear);
  // 0.3 * 1 + 1^2 / (2 * 0.5) = 1.3 m; braking takes 2 s.
  EXPECT_NEAR(clear.pose.x, 1.3, 1e-9);
  EXPECT_NEAR(clear.pose.y, 0.0, 1e-9);
  EXPECT_NEAR(clear.distance, 1.3, 1e-9);
  EXPECT_NEAR(clear.time, 2.3, 1e-9);
  obstacle(map, 1.53, .0);  // Only reached by the body late in braking.
  const auto hit = safety.checkCommand(request, map, rectangle());
  EXPECT_EQ(hit.status, SweepStatus::kCollision);
  EXPECT_EQ(hit.phase, SweepPhase::kBraking);
  EXPECT_GT(hit.distance, 1.0);
  EXPECT_LT(hit.distance, 1.31);
}

TEST(ExecutionSafety, holdCanCollideBeforeBrakingBegins)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto request = input({1., 0., 0.});
  request.hold_time = 1.;
  obstacle(map, .85, 0.);
  ExecutionSafety safety(options());
  const auto hit = safety.checkCommand(request, map, rectangle());
  EXPECT_EQ(hit.status, SweepStatus::kCollision);
  EXPECT_EQ(hit.phase, SweepPhase::kHold);
  EXPECT_GT(hit.time, .4);
  EXPECT_LT(hit.time, .7);
}

TEST(ExecutionSafety, lateralReverseAndMixedTranslationAreChecked)
{
  for (const auto & velocity : {HolonomicBodyCommand{0., 1., 0.},
    HolonomicBodyCommand{-1., 0., 0.}, HolonomicBodyCommand{.7, -.7, 0.}})
  {
    nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
    obstacle(map, velocity.vx * .6, velocity.vy * .6);
    ExecutionSafety safety(options());
    const auto result = safety.checkCommand(input(velocity), map, rectangle());
    EXPECT_EQ(result.status, SweepStatus::kCollision);
    EXPECT_GT(result.time, 0.0);
  }
}

TEST(ExecutionSafety, bodyVelocityIsTransformedByActualYaw)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  obstacle(map, 0., .7);
  auto request = input({1., 0., 0.});
  request.pose.yaw = M_PI / 2;
  ExecutionSafety safety(options());
  EXPECT_EQ(safety.checkCommand(request, map, rectangle()).status, SweepStatus::kCollision);
}

TEST(ExecutionSafety, inPlaceRotationCoversCornersBetweenClearEndpoints)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  const auto body = rectangle(.6, .1);
  obstacle(map, .42, .42);
  auto request = input({0., 0., 1.});
  request.deceleration.omega = 1. / M_PI;  // Ends at pi/2 with unchanged center.
  ExecutionSafety safety(options());
  EXPECT_EQ(safety.checkCommand(input(), map, body).status, SweepStatus::kClear);
  auto final_pose = input();
  final_pose.pose.yaw = M_PI / 2;
  EXPECT_EQ(safety.checkCommand(final_pose, map, body).status, SweepStatus::kClear);
  const auto hit = safety.checkCommand(request, map, body);
  EXPECT_EQ(hit.status, SweepStatus::kCollision);
  EXPECT_NEAR(hit.distance, 0., 1e-12);
  EXPECT_GT(hit.pose.yaw, .3);
  EXPECT_LT(hit.pose.yaw, 1.3);
}

TEST(ExecutionSafety, simultaneousTranslationAndRotationFollowCurvedMotion)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto request = input({1., 0., 1.});
  request.hold_time = M_PI / 2;
  request.deceleration = {1., 1., 1.};
  ExecutionSafety safety(options());
  // During the hold the exact center is (sin(t), 1-cos(t)).
  obstacle(map, .84, .46);
  EXPECT_EQ(safety.checkCommand(request, map, rectangle(.08, .08)).status,
    SweepStatus::kCollision);
}

TEST(ExecutionSafety, coupledBrakingMatchesAnalyticArcAndChecksItsTail)
{
  // All axes scale by (1 - t/2) during braking, so the path is the exact
  // constant-twist arc evaluated at virtual time 0.3 + integral_0^2(1-t/2) = 1.3.
  struct Case {double omega, x, y, obstacle_x, obstacle_y;};
  for (const auto & expected : {
    Case{1.2, .996362441134, .326155348551, .764195749865, .058461821589},
    Case{-1.2, .336893185839, -.992783162038, .411747992648, -.646433692846}})
  {
    nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
    auto request = input({.8, -.4, expected.omega});
    request.hold_time = .3;
    request.deceleration = {.4, .2, .6};
    ExecutionSafety safety(options());
    const auto clear = safety.checkCommand(request, map, rectangle(.08, .05));
    ASSERT_EQ(clear.status, SweepStatus::kClear);
    EXPECT_NEAR(clear.pose.x, expected.x, .001);
    EXPECT_NEAR(clear.pose.y, expected.y, .001);
    EXPECT_NEAR(clear.pose.yaw, 1.3 * expected.omega, 1e-9);
    EXPECT_NEAR(clear.time, 2.3, 1e-9);
    obstacle(map, expected.obstacle_x, expected.obstacle_y);
    const auto hit = safety.checkCommand(request, map, rectangle(.08, .05));
    EXPECT_EQ(hit.status, SweepStatus::kCollision);
    EXPECT_EQ(hit.phase, SweepPhase::kBraking);
    EXPECT_GT(hit.time, request.hold_time);
  }
}

TEST(ExecutionSafety, narrowPassageDoesNotTreatSamplingEnvelopeAsBodyCollision)
{
  // 10 mm side clearance. Old isotropic 25 mm sampling padding blocks a straight
  // passage even though the complete body and braking path remain inside it.
  nav2_costmap_2d::Costmap2D map(400, 200, .005, -1., -.5, 0);
  for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
    unsigned int mx, my;
    ASSERT_TRUE(map.worldToMap(-.99, .11, mx, my));
    map.setCost(x, my, 254);
  }
  auto request = input({.4, 0., 0.}); request.hold_time = .2;
  const auto result = ExecutionSafety(options()).checkCommand(request, map, rectangle(.1, .1));
  EXPECT_EQ(result.status, SweepStatus::kClear);
  EXPECT_FALSE(result.collision_cell);
  EXPECT_NEAR(result.pose.x, .24, 1e-9);
  obstacle(map, .2, 0.);
  EXPECT_EQ(ExecutionSafety(options()).checkCommand(request, map, rectangle(.1, .1)).status,
    SweepStatus::kCollision);
}

TEST(ExecutionSafety, thinObstacleCannotHideBetweenSamples)
{
  nav2_costmap_2d::Costmap2D map(400, 200, .005, -1, -.5, 0);
  auto opts = options();
  opts.step = .4;  // Deliberately coarse: endpoints do not cover this cell.
  auto request = input({1., 0., 0.});
  request.hold_time = .4;
  request.deceleration.vx = 10.;
  obstacle(map, .20, .0);
  ExecutionSafety safety(opts);
  EXPECT_EQ(safety.checkCommand(request, map, rectangle(.01, .01)).status,
    SweepStatus::kCollision);
}

TEST(ExecutionSafety, unknownAndMapBoundaryCannotBeReportedClear)
{
  ExecutionSafety safety(options());
  nav2_costmap_2d::Costmap2D map(100, 100, .02, -1, -1, 0);
  obstacle(map, .08, .06, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(safety.checkCommand(input(), map, rectangle()).status, SweepStatus::kUnavailable);
  map.resetMap(0, 0, 100, 100);
  auto request = input({1., 0., 0.});
  EXPECT_EQ(safety.checkCommand(request, map, rectangle()).status, SweepStatus::kUnavailable);
  request = input();
  request.pose.x = .9;
  EXPECT_EQ(safety.checkCommand(request, map, rectangle()).status, SweepStatus::kUnavailable);
}

TEST(ExecutionSafety, workBudgetsDoNotAuthorizeUncheckedMotion)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto opts = options();
  opts.max_samples = 1;
  ExecutionSafety samples(opts);
  EXPECT_EQ(samples.checkCommand(input({1., 0., 0.}), map, rectangle()).status,
    SweepStatus::kUnavailable);
  opts = options();
  opts.max_cell_checks = 1;
  ExecutionSafety cells(opts);
  EXPECT_EQ(cells.checkCommand(input(), map, rectangle()).status, SweepStatus::kUnavailable);
  opts = options();
  opts.budget_seconds = std::numeric_limits<double>::min();
  ExecutionSafety time(opts);
  EXPECT_EQ(time.checkCommand(input(), map, rectangle()).status, SweepStatus::kUnavailable);
}

TEST(ExecutionSafety, invalidMotionAndGeometryFailClosed)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  ExecutionSafety safety(options());
  for (int field = 0; field < 5; ++field) {
    auto request = input({1., 0., 0.});
    if (field == 0) {request.pose.yaw = std::numeric_limits<double>::quiet_NaN();}
    if (field == 1) {request.velocity.vy = std::numeric_limits<double>::infinity();}
    if (field == 2) {request.deceleration.vx = 0.;}
    if (field == 3) {request.deceleration.omega = -1.;}
    if (field == 4) {request.hold_time = -1.;}
    EXPECT_EQ(safety.checkCommand(request, map, rectangle()).status, SweepStatus::kUnavailable);
  }
  EXPECT_EQ(safety.checkCommand(input(), map, {}).status, SweepStatus::kUnavailable);
  auto body = rectangle();
  body[1].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(safety.checkCommand(input(), map, body).status, SweepStatus::kUnavailable);
  nav2_costmap_2d::Costmap2D empty;
  EXPECT_EQ(safety.checkCommand(input(), empty, rectangle()).status, SweepStatus::kUnavailable);
}

TEST(ExecutionSafety, extremeMotionCannotOverflowIntoClear)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto request = input({std::numeric_limits<double>::max(), 0., 0.});
  request.deceleration.vx = std::numeric_limits<double>::min();
  ExecutionSafety safety(options());
  EXPECT_EQ(safety.checkCommand(request, map, rectangle()).status, SweepStatus::kUnavailable);
}

TEST(ExecutionSafety, independentAxesStopWithoutReversal)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto request = input({-.8, .3, 0.});
  request.deceleration = {.4, .6, 1.};
  ExecutionSafety safety(options());
  const auto result = safety.checkCommand(request, map, rectangle());
  ASSERT_EQ(result.status, SweepStatus::kClear);
  EXPECT_NEAR(result.pose.x, -.8, 1e-9);
  EXPECT_NEAR(result.pose.y, .075, 1e-9);
  EXPECT_NEAR(result.time, 2., 1e-9);
}

MotionTransitionInput transition(HolonomicBodyCommand measured = {1., 0., 0.},
  HolonomicBodyCommand candidate = {0., 1., 0.})
{
  MotionTransitionInput in;
  in.measured = measured;
  in.candidate = candidate;
  in.candidate_execution = {0., 0.};
  in.candidate_duration = 1.;
  in.acceleration = in.deceleration = in.braking_deceleration = {1., 1., 1.};
  return in;
}

TEST(ExecutionSafety, individuallySafeCommandsCanCollideDuringTheirTransition)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .01, -2, -2, 0);
  ExecutionSafety safety(options());
  const auto body = rectangle(.015, .015);
  obstacle(map, .375, .125);  // At t=0.5 on x=t-t^2/2, y=t^2/2.
  auto old_command = input({1., 0., 0.});
  old_command.deceleration = {1., 1., 1.};
  auto new_command = old_command;
  new_command.velocity = {0., 1., 0.};
  ASSERT_EQ(safety.checkCommand(old_command, map, body).status, SweepStatus::kClear);
  ASSERT_EQ(safety.checkCommand(new_command, map, body).status, SweepStatus::kClear);
  const auto result = safety.checkTransition({0., 0., 0.}, transition(), map, body);
  EXPECT_EQ(result.status, SweepStatus::kCollision);
  EXPECT_EQ(result.phase, SweepPhase::kTransition);
  EXPECT_GT(result.time, .2);
  EXPECT_LT(result.time, .6);
}

TEST(ExecutionSafety, transitionIntegratesBothAxesAndTheirBrakingInBodyFrame)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  ExecutionSafety safety(options());
  for (double yaw : {0., M_PI / 2}) {
    const auto result = safety.checkTransition({0., 0., yaw}, transition(), map, rectangle());
    ASSERT_EQ(result.status, SweepStatus::kClear) << result.reason;
    EXPECT_NEAR(result.pose.x, .5 * std::cos(yaw) - std::sin(yaw), 1e-9);
    EXPECT_NEAR(result.pose.y, .5 * std::sin(yaw) + std::cos(yaw), 1e-9);
    EXPECT_NEAR(result.time, 2., 1e-9);
  }
}

TEST(ExecutionSafety, rotationCandidateRetainsResidualTranslation)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  auto opts = options();
  opts.step = .01;
  ExecutionSafety safety(opts);
  const auto in = transition({1., 0., 0.}, {0., 0., 1.});
  const auto body = rectangle(.02, .02);
  const auto result = safety.checkTransition({0., 0., 0.}, in, map, body);
  ASSERT_EQ(result.status, SweepStatus::kClear) << result.reason;
  // Independent Simpson quadrature of analytic v=1-t and theta=t^2/2.
  // No motion-model segments or production integrator are used as an oracle.
  double x = 0., y = 0.;
  constexpr int n = 10000;
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double weight = i == 0 || i == n ? 1. : (i % 2 ? 4. : 2.);
    x += weight * (1 - t) * std::cos(t * t / 2) / (3 * n);
    y += weight * (1 - t) * std::sin(t * t / 2) / (3 * n);
  }
  EXPECT_NEAR(result.pose.x, x, .0001);
  EXPECT_NEAR(result.pose.y, y, .0001);
  EXPECT_NEAR(result.pose.yaw, 1., 1e-9);  // +0.5 accelerating, +0.5 braking.
  EXPECT_NEAR(result.time, 2., 1e-9);
  obstacle(map, x, y);
  EXPECT_EQ(safety.checkTransition({0., 0., 0.}, in, map, body).status,
    SweepStatus::kCollision);
}

TEST(ExecutionSafety, issuedOldCommandAndFinalBrakingRemainInSweep)
{
  const auto body = rectangle(.015, .015);
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  ExecutionSafety safety(options());
  auto in = transition({1., 0., 0.}, {});
  in.candidate_execution = {.5, .5};
  in.active_command = in.measured;
  in.active_valid_for = 1.;
  in.candidate_duration = 0.;
  obstacle(map, .3, 0.);
  const auto delayed = safety.checkTransition({0., 0., 0.}, in, map, body);
  EXPECT_EQ(delayed.status, SweepStatus::kCollision);
  EXPECT_EQ(delayed.phase, SweepPhase::kDelay);
  map.resetMap(0, 0, 400, 400);
  obstacle(map, .9, 0.);
  const auto braking = safety.checkTransition({0., 0., 0.}, in, map, body);
  EXPECT_EQ(braking.status, SweepStatus::kCollision);
  EXPECT_EQ(braking.phase, SweepPhase::kBraking);
}

TEST(ExecutionSafety, acceleratingFromRestCannotSkipThinObstacle)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .01, -2, -2, 0);
  auto opts = options();
  opts.step = .5;
  ExecutionSafety safety(opts);
  obstacle(map, .125, 0.);
  const auto result = safety.checkTransition({0., 0., 0.}, transition({}, {1., 0., 0.}),
    map, rectangle(.01, .01));
  EXPECT_EQ(result.status, SweepStatus::kCollision);
  EXPECT_EQ(result.phase, SweepPhase::kTransition);
}

TEST(ExecutionSafety, longLegacyBrakingStillFitsDefaultSampleBudget)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .1, -10, -10, 0);
  ExecutionSafety safety(options());
  auto in = input({.8, 0., 0.});
  in.deceleration.vx = .05;
  const auto result = safety.checkCommand(in, map, rectangle(.01, .01));
  ASSERT_EQ(result.status, SweepStatus::kClear) << result.reason;
  EXPECT_NEAR(result.pose.x, 6.4, 1e-9);
  EXPECT_NEAR(result.time, 16., 1e-9);
}

TEST(ExecutionSafety, transitionBudgetsAreSharedAcrossAllPhases)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .02, -4, -4, 0);
  const auto body = rectangle(.01, .01);
  for (int budget = 0; budget < 3; ++budget) {
    auto opts = options();
    opts.step = .25;
    if (budget == 0) {opts.max_samples = 6;}
    if (budget == 1) {opts.max_cell_checks = 1;}
    if (budget == 2) {opts.budget_seconds = std::numeric_limits<double>::min();}
    ExecutionSafety safety(opts);
    const auto result = safety.checkTransition({0., 0., 0.}, transition(), map, body);
    EXPECT_EQ(result.status, SweepStatus::kUnavailable) << budget;
    EXPECT_STREQ(result.reason, "evaluation_budget");
  }
}

TEST(ExecutionSafety, transitionUnknownSpaceMapExitAndTimingFailClosed)
{
  nav2_costmap_2d::Costmap2D map(100, 100, .02, -1, -1, 0);
  ExecutionSafety safety(options());
  const auto body = rectangle(.015, .015);
  obstacle(map, .375, .125, nav2_costmap_2d::NO_INFORMATION);
  const auto unknown = safety.checkTransition({0., 0., 0.}, transition(), map, body);
  EXPECT_EQ(unknown.status, SweepStatus::kUnavailable);
  EXPECT_STREQ(unknown.reason, "unknown_space");
  map.resetMap(0, 0, 100, 100);
  const auto outside = safety.checkTransition({0., 0., 0.}, transition(), map, body);
  EXPECT_EQ(outside.status, SweepStatus::kUnavailable);
  EXPECT_STREQ(outside.reason, "outside_map");
  auto in = transition();
  in.candidate_execution = {0., .1};
  const auto uncertain = safety.checkTransition({0., 0., 0.}, in, map, body);
  EXPECT_EQ(uncertain.status, SweepStatus::kUnavailable);
  EXPECT_STREQ(uncertain.reason, "execution_timing_unknown");
}
}  // namespace
}  // namespace pid_tracking_controller
