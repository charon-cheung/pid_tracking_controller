#include <chrono>
#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "execution_safety_gate.hpp"

namespace pid_tracking_controller
{
namespace
{
using Wall = MotionObservation::Wall;
NominalTransitionAssumptions nominal()
{return {.1, .1, {1., 1., 1.}, {1., 1., 1.}, {1., 1., 1.}};}
HolonomicCommandLimiterConfig limits()
{
  HolonomicCommandLimiterConfig c;
  c.max_vx = c.max_vy = c.max_xy_speed = c.max_omega = 1.;
  c.max_ax = c.max_ay = 1.; c.max_decel_x = c.max_decel_y = .5; c.max_alpha = 1.;
  return c;
}
class GateTest : public ::testing::Test
{
protected:
  GateTest() : gate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}), limiter(limits())
  {
    SafetyInputSnapshot s;
    s.map = std::make_unique<nav2_costmap_2d::Costmap2D>(200, 200, .05, -5., -5., 0);
    s.pose = {0., 0., 0.}; s.header.frame_id = "map";
    for (auto xy : {std::pair<double, double>{-.1, -.1}, {.1, -.1}, {.1, .1}, {-.1, .1}}) {
      geometry_msgs::msg::Point p; p.x = xy.first; p.y = xy.second; s.footprint.push_back(p);
    }
    geometry.snapshot = std::move(s); geometry.reason = "ready";
    h.state = "ready"; h.epoch = 1;
  }
  SafetyGateDecision step(HolonomicBodyCommand candidate, HolonomicBodyCommand measured = {},
    double dt = .1, Wall::time_point started = Wall::now())
  {
    now += 100000000LL;
    if (refresh_risk) {risk.evidence_stamp_ns = risk.footprint_origin_ns = now;}
    auto & series = h.odometry; series = {};
    auto & s = series.samples[0];
    s.vx = measured.vx; s.vy = measured.vy; s.omega = measured.omega;
    s.stamp_ns = now; s.received = Wall::now(); s.count = 1;
    s.payload = "valid"; s.frame = "base"; series.size = series.next = 1;
    return gate.apply(geometry, h, now, now, revision, candidate, measured,
      dt, limiter.previousCommand(), limiter, started, risk);
  }
  void settle()
  {for (int i = 0; i < 8; ++i) {step({});}}
  bool refresh_risk{true};
  void red(double distance)
  {
    risk.state = msg::PathRiskState::COLLISION_AHEAD;
    risk.distance = distance; risk.pose.x = distance; risk.frame = "map";
  }
  void wall(double x)
  {
    auto & map = *geometry.snapshot->map;
    unsigned int mx, my;
    ASSERT_TRUE(map.worldToMap(x, 0., mx, my));
    for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {map.setCost(mx, y, 254);}
  }
  PathRiskTrigger risk{msg::PathRiskState::CLEARANCE_LOW, "ready"};
  ExecutionSafetyGate gate;
  HolonomicCommandLimiter limiter;
  SafetyInputResult geometry;
  MotionObservation::History h;
  int64_t now{1000000000LL};
  uint64_t revision{1};
};

TEST_F(GateTest, StopReserveRequiresConfirmedActualFootprintAndHandsOffAfterSettling)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2}, false, .2);
  red(.3);
  auto d = step({.3,0.,0.}, {.3,0.,0.});
  ASSERT_FALSE(d.inhibited);  // Conservative red alone does not use the extra reserve.
  risk.footprint_collision = true; risk.footprint_distance = .3; risk.footprint_pose.x = .3;
  d = step({.3,0.,0.}, {.3,0.,0.});
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.reason, "path_replan_reserve");
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  settle(); d = step({});
  EXPECT_TRUE(d.stopped_for_handoff);
}
TEST_F(GateTest, StopReserveUsesPathDistanceOnBendsAndAccountsForEvidenceAge)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2}, false, .2);
  red(1.); risk.footprint_collision = true;
  risk.footprint_distance = 1.; risk.footprint_pose.x = .1;
  auto d = step({.3,0.,0.}, {.3,0.,0.});
  EXPECT_FALSE(d.inhibited); EXPECT_NEAR(d.footprint_distance, .95, 1e-9);
  risk.footprint_distance = .45; refresh_risk = false;
  risk.footprint_origin_ns = now - 200000000LL;
  d = step({.3,0.,0.}, {.3,0.,0.});
  EXPECT_LT(d.footprint_distance, .45 - .05 - .09);
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.reason, "path_replan_reserve");
}

TEST_F(GateTest, StopReserveDoesNotStopYellowOrDistantConfirmedObstacle)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2}, false, .2);
  EXPECT_FALSE(step({.3,0.,0.}, {.3,0.,0.}).inhibited);
  red(1.);
  risk.footprint_collision = true; risk.footprint_distance = 1.; risk.footprint_pose.x = 1.;
  EXPECT_FALSE(step({.3,0.,0.}, {.3,0.,0.}).inhibited);
}

TEST_F(GateTest, PersistentNearbyYellowKeepsSafeMotionWithoutReplanning)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2}, false, .5);
  risk.frame = "map"; risk.distance = .01;
  for (int i = 0; i < 20; ++i) {
    const auto d = step({.3,0.,0.}, {.3,0.,0.});
    EXPECT_EQ(d.candidate.status, SweepStatus::kClear);
    EXPECT_FALSE(d.inhibited);
    EXPECT_FALSE(d.stopped_for_handoff);
    EXPECT_DOUBLE_EQ(d.command.vx, .3);
  }
}

TEST_F(GateTest, PersistentYellowAllowsSafeRotationAndLateralTranslation)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2}, false, .5);
  risk.frame = "map"; risk.distance = 0.;
  for (const auto & command : {HolonomicBodyCommand{0.,0.,.1}, {0.,.1,0.}}) {
    for (int i = 0; i < 10; ++i) {
      const auto d = step(command, command);
      EXPECT_EQ(d.candidate.status, SweepStatus::kClear);
      EXPECT_FALSE(d.inhibited);
      EXPECT_FALSE(d.stopped_for_handoff);
      EXPECT_DOUBLE_EQ(d.command.vy, command.vy);
      EXPECT_DOUBLE_EQ(d.command.omega, command.omega);
    }
  }
}

TEST_F(GateTest, YellowStillChecksActualCollisionAndRedStillStopsAtBrakingBoundary)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2}, false, .5);
  risk.frame = "map"; risk.distance = .4;
  wall(.2);
  auto d = step({.4,0.,0.}, {.4,0.,0.});
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.candidate.status, SweepStatus::kCollision);
  geometry.snapshot->map->resetMap(0,0,200,200);
  gate.reset(); limiter = HolonomicCommandLimiter(limits()); red(.05);
  d = step({.3,0.,0.}, {.3,0.,0.});
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.reason, "path_braking_boundary");
}

TEST_F(GateTest, ObstacleStopSurvivesActionResetAndRequiresFreshRestartEvidence)
{
  red(.15);
  auto d = step({.3, 0., 0.}, {.3, 0., 0.});
  ASSERT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  settle();
  gate.reset(true); ++revision;
  risk.state = msg::PathRiskState::CLEAR;
  // A new action and CLEAR cannot clear an earlier obstacle stop.
  d = step({.03, 0., 0.});
  EXPECT_TRUE(d.inhibited); EXPECT_FALSE(d.stopped);
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  settle();
  d = step({});
  EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped);
  EXPECT_FALSE(d.stopped_for_handoff);  // Waiting for a candidate is not current obstacle evidence.
  for (int i = 0; i < 2; ++i) {
    d = step({.03, 0., 0.});
    EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped);
    EXPECT_EQ(d.resume.status, SweepStatus::kClear);
    EXPECT_FALSE(d.stopped_for_handoff);  // BT must let safe restart confirmation finish.
  }
  d = step({.03, 0., 0.});
  EXPECT_FALSE(d.inhibited); EXPECT_GT(d.command.vx, 0.);
  EXPECT_FALSE(d.stopped_for_handoff);
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_NONE);
}

TEST_F(GateTest, InheritedStopAllowsFreshSafeRestartWithDistantRed)
{
  for (bool simulation : {false, true}) {
    gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2},
      simulation, .5);
    limiter = HolonomicCommandLimiter(limits());
    risk.footprint_collision = false; red(.05);
    ASSERT_TRUE(step({.3,0.,0.}, {.3,0.,0.}).inhibited);
    settle();
    gate.reset(true); ++revision;
    // Latest recorded case: stopped at the path start, red about 1.6 m ahead.
    red(1.6); risk.footprint_collision = true;
    risk.footprint_distance = 1.9; risk.footprint_pose.x = 1.9;
    auto d = step({0.,0.,.056});
    EXPECT_TRUE(d.inhibited); EXPECT_FALSE(d.stopped_for_handoff);
    settle();
    for (int i = 0; i < 3; ++i) {
      d = step({0.,0.,.056});
      EXPECT_EQ(d.resume.status, SweepStatus::kClear);
      EXPECT_FALSE(d.stopped_for_handoff);
      EXPECT_EQ(d.inhibited, i < 2);
    }
    EXPECT_GT(d.command.omega, 0.);
    EXPECT_FALSE(step({.03,0.,0.}).inhibited);
  }
}

TEST_F(GateTest, NewActionStillStopsForCurrentCollisionOrNearRedReserve)
{
  for (bool collision : {false, true}) {
    gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025,512,500000,.2},
      true, .5);
    limiter = HolonomicCommandLimiter(limits());
    geometry.snapshot->map->resetMap(0,0,200,200);
    risk.footprint_collision = false; red(.05);
    ASSERT_TRUE(step({.3,0.,0.}, {.3,0.,0.}).inhibited);
    settle(); gate.reset(true); ++revision;
    red(1.6); risk.footprint_collision = true;
    risk.footprint_distance = collision ? 1.9 : .3;
    risk.footprint_pose.x = risk.footprint_distance;
    if (collision) {wall(0.);}
    const auto d = step({0.,0.,.056});
    EXPECT_TRUE(d.inhibited);
    EXPECT_EQ(d.candidate.status, collision ? SweepStatus::kCollision : SweepStatus::kClear);
    if (!collision) {EXPECT_EQ(d.reason, "path_replan_reserve");}
    settle(); EXPECT_TRUE(step({}).stopped_for_handoff);
  }
}

TEST_F(GateTest, MissingInputsNeverBecomeObstacleAndDoNotEraseExistingObstacle)
{
  risk = {};
  auto d = step({.3, 0., 0.});
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_INPUT_UNAVAILABLE);
  EXPECT_FALSE(d.stopped_for_handoff);
  gate.reset(true); risk.state = msg::PathRiskState::CLEAR;
  EXPECT_FALSE(step({.03, 0., 0.}).inhibited);
  red(.05);  // No remaining approach distance, including after a low issued command.
  ASSERT_EQ(step({.3, 0., 0.}, {.3, 0., 0.}).stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  settle();
  EXPECT_TRUE(step({}).stopped_for_handoff);
  risk = {};
  d = step({});
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  EXPECT_FALSE(d.stopped_for_handoff);
  gate.reset(); risk.state = msg::PathRiskState::CLEAR;
  EXPECT_FALSE(step({.03, 0., 0.}).inhibited);
}

TEST_F(GateTest, CurrentBodyCollisionStillAllowsStoppedHandoffOnYellow)
{
  wall(0.);
  ASSERT_TRUE(step({.03, 0., 0.}).inhibited);
  settle();
  const auto d = step({.03, 0., 0.});
  EXPECT_EQ(d.candidate.status, SweepStatus::kCollision);
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  EXPECT_TRUE(d.stopped_for_handoff);
}

TEST_F(GateTest, ClearAndNewPathCannotReleaseOngoingBraking)
{
  limiter.seed({.4, 0., 0.}); wall(.30);
  ASSERT_TRUE(step({.5, 0., 0.}, {.4, 0., 0.}).inhibited);
  geometry.snapshot->map->resetMap(0, 0, 200, 200);
  risk.state = msg::PathRiskState::CLEAR;
  ++revision;
  const auto result = step({.5, 0., 0.}, {.4, 0., 0.});
  EXPECT_TRUE(result.inhibited);
  EXPECT_LT(result.command.vx, .4);
}

TEST_F(GateTest, PathClearSkipsSweepOnlyWhileTracking)
{
  wall(.30);
  risk.state = msg::PathRiskState::CLEAR;
  const auto bypass = step({.5, -.2, .1}, {.4, 0., 0.});
  EXPECT_FALSE(bypass.inhibited); EXPECT_FALSE(bypass.sweep_requested);
  EXPECT_STREQ(bypass.action, "path_clear_bypass");
  EXPECT_STREQ(bypass.candidate.reason, "not_checked_path_clear");
  risk.state = msg::PathRiskState::CLEARANCE_LOW;
  EXPECT_TRUE(step({.5, 0., 0.}, {.4, 0., 0.}).inhibited);
}

TEST_F(GateTest, FarRedAllowsApproachNearRedStopsAndCannotPulseAtTinySpeed)
{
  red(2.);
  EXPECT_FALSE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  red(.15);
  const auto boundary = step({.3, 0., 0.}, {.3, 0., 0.});
  ASSERT_TRUE(boundary.inhibited);
  EXPECT_EQ(boundary.candidate.status, SweepStatus::kClear);
  EXPECT_EQ(boundary.reason, "path_braking_boundary");
  for (int i = 0; i < 50; ++i) {
    const auto d = step({.005, 0., 0.}, *limiter.previousCommand());
    EXPECT_TRUE(d.inhibited);
  }
  EXPECT_DOUBLE_EQ(limiter.previousCommand()->vx, 0.);
  EXPECT_TRUE(step({}).stopped_for_handoff);  // Persistent red still hands off.
  // Once the red boundary caused a stop, a low-speed candidate cannot reopen it.
  risk.state = msg::PathRiskState::CLEARANCE_LOW;
  for (int i = 0; i < 3; ++i) {
    const auto d = step({.03, 0., 0.});
    EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped_for_handoff);
    EXPECT_DOUBLE_EQ(d.command.vx, 0.);
  }
}

TEST_F(GateTest, RepeatedRiskFrameCannotAccumulateRestartConfirmations)
{
  geometry.reason = "missing"; auto saved = std::move(geometry.snapshot); geometry.snapshot.reset();
  step({}); geometry.snapshot = std::move(saved); geometry.reason = "ready";
  settle(); refresh_risk = false;
  for (int i = 0; i < 8; ++i) {EXPECT_TRUE(step({.03, 0., 0.}).inhibited);}
  refresh_risk = true;
  EXPECT_TRUE(step({.03, 0., 0.}).inhibited);
  EXPECT_FALSE(step({.03, 0., 0.}).inhibited);
}

TEST_F(GateTest, YellowAloneNeverStopsSafeMotion)
{
  wall(.35);  // Adjacent wall, lateral command remains clear.
  for (int i = 0; i < 20; ++i) {
    const auto d = step({0., .1, 0.}, {0., .1, 0.});
    EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vy, .1);
  }
}

TEST_F(GateTest, NominalIssuedMotionCannotDisappearWithZeroLatestOutputAndOdometry)
{
  auto n = nominal(); n.delay = .5;
  gate = ExecutionSafetyGate(n, ExecutionSafetyOptions{.025, 512, 500000, .2});
  risk.state = msg::PathRiskState::CLEAR;
  step({.6, 0., 0.}); step({});
  geometry.snapshot->map = std::make_unique<nav2_costmap_2d::Costmap2D>(400, 400, .005, -1., -1., 0);
  wall(.105);
  risk.state = msg::PathRiskState::CLEARANCE_LOW;
  const auto d = step({});
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.candidate.status, SweepStatus::kCollision);
}

TEST_F(GateTest, NewerMovingOdometryVetoesStoppedAnchor)
{
  auto saved = std::move(geometry.snapshot); geometry.snapshot.reset(); step({});
  geometry.snapshot = std::move(saved); geometry.reason = "ready"; settle();
  auto & series = h.odometry;
  series.samples[1] = series.samples[0]; series.samples[1].stamp_ns = now + 50000000LL;
  series.samples[1].count = 2; series.samples[1].vx = .1; series.size = series.next = 2;
  const auto d = gate.apply(geometry, h, now, now + 100000000LL, revision, {.03, 0., 0.}, {},
    .1, limiter.previousCommand(), limiter, Wall::now(), risk);
  EXPECT_TRUE(d.inhibited); EXPECT_FALSE(d.stopped);
}

TEST_F(GateTest, UnavailableRiskAndMissingGeometryCannotUseClearBypass)
{
  risk.state = msg::PathRiskState::CLEAR;
  EXPECT_FALSE(step({.4, 0., 0.}).inhibited);
  risk = {};
  auto result = step({.5, 0., 0.}, {.4, 0., 0.});
  EXPECT_TRUE(result.inhibited); EXPECT_EQ(result.reason, "path_risk_missing");
  EXPECT_FALSE(result.sweep_requested); EXPECT_LT(result.command.vx, .4);
  risk = {msg::PathRiskState::CLEAR, "ready"};
  geometry.snapshot.reset(); geometry.reason = "map_missing_or_stale";
  result = step({.5, 0., 0.}, {.4, 0., 0.});
  EXPECT_TRUE(result.inhibited); EXPECT_EQ(result.reason, "map_missing_or_stale");
  EXPECT_STREQ(result.candidate.reason, "inputs_unavailable");
}

TEST(PathRiskTrigger, OnlyFreshMatchingLoadedPathEvidenceCanSkipOrRequestTheSweep)
{
  using Risk = msg::PathRiskState;
  const int64_t now = 2000000000LL;
  const auto wall = Wall::now();
  msg::TrackingProgress current;
  current.valid = true; current.controller_instance = "controller";
  current.plan_revision = 7; current.path_fingerprint = 42;
  current.path_header.frame_id = "map";
  current.header.stamp = rclcpp::Time(now); current.loaded_at = rclcpp::Time(now - 200000000LL);
  Risk valid;
  valid.header = current.header; valid.header.frame_id = "odom";  // Observer may transform the path.
  valid.path_header = current.path_header; valid.controller_instance = current.controller_instance;
  valid.plan_revision = current.plan_revision; valid.path_fingerprint = current.path_fingerprint;
  valid.progress_stamp = rclcpp::Time(now - 50000000LL);
  valid.map_stamp = rclcpp::Time(now - 100000000LL); valid.state = Risk::CLEAR;
  const auto check = [&](const Risk & value) {return validatePathRisk(&value, wall, current, now, wall);};
  for (auto state : {Risk::CLEAR, Risk::CLEARANCE_LOW, Risk::COLLISION_AHEAD}) {
    auto value = valid; value.state = state; EXPECT_EQ(check(value).state, state);
  }
  auto first = valid; first.header.stamp = rclcpp::Time(now - 30000000LL);
  const auto source_evidence = check(first).evidence_stamp_ns;
  EXPECT_EQ(check(valid).evidence_stamp_ns, source_evidence);
  auto new_source = valid; new_source.progress_stamp = rclcpp::Time(now - 10000000LL);
  EXPECT_GT(check(new_source).evidence_stamp_ns, source_evidence);
  EXPECT_EQ(validatePathRisk(nullptr, wall, current, now, wall).state, Risk::UNAVAILABLE);
  for (int mutation = 0; mutation < 14; ++mutation) {
    SCOPED_TRACE(mutation);
    auto value = valid;
    switch (mutation) {
      case 0: value.controller_instance = "old"; break;
      case 1: --value.plan_revision; break;
      case 2: ++value.path_fingerprint; break;
      case 3: value.path_header.frame_id = "old_map"; break;
      case 4: value.path_header.stamp = rclcpp::Time(1); break;
      case 5: value.header.stamp = rclcpp::Time(now - 301000000LL); break;
      case 6: value.header.stamp = rclcpp::Time(now + 1); break;
      case 7: value.progress_stamp = rclcpp::Time(now - 301000000LL); break;
      case 8: value.progress_stamp = rclcpp::Time(now + 1); break;
      case 9: value.map_stamp = rclcpp::Time(now - 601000000LL); break;
      case 10: value.map_stamp = rclcpp::Time(now + 1); break;
      case 11: value.state = Risk::UNAVAILABLE; break;
      case 12: value.state = 99; break;
      case 13: value.progress_stamp = rclcpp::Time(now - 201000000LL); break;
    }
    EXPECT_EQ(check(value).state, Risk::UNAVAILABLE);
  }
  EXPECT_EQ(validatePathRisk(&valid, wall - std::chrono::milliseconds(301), current, now, wall).state,
    Risk::UNAVAILABLE);
  current.valid = false; EXPECT_EQ(check(valid).state, Risk::UNAVAILABLE);
  current.valid = true; current.loaded_at = rclcpp::Time(now - 10000000LL);
  EXPECT_EQ(check(valid).state, Risk::UNAVAILABLE);
}

TEST_F(GateTest, ClearCurrentCandidatePassesAllAxesWithoutModification)
{
  for (const auto c : {HolonomicBodyCommand{.4, -.2, .1}, {-.3, .2, -.1}, {0., 0., .4}}) {
    auto result = step(c, {.1, .1, .1});
    EXPECT_EQ(result.candidate.status, SweepStatus::kClear);
    EXPECT_FALSE(result.inhibited);
    EXPECT_DOUBLE_EQ(result.command.vx, c.vx); EXPECT_DOUBLE_EQ(result.command.vy, c.vy);
    EXPECT_DOUBLE_EQ(result.command.omega, c.omega);
    ASSERT_TRUE(limiter.previousCommand());
    EXPECT_DOUBLE_EQ(limiter.previousCommand()->vy, c.vy);
  }
}

TEST_F(GateTest, CollisionBrakesThenHoldsUntilCurrentRestartIsClear)
{
  limiter.seed({.4, 0., 0.}); wall(.30);
  auto first = step({.5, 0., 0.}, {.4, 0., 0.});
  ASSERT_EQ(first.candidate.status, SweepStatus::kCollision);
  ASSERT_STREQ(first.action, "braking");
  EXPECT_NEAR(first.command.vx, .35, 1e-12);
  EXPECT_EQ(first.output.status, SweepStatus::kClear);
  for (int i = 0; i < 20; ++i) {
    const auto previous = *limiter.previousCommand();
    const auto result = step({.5, 0., 0.}, previous);
    EXPECT_TRUE(result.inhibited);
    EXPECT_LE(result.command.vx, previous.vx);
  }
  EXPECT_NEAR(limiter.previousCommand()->vx, 0., 1e-12);
  // A zero or temporarily safe low-speed candidate must hand off after the stop.
  const auto stopped = step({});
  EXPECT_TRUE(stopped.inhibited); EXPECT_TRUE(stopped.stopped_for_handoff);
  geometry.snapshot->map->resetMap(0, 0, 200, 200);
  for (int i = 0; i < 3; ++i) {
    const auto handoff = step({.05, 0., 0.});
    EXPECT_TRUE(handoff.inhibited); EXPECT_TRUE(handoff.stopped_for_handoff);
    EXPECT_DOUBLE_EQ(handoff.command.vx, 0.);
  }
  // No stale clear authorization: the very next candidate sees the new obstacle.
  wall(0.);
  EXPECT_TRUE(step({.1, 0., 0.}).inhibited);
}

TEST_F(GateTest, RecoveryUsesCurrentLowSpeedAndDirectionAfterRejectedFastMotion)
{
  limiter.seed({.4, 0., 0.}); wall(.30);
  ASSERT_TRUE(step({.5, 0., 0.}, {.4, 0., 0.}).inhibited);
  // Standstill after braking. The old high-speed target still intersects the wall,
  // but a new lateral correction and low-speed restart fit in the available space.
  limiter.seed({});
  settle();
  const auto stopped = step({});
  EXPECT_TRUE(stopped.inhibited); EXPECT_TRUE(stopped.stopped_for_handoff);
  for (int i = 0; i < 3; ++i) {
    const auto handoff = step({0., .05, 0.});
    EXPECT_TRUE(handoff.inhibited); EXPECT_TRUE(handoff.stopped_for_handoff);
    EXPECT_DOUBLE_EQ(handoff.command.vx, 0.);
    EXPECT_DOUBLE_EQ(handoff.command.vy, 0.);
  }
  wall(0.);  // An obstacle at the body must still block the next command.
  EXPECT_TRUE(step({0., .05, 0.}).inhibited);
}

TEST_F(GateTest, TinyClearCandidateCannotReleaseAnUnsafeSustainedRestart)
{
  auto saved = std::move(geometry.snapshot);
  geometry.snapshot.reset(); geometry.reason = "map_missing_or_stale";
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  geometry.snapshot = std::move(saved); geometry.reason = "ready";
  geometry.snapshot->map = std::make_unique<nav2_costmap_2d::Costmap2D>(400, 400, .005, -1., -1., 0);
  wall(.115);
  settle();
  auto d = step({.08, 0., 0.});
  EXPECT_EQ(d.candidate.status, SweepStatus::kClear);
  EXPECT_EQ(d.resume.status, SweepStatus::kCollision);
  EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped_for_handoff);
  EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
  EXPECT_DOUBLE_EQ(d.command.vx, 0.);
  for (int i = 0; i < 4; ++i) {
    d = step({.08, 0., 0.});
    EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped_for_handoff);
    EXPECT_EQ(d.stop_cause, msg::TrackingProgress::SAFETY_OBSTACLE);
    EXPECT_DOUBLE_EQ(d.command.vx, 0.);
  }
}

TEST_F(GateTest, MissingInputBrakesEveryAxisAndCommitsActualOutput)
{
  limiter.seed({.4, -.2, .3}); geometry.snapshot.reset(); geometry.reason = "map_missing_or_stale";
  const auto result = step({.8, -.5, .6}, {.4, -.2, .3});
  EXPECT_TRUE(result.inhibited); EXPECT_STREQ(result.action, "braking");
  EXPECT_EQ(result.reason, "map_missing_or_stale");
  EXPECT_NEAR(result.command.vx, .35, 1e-12); EXPECT_NEAR(result.command.vy, -.15, 1e-12);
  EXPECT_NEAR(result.command.omega, .2, 1e-12);
  const auto actual = limiter.previousCommand(); ASSERT_TRUE(actual);
  EXPECT_DOUBLE_EQ(actual->vx, result.command.vx);
  EXPECT_DOUBLE_EQ(actual->vy, result.command.vy);
  EXPECT_DOUBLE_EQ(actual->omega, result.command.omega);
}

TEST_F(GateTest, UnknownHistoryInvalidDtAndBrakingCollisionRequestEmergencyZero)
{
  wall(0.);
  for (double dt : {0., -1., 1., std::numeric_limits<double>::quiet_NaN()}) {
    const auto result = step({.3, .2, .1}, {.3, .2, .1}, dt);
    EXPECT_TRUE(result.inhibited); EXPECT_STREQ(result.action, "emergency_zero");
    EXPECT_DOUBLE_EQ(result.command.vx, 0.); EXPECT_DOUBLE_EQ(result.command.vy, 0.);
    EXPECT_DOUBLE_EQ(result.command.omega, 0.);
    EXPECT_EQ(result.output.status, SweepStatus::kCollision);
  }
}

TEST_F(GateTest, BudgetUnavailableNeverReleasesOrReusesPreviousClear)
{
  EXPECT_FALSE(step({.2, 0., 0.}).inhibited);
  auto result = step({.2, 0., 0.}, {}, .1, Wall::now() - std::chrono::seconds(1));
  EXPECT_TRUE(result.inhibited); EXPECT_STREQ(result.candidate.reason, "gate_budget_exceeded");
  EXPECT_LT(result.command.vx, .2);
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
}

TEST_F(GateTest, PathRevisionAndInputGapResetResumeConfirmation)
{
  limiter.seed({}); geometry.reason = "map_missing_or_stale";
  auto saved = std::move(geometry.snapshot); geometry.snapshot.reset();
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  geometry.snapshot = std::move(saved); geometry.reason = "ready";
  settle();
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  ++revision;
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  ++h.dropped;
  const auto gap = step({.05, 0., 0.});
  EXPECT_TRUE(gap.inhibited); EXPECT_EQ(gap.reason, "odometry_history_gap");
  settle();
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  EXPECT_FALSE(step({.05, 0., 0.}).inhibited);
}

TEST_F(GateTest, RequiresExactFreshPoseOdometryAndRejectsInvalidNewerEvidence)
{
  step({.2, 0., 0.});
  auto run = [&](int64_t pose, int64_t time) {
      return gate.apply(geometry, h, pose, time, revision, {.2, 0., 0.}, {}, .1,
        limiter.previousCommand(), limiter, Wall::now(), risk);
    };
  EXPECT_TRUE(run(now - 10000000LL, now + 100000000LL).inhibited);
  auto & series = h.odometry;
  series.samples[0].received = Wall::now() - std::chrono::seconds(1);
  auto stale = run(now, now + 200000000LL);
  EXPECT_EQ(stale.reason, "odometry_receipt_stale"); EXPECT_TRUE(stale.inhibited);
  series.samples[0].received = Wall::now();
  series.samples[1] = series.samples[0]; series.samples[1].count = 2;
  series.samples[1].stamp_ns = now + 500000000LL; series.samples[1].payload = "invalid";
  series.size = series.next = 2;
  auto invalid = run(now, now + 250000000LL);
  EXPECT_EQ(invalid.reason, "odometry_invalid"); EXPECT_TRUE(invalid.inhibited);
}

TEST_F(GateTest, RealHistoryResetsCannotReuseOldClearAuthorizationWithoutLogging)
{
  MotionObservation motion("base"); motion.activate();
  now = 10000000000LL; motion.history(now, revision);
  risk.state = msg::PathRiskState::CLEAR;
  const auto record = [&](double vx = 0.) {
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(now);
      geometry_msgs::msg::Twist twist; twist.linear.x = vx;
      motion.record(header, twist, "base");
    };
  const auto run = [&]() {
      const auto history = motion.history(now, revision);
      risk.evidence_stamp_ns = now;
      return gate.apply(geometry, history, now, now, revision, {.2, 0., 0.}, {}, .1,
        limiter.previousCommand(), limiter, Wall::now(), risk);
    };
  const auto recover = [&]() {
      SafetyGateDecision d;
      for (int i = 0; i < 12; ++i) {now += 100000000LL; record(); d = run();}
      EXPECT_FALSE(d.inhibited);
    };
  record(); EXPECT_FALSE(run().inhibited);
  // A new plan discards even a just-received old-plan velocity sample.
  now += 100000000LL; record(); ++revision;
  EXPECT_TRUE(run().inhibited);
  recover();
  // Clock rollback must be fenced without any timer or logger call.
  now -= 1000000000LL;
  EXPECT_TRUE(run().inhibited);
  recover();
  // Same-stamp conflicting velocities cannot be silently resolved by picking latest.
  now += 100000000LL; record(0.); record(.3);
  const auto ambiguous = run();
  EXPECT_TRUE(ambiguous.inhibited); EXPECT_EQ(ambiguous.reason, "odometry_anchor_ambiguous");
  motion.deactivate(); motion.activate(); now += 100000000LL;
  EXPECT_TRUE(run().inhibited);
  recover();
}

TEST_F(GateTest, ValidTenMillisecondFutureOdometryDoesNotHideExactPoseAnchor)
{
  auto result = step({.2, 0., 0.}); ASSERT_FALSE(result.inhibited);
  auto & series = h.odometry;
  series.samples[1] = series.samples[0]; series.samples[1].count = 2;
  series.samples[1].stamp_ns = now + 110000000LL;
  series.size = series.next = 2;
  result = gate.apply(geometry, h, now, now + 100000000LL, revision, {.2, 0., 0.}, {}, .1,
    limiter.previousCommand(), limiter, Wall::now(), risk);
  EXPECT_FALSE(result.inhibited);
}

TEST_F(GateTest, SimulationAcceptsBoundedPoseAndOdometrySkewButStillRejectsInvalidData)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  risk.state = msg::PathRiskState::CLEAR;
  ASSERT_FALSE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  for (const int64_t skew : {-50000000LL, 10000000LL}) {
    now += 100000000LL;
    h.odometry.samples[0].stamp_ns = now + skew;
    h.odometry.samples[0].received = Wall::now();
    ++h.dropped;  // A missed callback is harmless when a recent usable sample exists.
    const auto d = gate.apply(geometry, h, now + 10000000LL, now, revision,
      {.3, 0., 0.}, {.3, 0., 0.}, .1, limiter.previousCommand(), limiter, Wall::now(), risk);
    EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, .3);
  }
  now += 100000000LL;
  h.odometry.samples[0].stamp_ns = now - 150000000LL;
  auto d = gate.apply(geometry, h, now, now, revision, {.3, 0., 0.}, {.3, 0., 0.},
    .1, limiter.previousCommand(), limiter, Wall::now(), risk);
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.reason, "odometry_anchor_missing");
  now += 100000000LL; h.odometry.samples[0].stamp_ns = now;
  h.odometry.samples[0].payload = "nonfinite_velocity";
  d = gate.apply(geometry, h, now, now, revision, {.3, 0., 0.}, {.3, 0., 0.},
    .1, limiter.previousCommand(), limiter, Wall::now(), risk);
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.reason, "odometry_invalid");
}

TEST_F(GateTest, SimulationRepeatedControlTickPreservesOutputAndStillChecksCollision)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  ASSERT_FALSE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  auto run = [&]() {
      // Matches the controller's nonpositive-dt reseed: no accepted-output argument.
      return gate.apply(geometry, h, now, now, revision, {}, {.3, 0., 0.}, 0.,
        std::nullopt, limiter, Wall::now(), risk);
    };
  auto d = run(); EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, .3);
  wall(.1);
  d = run(); EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.candidate.status, SweepStatus::kCollision);
  EXPECT_DOUBLE_EQ(d.command.vx, 0.);
}

TEST_F(GateTest, SimulationRepeatUsesTheLastSafetyOutputAfterSameStampInputRecovery)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  ASSERT_FALSE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  auto run = [&]() {
      return gate.apply(geometry, h, now, now, revision, {}, {.3, 0., 0.}, 0.,
        std::nullopt, limiter, Wall::now(), risk);
    };
  auto saved = std::move(geometry.snapshot); geometry.snapshot.reset();
  auto d = run(); ASSERT_TRUE(d.inhibited); ASSERT_DOUBLE_EQ(d.command.vx, 0.);
  geometry.snapshot = std::move(saved);
  d = run(); EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, 0.);
  EXPECT_DOUBLE_EQ(limiter.previousCommand()->vx, 0.);
  // The next advancing tick may accelerate normally, after fresh safety checks.
  d = step({.05, 0., 0.}); EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, .05);
}

TEST_F(GateTest, SimulationRepeatsDoNotEraseObstacleRestartConfirmations)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  wall(.1); ASSERT_TRUE(step({.3, 0., 0.}).inhibited);
  geometry.snapshot->map->resetMap(0, 0, 200, 200); settle();
  for (int i = 0; i < 3; ++i) {
    auto d = step({.03, 0., 0.});
    EXPECT_EQ(d.clear_cycles, 0u);
    EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped_for_handoff);
    d = gate.apply(geometry, h, now, now, revision, {}, {}, 0., std::nullopt,
      limiter, Wall::now(), risk);
    EXPECT_EQ(d.clear_cycles, 0u);
    EXPECT_TRUE(d.inhibited); EXPECT_TRUE(d.stopped_for_handoff);
    EXPECT_DOUBLE_EQ(d.command.vx, 0.);
  }
}

TEST_F(GateTest, SimulationFrozenControlClockCannotKeepPublishingOldMotion)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  risk.state = msg::PathRiskState::CLEAR;
  ASSERT_FALSE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  auto d = gate.apply(geometry, h, now, now, revision, {.3, 0., 0.}, {.3, 0., 0.},
    0., std::nullopt, limiter, Wall::now(), risk);
  EXPECT_TRUE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, 0.);
  EXPECT_EQ(d.reason, "control_time_or_revision_invalid");
}

TEST_F(GateTest, SimulationInputRecoveryDoesNotLatchFarRedButNearRedStillStops)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  red(2.3);
  ASSERT_FALSE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  auto saved = std::move(geometry.snapshot); geometry.snapshot.reset();
  geometry.reason = "observation_tf_unavailable";
  EXPECT_TRUE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  geometry.snapshot = std::move(saved); geometry.reason = "ready";
  auto d = step({.3, 0., 0.}, {.25, 0., 0.});
  EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, .3);
  EXPECT_STREQ(d.action, "inputs_recovered");
  red(.1);
  ASSERT_TRUE(step({.3, 0., 0.}, {.3, 0., 0.}).inhibited);
  for (int i = 0; i < 20; ++i) {EXPECT_TRUE(step({.005, 0., 0.}).inhibited);}
  EXPECT_DOUBLE_EQ(limiter.previousCommand()->vx, 0.);
}

TEST_F(GateTest, SimulationInitialRiskWaitDoesNotLockAnOtherwiseSafeFarRedApproach)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  risk = PathRiskTrigger();
  ASSERT_TRUE(step({}).inhibited);
  red(2.3);
  const auto d = step({.05, 0., 0.});
  EXPECT_FALSE(d.inhibited); EXPECT_DOUBLE_EQ(d.command.vx, .05);
}

TEST_F(GateTest, SimulationRealObstacleStopStillRequiresFullStopAfterRiskClears)
{
  gate = ExecutionSafetyGate(nominal(), ExecutionSafetyOptions{.025, 512, 500000, .2}, true);
  limiter.seed({.4, 0., 0.}); wall(.3);
  ASSERT_TRUE(step({.5, 0., 0.}, {.4, 0., 0.}).inhibited);
  geometry.snapshot->map->resetMap(0, 0, 200, 200);
  risk.state = msg::PathRiskState::CLEAR;
  const auto d = step({.5, 0., 0.}, {.4, 0., 0.});
  EXPECT_TRUE(d.inhibited); EXPECT_EQ(d.reason, "braking_not_stopped");
}

TEST(PathRiskTrigger, SimulationSkewAllowanceDoesNotAcceptExpiredOrWrongPathEvidence)
{
  using Risk = msg::PathRiskState;
  const int64_t now = 2000000000LL; const auto wall = Wall::now();
  msg::TrackingProgress current;
  current.valid = true; current.controller_instance = "controller";
  current.plan_revision = 7; current.path_fingerprint = 42; current.path_header.frame_id = "map";
  current.header.stamp = rclcpp::Time(now); current.loaded_at = rclcpp::Time(now - 200000000LL);
  Risk risk;
  risk.header.frame_id = "map"; risk.header.stamp = rclcpp::Time(now + 10000000LL);
  risk.progress_stamp = rclcpp::Time(now + 20000000LL); risk.map_stamp = risk.header.stamp;
  risk.path_header = current.path_header; risk.controller_instance = current.controller_instance;
  risk.plan_revision = current.plan_revision; risk.path_fingerprint = current.path_fingerprint;
  risk.state = Risk::CLEAR;
  auto check = [&]() {return validatePathRisk(&risk, wall, current, now, wall, true);};
  EXPECT_EQ(check().state, Risk::CLEAR);
  risk.header.stamp = rclcpp::Time(now + 101000000LL); EXPECT_EQ(check().state, Risk::UNAVAILABLE);
  risk.header.stamp = rclcpp::Time(now - 400000000LL); EXPECT_EQ(check().state, Risk::UNAVAILABLE);
  risk.header.stamp = rclcpp::Time(now + 10000000LL); --risk.plan_revision;
  EXPECT_EQ(check().state, Risk::UNAVAILABLE);
  ++risk.plan_revision; risk.state = Risk::COLLISION_AHEAD;
  risk.footprint_collision = true; risk.footprint_distance = .4;
  EXPECT_EQ(check().state, Risk::UNAVAILABLE);  // No actual pose time.
  risk.footprint_origin_stamp = rclcpp::Time(now - 200000000LL);
  EXPECT_EQ(check().state, Risk::COLLISION_AHEAD);
  EXPECT_EQ(check().footprint_origin_ns, now - 200000000LL);
  risk.footprint_origin_stamp = rclcpp::Time(now - 601000000LL);
  EXPECT_EQ(check().state, Risk::UNAVAILABLE);
  risk.footprint_origin_stamp = rclcpp::Time(now);
  risk.footprint_distance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(check().state, Risk::UNAVAILABLE);
}
}  // namespace
}  // namespace pid_tracking_controller
