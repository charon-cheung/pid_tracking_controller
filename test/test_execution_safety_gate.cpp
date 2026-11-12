#include <chrono>
#include <limits>
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
    s.pose = {0., 0., 0.};
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
    auto & series = h.odometry; series = {};
    auto & s = series.samples[0];
    s.vx = measured.vx; s.vy = measured.vy; s.omega = measured.omega;
    s.stamp_ns = now; s.received = Wall::now(); s.count = 1;
    s.payload = "valid"; s.frame = "base"; series.size = series.next = 1;
    return gate.apply(geometry, h, now, now, revision, candidate, measured,
      dt, limiter.previousCommand(), limiter, started, risk);
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

TEST_F(GateTest, PathClearSkipsSweepAndHazardsReenableItWithoutRetainingAnOldStop)
{
  wall(.30);
  for (auto state : {msg::PathRiskState::CLEARANCE_LOW, msg::PathRiskState::COLLISION_AHEAD}) {
    risk.state = state;
    limiter.seed({.4, 0., 0.});
    const auto blocked = step({.5, 0., 0.}, {.4, 0., 0.});
    ASSERT_EQ(blocked.candidate.status, SweepStatus::kCollision);
    ASSERT_TRUE(blocked.inhibited); EXPECT_TRUE(blocked.sweep_requested);
    risk.state = msg::PathRiskState::CLEAR;
    const auto bypass = step({.5, -.2, .1}, {.4, 0., 0.});
    EXPECT_FALSE(bypass.inhibited); EXPECT_FALSE(bypass.sweep_requested);
    EXPECT_STREQ(bypass.action, "path_clear_bypass");
    // Deliberately contradictory map: verify bypass, not accidental sweep success.
    EXPECT_EQ(bypass.candidate.status, SweepStatus::kUnavailable);
    EXPECT_STREQ(bypass.candidate.reason, "not_checked_path_clear");
    EXPECT_DOUBLE_EQ(bypass.command.vx, .5); EXPECT_DOUBLE_EQ(bypass.command.vy, -.2);
    EXPECT_DOUBLE_EQ(bypass.command.omega, .1);
    EXPECT_DOUBLE_EQ(limiter.previousCommand()->vx, .5);
  }
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

TEST_F(GateTest, CollisionBrakesThenHoldsUntilRejectedMotionAndCurrentCandidateAreClear)
{
  limiter.seed({.4, 0., 0.}); wall(.30);
  auto first = step({.5, 0., 0.}, {.4, 0., 0.});
  ASSERT_EQ(first.candidate.status, SweepStatus::kCollision);
  ASSERT_STREQ(first.action, "braking");
  EXPECT_NEAR(first.command.vx, .35, 1e-12);
  EXPECT_EQ(first.output.status, SweepStatus::kClear);
  for (int i = 0; i < 20; ++i) {
    const auto previous = *limiter.previousCommand();
    const auto result = step({.05, 0., 0.}, previous);
    EXPECT_TRUE(result.inhibited);
    EXPECT_LE(result.command.vx, previous.vx);
  }
  EXPECT_NEAR(limiter.previousCommand()->vx, 0., 1e-12);
  // A zero candidate is not evidence that a motion restart is clear.
  EXPECT_TRUE(step({}).inhibited);
  geometry.snapshot->map->resetMap(0, 0, 200, 200);
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  const auto released = step({.05, 0., 0.});
  EXPECT_FALSE(released.inhibited); EXPECT_STREQ(released.action, "released");
  EXPECT_DOUBLE_EQ(released.command.vx, .05);
  // No stale clear authorization: the very next candidate sees the new obstacle.
  wall(0.);
  EXPECT_TRUE(step({.1, 0., 0.}).inhibited);
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
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  ++revision;
  EXPECT_TRUE(step({.05, 0., 0.}).inhibited);
  ++h.dropped;
  const auto gap = step({.05, 0., 0.});
  EXPECT_TRUE(gap.inhibited); EXPECT_EQ(gap.reason, "odometry_history_gap");
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
      return gate.apply(geometry, history, now, now, revision, {.2, 0., 0.}, {}, .1,
        limiter.previousCommand(), limiter, Wall::now(), risk);
    };
  record(); EXPECT_FALSE(run().inhibited);
  // A new plan discards even a just-received old-plan velocity sample.
  now += 100000000LL; record(); ++revision;
  EXPECT_TRUE(run().inhibited);
  now += 100000000LL; record(); EXPECT_FALSE(run().inhibited);
  // Clock rollback must be fenced without any timer or logger call.
  now -= 1000000000LL;
  EXPECT_TRUE(run().inhibited);
  now += 100000000LL; record(); EXPECT_FALSE(run().inhibited);
  // Same-stamp conflicting velocities cannot be silently resolved by picking latest.
  now += 100000000LL; record(0.); record(.3);
  const auto ambiguous = run();
  EXPECT_TRUE(ambiguous.inhibited); EXPECT_EQ(ambiguous.reason, "odometry_anchor_ambiguous");
  motion.deactivate(); motion.activate(); now += 100000000LL;
  EXPECT_TRUE(run().inhibited);
  now += 100000000LL; record(); EXPECT_FALSE(run().inhibited);
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
}  // namespace
}  // namespace pid_tracking_controller
