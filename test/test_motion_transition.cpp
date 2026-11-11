#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "motion_transition.hpp"

namespace pid_tracking_controller
{
namespace
{
MotionTransitionInput request(HolonomicBodyCommand measured = {},
  HolonomicBodyCommand candidate = {})
{
  MotionTransitionInput in;
  in.measured = measured;
  in.candidate = candidate;
  in.candidate_execution = {0., 0.};
  in.candidate_duration = 1.;
  in.acceleration = in.deceleration = in.braking_deceleration = {1., 1., 1.};
  return in;
}

// Exact integral for the emitted piecewise-linear body velocities. World-frame
// coupled translation/rotation is independently checked in test_execution_safety.
HolonomicBodyCommand integral(const MotionTransition & motion)
{
  HolonomicBodyCommand value;
  for (const auto & s : motion.segments) {
    value.vx += (s.start.vx + s.end.vx) * s.duration / 2;
    value.vy += (s.start.vy + s.end.vy) * s.duration / 2;
    value.omega += (s.start.omega + s.end.omega) * s.duration / 2;
  }
  return value;
}

double duration(const MotionTransition & motion)
{
  double time = 0.;
  for (const auto & s : motion.segments) {time += s.duration;}
  return time;
}

void expectContinuousStop(const MotionTransition & motion, HolonomicBodyCommand start)
{
  ASSERT_TRUE(motion.available) << motion.reason;
  for (const auto & s : motion.segments) {
    EXPECT_GT(s.duration, 0.);
    EXPECT_DOUBLE_EQ(s.start.vx, start.vx);
    EXPECT_DOUBLE_EQ(s.start.vy, start.vy);
    EXPECT_DOUBLE_EQ(s.start.omega, start.omega);
    start = s.end;
  }
  EXPECT_DOUBLE_EQ(start.vx, 0.);
  EXPECT_DOUBLE_EQ(start.vy, 0.);
  EXPECT_DOUBLE_EQ(start.omega, 0.);
}

TEST(MotionTransition, straightAndLateralAccelerationIncludeFullBraking)
{
  for (const auto v : {HolonomicBodyCommand{1., 0., 0.}, {0., -1., 0.}}) {
    const auto motion = buildMotionTransition(request({}, v));
    expectContinuousStop(motion, {});
    ASSERT_TRUE(motion.available);
    EXPECT_NEAR(duration(motion), 2., 1e-12);
    EXPECT_NEAR(integral(motion).vx, v.vx, 1e-12);
    EXPECT_NEAR(integral(motion).vy, v.vy, 1e-12);
  }
}

TEST(MotionTransition, reversalBrakesToZeroBeforeAcceleratingOtherWay)
{
  auto in = request({1., 0., 0.}, {-1., 0., 0.});
  in.acceleration.vx = 2.;
  in.candidate_duration = 1.5;
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, in.measured);
  ASSERT_TRUE(motion.available);
  // +0.5 while braking, -0.25 while accelerating, -0.5 in final braking.
  EXPECT_NEAR(integral(motion).vx, -.25, 1e-12);
  EXPECT_NEAR(duration(motion), 2.5, 1e-12);
  bool reached_zero_before_reverse = false;
  for (const auto & s : motion.segments) {
    if (s.end.vx == 0.) {reached_zero_before_reverse = true;}
    if (s.end.vx < 0.) {EXPECT_TRUE(reached_zero_before_reverse);}
  }
}

TEST(MotionTransition, finiteHorizonCanBrakeAnUnfinishedCandidateRamp)
{
  auto in = request({}, {3., 0., 0.});
  in.candidate_duration = .25;
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, {});
  ASSERT_TRUE(motion.available);
  EXPECT_NEAR(duration(motion), .5, 1e-12);
  EXPECT_NEAR(integral(motion).vx, .0625, 1e-12);
}

TEST(MotionTransition, activeCommandExpiresDuringCandidateDelay)
{
  auto in = request();
  in.active_command = HolonomicBodyCommand{1., 0., 0.};
  in.active_valid_for = .25;
  in.candidate_execution = {1., 1.};
  in.candidate_duration = 0.;
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, {});
  ASSERT_TRUE(motion.available);
  EXPECT_NEAR(duration(motion), 1., 1e-12);
  EXPECT_NEAR(integral(motion).vx, .0625, 1e-12);
}

TEST(MotionTransition, pendingPublishedCommandExecutesAndExpires)
{
  auto in = request();
  in.active_command = HolonomicBodyCommand{};
  in.active_valid_for = 2.;
  in.candidate_execution = {1., 1.};
  in.candidate_duration = 0.;
  in.pending.push_back({{1., 0., 0.}, {.2, .2}, .6, true});
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, {});
  ASSERT_TRUE(motion.available);
  EXPECT_NEAR(duration(motion), 1., 1e-12);
  EXPECT_NEAR(integral(motion).vx, .16, 1e-12);
}

TEST(MotionTransition, supersededExpiryDoesNotCancelNewerCommand)
{
  auto in = request();
  in.active_command = HolonomicBodyCommand{};
  in.active_valid_for = .25;
  in.candidate_execution = {.5, .5};
  in.candidate_duration = 0.;
  in.pending.push_back({{1., 0., 0.}, {0., 0.}, 2., true});
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, {});
  ASSERT_TRUE(motion.available);
  EXPECT_NEAR(duration(motion), 1., 1e-12);
  EXPECT_NEAR(integral(motion).vx, .25, 1e-12);
}

TEST(MotionTransition, unissuedAndAlreadyExpiredEntriesDoNotGovernMotion)
{
  auto in = request({}, {1., 0., 0.});
  in.pending.push_back({});  // Rejected/unissued, no usable timing needed.
  in.pending.push_back({{}, {}, 0., true});  // Expired at the observation origin.
  in.pending.push_back({{9., 0., 0.}, {.5, .5}, .4, true});  // Expires before apply.
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, {});
  ASSERT_TRUE(motion.available);
  EXPECT_NEAR(integral(motion).vx, 1., 1e-12);
}

TEST(MotionTransition, missingOrUncertainExecutionHistoryIsUnavailable)
{
  auto in = request();
  in.candidate_execution = {.1, .2};
  EXPECT_FALSE(buildMotionTransition(in).available);
  in.candidate_execution = {.2, .2};
  EXPECT_FALSE(buildMotionTransition(in).available);  // No active command.
  in.active_command = HolonomicBodyCommand{};
  EXPECT_FALSE(buildMotionTransition(in).available);  // Unknown active expiry.
  in.active_valid_for = 1.;
  in.pending.push_back({{}, {.05, .1}, 1., true});
  EXPECT_FALSE(buildMotionTransition(in).available);
  in.pending.front().execution = {.1, .1};
  EXPECT_TRUE(buildMotionTransition(in).available);
  in.pending.push_back({{}, {.08, .08}, 1., true});
  EXPECT_FALSE(buildMotionTransition(in).available);  // Unknown ordering.
  in.pending.back().execution = {.1, .1};
  EXPECT_FALSE(buildMotionTransition(in).available);  // Simultaneous ambiguity.
  in.pending.back().execution = {.3, .3};
  EXPECT_FALSE(buildMotionTransition(in).available);  // Could override candidate.
}

TEST(MotionTransition, invalidRatesNonfiniteInputsAndHistoryOverflowFailClosed)
{
  for (int field = 0; field < 7; ++field) {
    auto in = request({}, {1., 0., 0.});
    if (field == 0) {in.acceleration.vx = 0.;}
    if (field == 1) {in.deceleration.vy = -1.;}
    if (field == 2) {in.braking_deceleration.omega = 0.;}
    if (field == 3) {in.measured.vx = std::numeric_limits<double>::infinity();}
    if (field == 4) {in.candidate_execution = {};}
    if (field == 5) {in.candidate_duration = -1.;}
    if (field == 6) {in.pending.resize(17);}
    EXPECT_FALSE(buildMotionTransition(in).available) << field;
  }
  auto in = request({1., 0., 0.});
  in.candidate_duration = 0.;
  in.braking_deceleration.vx = std::numeric_limits<double>::denorm_min();
  EXPECT_FALSE(buildMotionTransition(in).available);
}

TEST(MotionTransition, unequalAxisStopEventsCompleteDespiteFloatingPointResiduals)
{
  for (const auto v : {HolonomicBodyCommand{1., .2, .3},
    {.0801, -.04985, .73}, {-.7, .3, -.8}})
  {
    for (const auto rates : {HolonomicBodyCommand{1., 1., 1.}, {1.1, .6, 1.}}) {
      auto in = request(v);
      in.candidate_duration = 0.;
      in.braking_deceleration = rates;
      for (const auto & motion : {buildMotionTransition(in), buildCommandMotion(v, 0., rates)}) {
        expectContinuousStop(motion, v);
        ASSERT_TRUE(motion.available) << motion.reason;
        const auto area = integral(motion);
        EXPECT_NEAR(area.vx, v.vx * std::abs(v.vx) / (2 * rates.vx), 1e-12);
        EXPECT_NEAR(area.vy, v.vy * std::abs(v.vy) / (2 * rates.vy), 1e-12);
        EXPECT_NEAR(area.omega, v.omega * std::abs(v.omega) / (2 * rates.omega), 1e-12);
      }
    }
  }
}

TEST(MotionTransition, reconstructedSmootherStartupChangesDirectionBeforeTarget)
{
  // 2026-11-11 log-inspired reconstruction, not replay: independent axes at
  // 1.1 m/s^2 for one 50 Hz cycle. No assumption about wheel steering response.
  auto in = request({}, {.0801, .04985, 0.});
  in.acceleration = {1.1, 1.1, 1.};
  in.candidate_duration = .02;
  const auto motion = buildMotionTransition(in);
  expectContinuousStop(motion, {});
  ASSERT_TRUE(motion.available);
  ASSERT_FALSE(motion.segments.empty());
  const auto & reached = motion.segments.front().end;
  EXPECT_NEAR(reached.vx, .022, 1e-12);
  EXPECT_NEAR(reached.vy, .022, 1e-12);
  EXPECT_NEAR(std::atan2(reached.vy, reached.vx), M_PI / 4, 1e-12);
  EXPECT_LT(std::atan2(in.candidate.vy, in.candidate.vx), .56);
}
}  // namespace
}  // namespace pid_tracking_controller
