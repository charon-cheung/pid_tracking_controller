#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"
#include "holonomic_command_limiter.hpp"
#include "rotation_completion_tracker.hpp"
#include "rotation_velocity_planner.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1.0 / 15.0;
constexpr double kTolerance = kPi / 180.0;

RotationVelocityPlannerConfig config()
{
  return {2.5, 0.20, 0.50, 0.50, kTolerance};
}

TEST(RotationVelocityPlanner, independentGainAndDelayBoundSmallAndLargeTurns)
{
  const RotationVelocityPlanner planner(config());
  for (const double sign : {-1.0, 1.0}) {
    // Small-angle target is larger than the old kp_yaw=1 output.
    const auto small = planner.plan(sign * 0.10, 0.0, 0.0, kDt);
    EXPECT_GT(sign * small.omega, 0.10);
    EXPECT_LE(std::abs(small.omega), 2.5 * 0.10);
    EXPECT_LE(std::abs(small.omega) * (0.2 + kDt) +
      small.omega * small.omega / (2.0 * 0.5), 0.10 + 1e-12);
    EXPECT_DOUBLE_EQ(planner.plan(sign * 3.0, 0.0, 0.0, kDt).omega, sign * 0.5);
  }
}

TEST(RotationVelocityPlanner, rawFeedbackAndAcceptedOutputBothAdvanceBraking)
{
  const RotationVelocityPlanner planner(config());
  for (const double sign : {-1.0, 1.0}) {
    const auto stopped = planner.plan(sign * 0.15, 0.0, 0.0, kDt);
    for (const auto & speeds : {std::pair<double, double>{0.5, 0.1}, {0.1, 0.5}}) {
      const auto moving = planner.plan(sign * 0.15, sign * speeds.first,
        sign * speeds.second, kDt);
      EXPECT_LT(std::abs(moving.omega), std::abs(stopped.omega));
      EXPECT_GT(moving.predicted_stopping_angle, 0.15);
      // Feedback well beyond the configured speed cap must not be clipped away.
      EXPECT_DOUBLE_EQ(planner.plan(sign * 0.15, sign * 1.0, 0.0, kDt).omega, 0.0);
    }
  }
}

TEST(RotationVelocityPlanner, toleranceStopsTargetWithoutHidingMotionOrLatching)
{
  const RotationVelocityPlanner planner(config());
  for (const double error : {-kTolerance, 0.0, kTolerance}) {
    const auto plan = planner.plan(error, 0.3, 0.3, kDt);
    EXPECT_DOUBLE_EQ(plan.omega, 0.0);
  }
  // Leaving tolerance must resume correction in either direction.
  EXPECT_GT(planner.plan(0.03, 0.0, 0.0, kDt).omega, 0.0);
  EXPECT_LT(planner.plan(-0.03, 0.0, 0.0, kDt).omega, 0.0);
}

TEST(RotationVelocityPlanner, wrapAndOppositeMotionRetainShortestCorrection)
{
  const RotationVelocityPlanner planner(config());
  EXPECT_NEAR(planner.plan(2 * kPi - 0.1, 0.0, 0.0, kDt).omega,
    planner.plan(-0.1, 0.0, 0.0, kDt).omega, 1e-12);
  EXPECT_NEAR(planner.plan(-2 * kPi + 0.1, 0.0, 0.0, kDt).omega,
    planner.plan(0.1, 0.0, 0.0, kDt).omega, 1e-12);
  for (const double sign : {-1.0, 1.0}) {
    const auto plan = planner.plan(sign * 0.1, -sign * 0.3, -sign * 0.3, kDt);
    EXPECT_GT(sign * plan.omega, 0.0);  // Limiter owns bounded reversal.
    EXPECT_DOUBLE_EQ(plan.predicted_stopping_angle, 0.0);
  }
}

TEST(RotationVelocityPlanner, largerDelayAndControlIntervalReduceTarget)
{
  auto delayed_config = config();
  delayed_config.response_delay = 0.4;
  const RotationVelocityPlanner planner(config()), delayed(delayed_config);
  const auto normal = planner.plan(0.15, 0.25, 0.25, kDt);
  EXPECT_LT(delayed.plan(0.15, 0.25, 0.25, kDt).omega, normal.omega);
  EXPECT_LT(planner.plan(0.15, 0.25, 0.25, 0.2).omega, normal.omega);
}

TEST(RotationVelocityPlanner, rejectsInvalidConfigurationAndInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto member : {&RotationVelocityPlannerConfig::kp_yaw,
      &RotationVelocityPlannerConfig::response_delay, &RotationVelocityPlannerConfig::max_omega,
      &RotationVelocityPlannerConfig::max_deceleration,
      &RotationVelocityPlannerConfig::yaw_tolerance})
  {
    for (const double invalid : {-1.0, nan, inf}) {
      auto bad = config(); bad.*member = invalid;
      EXPECT_THROW((RotationVelocityPlanner{bad}), std::invalid_argument);
    }
    auto zero = config(); zero.*member = 0.0;
    if (member == &RotationVelocityPlannerConfig::response_delay ||
      member == &RotationVelocityPlannerConfig::yaw_tolerance)
    {
      EXPECT_NO_THROW((RotationVelocityPlanner{zero}));
    } else {
      EXPECT_THROW((RotationVelocityPlanner{zero}), std::invalid_argument);
    }
  }
  const RotationVelocityPlanner planner(config());
  for (const double invalid : {nan, inf}) {
    EXPECT_THROW(planner.plan(invalid, 0, 0, kDt), std::domain_error);
    EXPECT_THROW(planner.plan(0.1, invalid, 0, kDt), std::domain_error);
    EXPECT_THROW(planner.plan(0.1, 0, invalid, kDt), std::domain_error);
    EXPECT_THROW(planner.plan(0.1, 0, 0, invalid), std::domain_error);
  }
  EXPECT_THROW(planner.plan(0.1, 0, 0, -0.1), std::domain_error);
}

struct TurnResult
{
  double duration{std::numeric_limits<double>::infinity()};
  double overshoot{0.0};
  double final_error{0.0};
};

// Deterministic plant: transport delay, first-order response (80 ms), drive gain.
// This probes the controller envelope; it is not a calibrated vehicle model.
TurnResult simulateTurn(
  double degrees, double delay, double gain, bool use_planner,
  double response_budget = 0.20)
{
  auto limits = HolonomicCommandLimiterConfig{};
  limits.max_omega = 0.5; limits.max_alpha = 0.5; limits.rotation_max_alpha = 0.8;
  HolonomicCommandLimiter limiter(limits);
  limiter.seed({});
  auto planner_config = config();
  planner_config.response_delay = response_budget;
  const RotationVelocityPlanner planner(planner_config);
  RotationCompletionTracker completion({kTolerance, 0.02, 0.01, 0.3});
  std::deque<double> transport(static_cast<std::size_t>(std::llround(delay / kDt)), 0.0);
  const double goal = degrees * kPi / 180.0;
  double yaw = 0.0, measured = 0.0, previous = 0.0;
  bool previously_settled = false;
  TurnResult result;
  for (int step = 0; step < 450; ++step) {
    const double error = goal - yaw;
    const double target = use_planner ? planner.plan(error, measured, previous, kDt).omega :
      std::clamp(error, -limits.max_omega, limits.max_omega);
    const auto output = limiter.limit({0, 0, target}, {0, 0, measured}, kDt,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), true);
    EXPECT_LE(std::abs(output.command.omega), limits.max_omega + 1e-12);
    const bool accelerating = target * goal > 0 && target * previous >= 0 &&
      std::abs(target) > std::abs(previous);
    EXPECT_LE(std::abs(output.command.omega - previous),
      (accelerating ? 0.8 : 0.5) * kDt + 1e-12);
    previous = output.command.omega;
    transport.push_back(previous);
    const double delayed = transport.front(); transport.pop_front();
    measured += (gain * delayed - measured) * (1.0 - std::exp(-kDt / 0.08));
    yaw += measured * kDt;
    result.overshoot = std::max(result.overshoot, std::copysign(1.0, goal) * (yaw - goal));
    const auto status = completion.update(goal - yaw, 0, 0, measured,
      previously_settled ? kDt : 0.0);
    previously_settled = status.all_conditions;
    if (status.acknowledge_candidate) {
      result.duration = (step + 1) * kDt;
      result.final_error = goal - yaw;
      break;
    }
  }
  return result;
}

using TurnScenario = std::tuple<double, double, double>;
class RotationResponse : public ::testing::TestWithParam<TurnScenario> {};

TEST_P(RotationResponse, convergesWithBoundedCommandsAndContinuousMeasuredSettle)
{
  const auto [degrees, delay, gain] = GetParam();
  const auto result = simulateTurn(degrees, delay, gain, true);
  EXPECT_LT(result.duration, 15.0);
  EXPECT_LE(std::abs(result.final_error), kTolerance);
  // Above-budget delays must still converge with bounded commands. They cannot
  // inherit an overshoot guarantee from a deliberately insufficient budget.
  if (delay <= 0.2 && gain <= 1.0) {
    EXPECT_LE(result.overshoot, kTolerance);
  }
}

INSTANTIATE_TEST_SUITE_P(SignedAnglesDelaysAndGains, RotationResponse,
  ::testing::Combine(::testing::Values(-172., -90., -19.8, -17., -7.6, -2.,
      2., 7.6, 17., 19.8, 90., 172.),
    ::testing::Values(0., kDt, 0.2, 0.3), ::testing::Values(0.85, 1.0, 1.1)));

TEST(RotationVelocityPlanner, largerResponseBudgetReducesOvershootForSlowDrive)
{
  for (const double degrees : {-172., -90., -19.8, 19.8, 90., 172.}) {
    SCOPED_TRACE(degrees);
    // 0.3 s transport rounds up to five cycles (0.333 s), plus 80 ms response.
    const auto underestimated = simulateTurn(degrees, 0.3, 1.1, true, 0.20);
    const auto matched = simulateTurn(degrees, 0.3, 1.1, true, 0.40);
    EXPECT_LT(matched.duration, 15.0);
    EXPECT_LT(matched.overshoot, underestimated.overshoot);
    EXPECT_LE(matched.overshoot, kTolerance);
    RecordProperty("slow_drive_" + std::to_string(degrees),
      std::to_string(underestimated.overshoot * 180.0 / kPi) + " -> " +
      std::to_string(matched.overshoot * 180.0 / kPi));
  }
}

TEST(RotationVelocityPlanner, reducesSmallTurnTailAgainstSamePlantAndLimits)
{
  for (const double degrees : {7.6, 17., 19.8, 172.}) {
    SCOPED_TRACE(degrees);
    const auto before = simulateTurn(degrees, kDt, 0.9, false);
    const auto after = simulateTurn(degrees, kDt, 0.9, true);
    EXPECT_LT(after.duration, before.duration);
    if (degrees < 20.) {EXPECT_LT(after.duration, 0.8 * before.duration);}
    EXPECT_LE(after.overshoot, kTolerance);
    RecordProperty("turn_" + std::to_string(degrees),
      std::to_string(before.duration) + " -> " + std::to_string(after.duration));
  }
}

}  // namespace
}  // namespace pid_tracking_controller
