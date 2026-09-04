#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "holonomic_command_limiter.hpp"

namespace pid_tracking_controller
{
namespace
{

HolonomicCommandLimiterConfig makeConfig()
{
  HolonomicCommandLimiterConfig config;
  config.max_vx = 1.0;
  config.max_vy = 1.0;
  config.max_xy_speed = 1.0;
  config.max_omega = 1.0;
  config.max_ax = 1.0;
  config.max_ay = 1.0;
  config.max_alpha = 2.0;
  config.control_gap_threshold = 0.5;
  return config;
}

HolonomicBodyCommand command(double vx, double vy, double omega)
{
  return {vx, vy, omega};
}

long double xySpeed(const HolonomicBodyCommand & value)
{
  return std::hypot(
    static_cast<long double>(value.vx),
    static_cast<long double>(value.vy));
}

TEST(HolonomicCommandLimiter, lateralVelocityCannotFlipSignInOneNormalCycle)
{
  auto config = makeConfig();
  config.max_ay = 0.5;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.0, 0.5, 0.0));

  const auto result = limiter.limit(
    command(0.0, -0.5, 0.0), command(0.0, 0.5, 0.0), 0.1);

  EXPECT_NEAR(result.command.vy, 0.45, 1e-12);
  EXPECT_GT(result.command.vy, 0.0);
  EXPECT_TRUE(result.ay_limited);
  EXPECT_NEAR(result.translation_lambda, 0.05, 1e-12);
}

TEST(HolonomicCommandLimiter, angularVelocityCannotFlipSignInOneNormalCycle)
{
  auto config = makeConfig();
  config.max_alpha = 1.0;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.0, 0.0, 0.8));

  const auto result = limiter.limit(
    command(0.0, 0.0, -0.8), command(0.0, 0.0, 0.8), 0.1);

  EXPECT_NEAR(result.command.omega, 0.7, 1e-12);
  EXPECT_GT(result.command.omega, 0.0);
  EXPECT_TRUE(result.alpha_limited);
}

TEST(HolonomicCommandLimiter, commonLambdaPreservesTargetDifferenceDirection)
{
  auto config = makeConfig();
  config.max_vx = 5.0;
  config.max_vy = 5.0;
  config.max_xy_speed = 5.0;
  config.max_ax = 1.0;
  config.max_ay = 10.0;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.2, -0.1, 0.0));

  const auto result = limiter.limit(
    command(2.2, 1.9, 0.0), command(-4.0, -4.0, 0.0), 0.25);

  EXPECT_NEAR(result.translation_lambda, 0.125, 1e-12);
  EXPECT_NEAR(result.command.vx, 0.45, 1e-12);
  EXPECT_NEAR(result.command.vy, 0.15, 1e-12);
  const double target_dx = result.projected_target.vx - 0.2;
  const double target_dy = result.projected_target.vy + 0.1;
  const double command_dx = result.command.vx - 0.2;
  const double command_dy = result.command.vy + 0.1;
  EXPECT_NEAR(command_dx * target_dy - command_dy * target_dx, 0.0, 1e-12);
  EXPECT_GT(command_dx * target_dx + command_dy * target_dy, 0.0);
  EXPECT_TRUE(result.ax_limited);
  EXPECT_FALSE(result.ay_limited);
}

TEST(HolonomicCommandLimiter, baseProjectionSatisfiesAllVelocityLimits)
{
  auto config = makeConfig();
  config.max_vx = 0.6;
  config.max_vy = 0.4;
  config.max_xy_speed = 0.5;
  config.max_omega = 0.7;
  HolonomicCommandLimiter limiter(config);

  const auto result = limiter.limit(
    command(2.0, -3.0, 1.5), command(2.0, -3.0, 1.5), 0.0);

  EXPECT_LE(std::abs(result.command.vx), config.max_vx);
  EXPECT_LE(std::abs(result.command.vy), config.max_vy);
  EXPECT_LE(std::hypot(result.command.vx, result.command.vy), config.max_xy_speed + 1e-12);
  EXPECT_LE(std::abs(result.command.omega), config.max_omega);
  EXPECT_TRUE(result.vx_saturated);
  EXPECT_TRUE(result.vy_saturated);
  EXPECT_TRUE(result.xy_saturated);
  EXPECT_TRUE(result.omega_saturated);
}

TEST(HolonomicCommandLimiter, normalCycleChangesRespectAccelerationLimits)
{
  auto config = makeConfig();
  config.max_vx = 5.0;
  config.max_vy = 5.0;
  config.max_xy_speed = 5.0;
  config.max_omega = 5.0;
  config.max_ax = 0.4;
  config.max_ay = 0.7;
  config.max_alpha = 0.9;
  HolonomicCommandLimiter limiter(config);
  const auto previous = command(-0.2, 0.3, -0.4);
  limiter.seed(previous);

  const double dt = 0.2;
  const auto result = limiter.limit(
    command(2.0, -2.0, 3.0), command(4.0, 4.0, 4.0), dt);

  EXPECT_LE(std::abs(result.command.vx - previous.vx), config.max_ax * dt + 1e-12);
  EXPECT_LE(std::abs(result.command.vy - previous.vy), config.max_ay * dt + 1e-12);
  EXPECT_LE(
    std::abs(result.command.omega - previous.omega), config.max_alpha * dt + 1e-12);
  EXPECT_TRUE(result.ax_limited);
  EXPECT_TRUE(result.ay_limited);
  EXPECT_TRUE(result.alpha_limited);
}

TEST(HolonomicCommandLimiter, abnormalDtReseedsFromMeasuredWithoutTargetAdvance)
{
  for (const double dt : {
        0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), 0.500001})
  {
    HolonomicCommandLimiter limiter(makeConfig());
    limiter.seed(command(0.4, 0.0, 0.2));

    const auto result = limiter.limit(
      command(-1.0, -1.0, -1.0), command(0.3, -0.2, 0.1), dt);

    EXPECT_TRUE(result.reseeded);
    EXPECT_NEAR(result.translation_lambda, 0.0, 1e-12);
    EXPECT_NEAR(result.command.vx, 0.3, 1e-12);
    EXPECT_NEAR(result.command.vy, -0.2, 1e-12);
    EXPECT_NEAR(result.command.omega, 0.1, 1e-12);
  }
}

TEST(HolonomicCommandLimiter, initialCallReseedsAndSubsequentCallUsesStoredState)
{
  HolonomicCommandLimiter limiter(makeConfig());

  const auto initial = limiter.limit(
    command(-1.0, 0.0, 0.0), command(0.4, 0.0, 0.0), 0.1);
  const auto next = limiter.limit(
    command(-1.0, 0.0, 0.0), command(0.9, 0.0, 0.0), 0.1);

  EXPECT_TRUE(initial.reseeded);
  EXPECT_NEAR(initial.command.vx, 0.4, 1e-12);
  EXPECT_FALSE(next.reseeded);
  EXPECT_NEAR(next.command.vx, 0.3, 1e-12);
}

TEST(HolonomicCommandLimiter, externalLimitDecreaseIsImmediateAndIncreaseRamps)
{
  auto config = makeConfig();
  config.max_vx = 2.0;
  config.max_vy = 2.0;
  config.max_xy_speed = 2.0;
  config.max_ax = 0.5;
  config.max_ay = 0.5;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(1.0, 0.0, 0.0));

  const auto decreased = limiter.limit(
    command(1.0, 0.0, 0.0), command(0.0, 0.0, 0.0), 0.1, 0.2);
  const auto increased = limiter.limit(
    command(1.0, 0.0, 0.0), command(0.9, 0.0, 0.0), 0.1, 1.0);

  EXPECT_TRUE(decreased.external_limit_applied);
  EXPECT_NEAR(decreased.command.vx, 0.2, 1e-12);
  EXPECT_FALSE(increased.external_limit_applied);
  EXPECT_TRUE(increased.ax_limited);
  EXPECT_NEAR(increased.command.vx, 0.25, 1e-12);
}

TEST(HolonomicCommandLimiter, externalCapScalingPreservesTranslationDirection)
{
  HolonomicCommandLimiter limiter(makeConfig());
  limiter.seed(command(0.0, 0.0, 0.0));

  const auto result = limiter.limit(
    command(0.6, 0.8, 0.0), command(0.0, 0.0, 0.0), 0.1, 0.05);

  EXPECT_NEAR(std::hypot(result.command.vx, result.command.vy), 0.05, 1e-12);
  EXPECT_NEAR(result.command.vx / result.command.vy, 0.75, 1e-12);
  EXPECT_TRUE(result.external_limit_applied);
}

TEST(HolonomicCommandLimiter, explicitSeedProjectsMeasuredVelocityIntoAllHardLimits)
{
  auto config = makeConfig();
  config.max_vx = 0.6;
  config.max_vy = 0.4;
  config.max_xy_speed = 0.5;
  config.max_omega = 0.7;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(2.0, -3.0, 2.0), 0.2);

  const auto result = limiter.limit(
    command(0.0, 0.0, 0.0), command(0.0, 0.0, 0.0), 0.1);

  EXPECT_FALSE(result.reseeded);
  EXPECT_NEAR(result.command.vx, 0.06641005886756875, 1e-12);
  EXPECT_NEAR(result.command.vy, -0.04427337257837917, 1e-12);
  EXPECT_NEAR(result.command.omega, 0.5, 1e-12);
}

TEST(HolonomicCommandLimiter, extremeTranslationInterpolationRemainsFinite)
{
  const double maximum = std::numeric_limits<double>::max();
  auto config = makeConfig();
  config.max_vx = maximum;
  config.max_vy = maximum;
  config.max_xy_speed = maximum;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(-maximum, 0.0, 0.0));

  const auto result = limiter.limit(
    command(maximum, 0.0, 0.0), command(0.0, 0.0, 0.0), 0.1);

  EXPECT_TRUE(std::isfinite(result.command.vx));
  EXPECT_EQ(result.command.vx, -maximum);
  EXPECT_EQ(result.translation_lambda, 0.0);
  EXPECT_TRUE(result.ax_limited);
}

TEST(HolonomicCommandLimiter, tinyNonzeroDeltaStillRespectsAccelerationLimit)
{
  auto config = makeConfig();
  config.max_ax = 1.0;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.0, 0.0, 0.0));

  const double dt = 1e-15;
  const auto result = limiter.limit(
    command(5e-13, 0.0, 0.0), command(0.0, 0.0, 0.0), dt);

  EXPECT_TRUE(result.ax_limited);
  EXPECT_NEAR(result.translation_lambda, 0.002, 1e-15);
  EXPECT_NEAR(result.command.vx, config.max_ax * dt, 1e-27);
  EXPECT_LE(std::abs(result.command.vx), config.max_ax * dt);
}

TEST(HolonomicCommandLimiter, subUlpAllowancesRetainNonzeroPreviousCommand)
{
  auto config = makeConfig();
  config.max_vx = 2.0;
  config.max_xy_speed = 2.0;
  config.max_omega = 2.0;
  config.max_ax = 1.0;
  config.max_alpha = 1.0;
  HolonomicCommandLimiter limiter(config);
  const auto previous = command(1.0, 0.0, 1.0);
  limiter.seed(previous);

  const double dt = 1.5e-16;
  const auto result = limiter.limit(
    command(2.0, 0.0, 2.0), command(0.0, 0.0, 0.0), dt);
  const long double translation_change = std::abs(
    static_cast<long double>(result.command.vx) - previous.vx);
  const long double angular_change = std::abs(
    static_cast<long double>(result.command.omega) - previous.omega);

  EXPECT_LE(translation_change, static_cast<long double>(config.max_ax) * dt);
  EXPECT_LE(angular_change, static_cast<long double>(config.max_alpha) * dt);
  EXPECT_EQ(result.command.vx, previous.vx);
  EXPECT_EQ(result.command.omega, previous.omega);
  EXPECT_EQ(result.translation_lambda, 0.0);
  EXPECT_TRUE(result.ax_limited);
  EXPECT_TRUE(result.alpha_limited);
}

TEST(HolonomicCommandLimiter, unsafeSubnormalInterpolationRetainsCoupledTranslation)
{
  const double quantum = std::numeric_limits<double>::denorm_min();
  auto config = makeConfig();
  config.max_vx = 3.0 * quantum;
  config.max_vy = 3.0 * quantum;
  config.max_xy_speed = 3.0 * quantum;
  config.max_ax = quantum;
  config.max_ay = 2.0 * quantum;
  HolonomicCommandLimiter limiter(config);
  const auto previous = command(-3.0 * quantum, 0.0, 0.0);
  limiter.seed(previous);

  const double dt = 0.26;
  const auto result = limiter.limit(
    command(-2.0 * quantum, -2.0 * quantum, 0.0),
    command(0.0, 0.0, 0.0), dt);
  const long double vx_change = std::abs(
    static_cast<long double>(result.command.vx) - previous.vx);
  const long double vy_change = std::abs(
    static_cast<long double>(result.command.vy) - previous.vy);

  EXPECT_LE(vx_change, static_cast<long double>(config.max_ax) * dt);
  EXPECT_LE(vy_change, static_cast<long double>(config.max_ay) * dt);
  EXPECT_LE(xySpeed(result.command), static_cast<long double>(config.max_xy_speed));
  EXPECT_EQ(result.command.vx, previous.vx);
  EXPECT_EQ(result.command.vy, previous.vy);
  EXPECT_EQ(result.translation_lambda, 0.0);
}

TEST(HolonomicCommandLimiter, extremeAngularSlewRemainsFinite)
{
  const double maximum = std::numeric_limits<double>::max();
  auto config = makeConfig();
  config.max_omega = maximum;
  config.max_alpha = maximum;
  config.control_gap_threshold = 2.0;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.0, 0.0, -maximum));

  const auto result = limiter.limit(
    command(0.0, 0.0, maximum), command(0.0, 0.0, 0.0), 2.0);

  EXPECT_TRUE(std::isfinite(result.command.omega));
  EXPECT_EQ(result.command.omega, maximum);
  EXPECT_FALSE(result.alpha_limited);
}

TEST(HolonomicCommandLimiter, extremeBaseProjectionPreservesDirection)
{
  const double maximum = std::numeric_limits<double>::max();
  const double expected_component = maximum / std::sqrt(2.0);
  auto config = makeConfig();
  config.max_vx = maximum;
  config.max_vy = maximum;
  config.max_xy_speed = maximum;
  HolonomicCommandLimiter limiter(config);

  const auto result = limiter.limit(
    command(maximum, maximum, 0.0), command(maximum, maximum, 0.0), 0.0);

  EXPECT_TRUE(result.xy_saturated);
  EXPECT_TRUE(std::isfinite(result.command.vx));
  EXPECT_TRUE(std::isfinite(result.command.vy));
  EXPECT_NEAR(result.command.vx / expected_component, 1.0, 1e-15);
  EXPECT_NEAR(result.command.vy / expected_component, 1.0, 1e-15);
  EXPECT_NEAR(result.command.vx / result.command.vy, 1.0, 1e-15);
}

TEST(HolonomicCommandLimiter, subnormalBaseCapRemainsHardAfterComponentRounding)
{
  const double cap = std::numeric_limits<double>::denorm_min();
  auto config = makeConfig();
  config.max_xy_speed = cap;
  HolonomicCommandLimiter limiter(config);

  const auto result = limiter.limit(
    command(1.0, 1.0, 0.0), command(1.0, 1.0, 0.0), 0.0);

  EXPECT_TRUE(result.xy_saturated);
  EXPECT_LE(xySpeed(result.projected_target), static_cast<long double>(cap));
  EXPECT_LE(xySpeed(result.command), static_cast<long double>(cap));
  EXPECT_TRUE(result.command.vx != 0.0 || result.command.vy != 0.0);
}

TEST(HolonomicCommandLimiter, extremeExternalProjectionDoesNotCollapseToZero)
{
  const double maximum = std::numeric_limits<double>::max();
  const double minimum = std::numeric_limits<double>::min();
  auto config = makeConfig();
  config.max_vx = maximum;
  config.max_vy = maximum;
  config.max_xy_speed = maximum;
  config.max_ax = maximum;
  config.max_ay = maximum;
  config.control_gap_threshold = 1.0;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.0, 0.0, 0.0));

  const auto result = limiter.limit(
    command(0.5 * maximum, 0.5 * maximum, 0.0),
    command(0.0, 0.0, 0.0), 1.0, minimum);

  EXPECT_TRUE(result.external_limit_applied);
  EXPECT_GT(result.command.vx, 0.0);
  EXPECT_GT(result.command.vy, 0.0);
  EXPECT_NEAR(result.command.vx / result.command.vy, 1.0, 1e-15);
  EXPECT_NEAR(
    std::hypot(result.command.vx, result.command.vy) / minimum,
    1.0, 1e-15);
}

TEST(HolonomicCommandLimiter, subnormalExternalCapRemainsHardAfterComponentRounding)
{
  const double cap = std::numeric_limits<double>::denorm_min();
  HolonomicCommandLimiter limiter(makeConfig());
  limiter.seed(command(0.0, 0.0, 0.0));

  const auto result = limiter.limit(
    command(0.5, 0.5, 0.0), command(0.0, 0.0, 0.0), 0.1, cap);

  EXPECT_TRUE(result.external_limit_applied);
  EXPECT_LE(xySpeed(result.command), static_cast<long double>(cap));
  EXPECT_TRUE(result.command.vx != 0.0 || result.command.vy != 0.0);
}

TEST(HolonomicCommandLimiter, zeroExternalCapIsActiveHardStop)
{
  HolonomicCommandLimiter limiter(makeConfig());
  limiter.seed(command(0.0, 0.0, 0.0));

  const auto result = limiter.limit(
    command(0.8, 0.4, 0.0), command(0.0, 0.0, 0.0), 0.1, 0.0);

  EXPECT_TRUE(result.external_limit_applied);
  EXPECT_NEAR(result.command.vx, 0.0, 1e-12);
  EXPECT_NEAR(result.command.vy, 0.0, 1e-12);
}

TEST(HolonomicCommandLimiter, reportsIndependentSaturationAndSlewDiagnostics)
{
  auto config = makeConfig();
  config.max_vx = 0.6;
  config.max_vy = 0.4;
  config.max_xy_speed = 0.5;
  config.max_omega = 0.7;
  config.max_ax = 0.2;
  config.max_ay = 0.3;
  config.max_alpha = 0.4;
  HolonomicCommandLimiter limiter(config);
  limiter.seed(command(0.0, 0.0, 0.0));

  const auto result = limiter.limit(
    command(2.0, -3.0, 1.5), command(0.0, 0.0, 0.0), 0.1, 0.01);

  EXPECT_TRUE(result.vx_saturated);
  EXPECT_TRUE(result.vy_saturated);
  EXPECT_TRUE(result.xy_saturated);
  EXPECT_TRUE(result.omega_saturated);
  EXPECT_TRUE(result.ax_limited);
  EXPECT_TRUE(result.ay_limited);
  EXPECT_TRUE(result.alpha_limited);
  EXPECT_TRUE(result.external_limit_applied);
  EXPECT_FALSE(result.reseeded);
  EXPECT_NEAR(result.projected_target.omega, 0.7, 1e-12);
  EXPECT_NEAR(result.command.omega, 0.04, 1e-12);
  EXPECT_NEAR(std::hypot(result.command.vx, result.command.vy), 0.01, 1e-12);
}

TEST(HolonomicCommandLimiter, rejectsInvalidConfiguration)
{
  auto expect_invalid = [](HolonomicCommandLimiterConfig config) {
      EXPECT_THROW(
        static_cast<void>(HolonomicCommandLimiter{config}),
        std::invalid_argument);
    };

  auto config = makeConfig();
  config.max_vx = 0.0;
  expect_invalid(config);
  config = makeConfig();
  config.max_vy = -1.0;
  expect_invalid(config);
  config = makeConfig();
  config.max_xy_speed = std::numeric_limits<double>::infinity();
  expect_invalid(config);
  config = makeConfig();
  config.max_omega = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(config);
  config = makeConfig();
  config.max_ax = 0.0;
  expect_invalid(config);
  config = makeConfig();
  config.max_ay = -1.0;
  expect_invalid(config);
  config = makeConfig();
  config.max_alpha = std::numeric_limits<double>::infinity();
  expect_invalid(config);
  config = makeConfig();
  config.control_gap_threshold = 0.0;
  expect_invalid(config);
}

TEST(HolonomicCommandLimiter, rejectsNonfiniteCommandsAndInvalidExternalCaps)
{
  HolonomicCommandLimiter limiter(makeConfig());
  const auto finite = command(0.0, 0.0, 0.0);
  auto invalid = finite;
  invalid.vx = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(limiter.seed(invalid), std::domain_error);
  EXPECT_THROW(limiter.limit(invalid, finite, 0.1), std::domain_error);

  invalid = finite;
  invalid.omega = std::numeric_limits<double>::infinity();
  EXPECT_THROW(limiter.limit(finite, invalid, 0.1), std::domain_error);

  EXPECT_THROW(limiter.seed(finite, -0.1), std::domain_error);
  EXPECT_THROW(limiter.limit(finite, finite, 0.1, -0.1), std::domain_error);
  EXPECT_THROW(
    limiter.limit(
      finite, finite, 0.1, -std::numeric_limits<double>::infinity()),
    std::domain_error);
  EXPECT_THROW(
    limiter.limit(
      finite, finite, 0.1, std::numeric_limits<double>::quiet_NaN()),
    std::domain_error);

  EXPECT_NO_THROW(limiter.seed(finite, std::numeric_limits<double>::infinity()));
  EXPECT_NO_THROW(
    limiter.limit(
      finite, finite, 0.1, std::numeric_limits<double>::infinity()));
}

}  // namespace
}  // namespace pid_tracking_controller
