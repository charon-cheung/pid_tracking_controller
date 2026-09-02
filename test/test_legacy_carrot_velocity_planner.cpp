#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "legacy_carrot_velocity_planner.hpp"

namespace pid_tracking_controller
{
namespace
{

LegacyCarrotVelocityPlannerConfig baselineConfig()
{
  LegacyCarrotVelocityPlannerConfig config;
  config.model_k_v = 1.7;
  config.model_k_w = 1.9;
  config.characteristic_length = 0.2;
  config.cruise_speed = 0.51;
  config.max_angular_speed = 0.6;
  config.angular_speed_reserve = 0.1;
  return config;
}

bool allFieldsAreFinite(const LegacyCarrotVelocityPlan & plan)
{
  return std::isfinite(plan.carrot_distance) &&
         std::isfinite(plan.alpha) &&
         std::isfinite(plan.sin_alpha) &&
         std::isfinite(plan.cos_alpha) &&
         std::isfinite(plan.steering_gain_ratio) &&
         std::isfinite(plan.cruise_speed) &&
         std::isfinite(plan.angular_planning_limit) &&
         std::isfinite(plan.angular_limited_speed) &&
         std::isfinite(plan.reference_speed) &&
         std::isfinite(plan.required_angular_at_cruise) &&
         std::isfinite(plan.raw_linear_command) &&
         std::isfinite(plan.raw_angular_command);
}

TEST(LegacyCarrotVelocityPlanner, straightCarrotUsesIndependentCruiseSpeed)
{
  const auto plan = planLegacyCarrotVelocity(0.3, 0.0, baselineConfig());

  EXPECT_NEAR(plan.raw_linear_command, 0.51, 1e-12);
  EXPECT_NEAR(plan.raw_angular_command, 0.0, 1e-12);
  EXPECT_NEAR(plan.reference_speed, 0.51, 1e-12);
  EXPECT_EQ(plan.speed_limit_reason, SpeedLimitReason::kCruise);
}

TEST(LegacyCarrotVelocityPlanner, equalAnglesProduceEqualCommandsAtDifferentDistances)
{
  const auto near_plan = planLegacyCarrotVelocity(0.2819077862357725, 0.1026060429977006,
      baselineConfig());
  const auto far_plan = planLegacyCarrotVelocity(0.8457233587073176, 0.3078181289931018,
      baselineConfig());

  EXPECT_NEAR(near_plan.raw_linear_command, far_plan.raw_linear_command, 1e-12);
  EXPECT_NEAR(near_plan.raw_angular_command, far_plan.raw_angular_command, 1e-12);
}

TEST(LegacyCarrotVelocityPlanner, angularDemandReducesCommonCommandScaleBeforeSaturation)
{
  const auto plan = planLegacyCarrotVelocity(0.2819077862357725, 0.1026060429977006,
      baselineConfig());

  EXPECT_EQ(plan.speed_limit_reason, SpeedLimitReason::kAngular);
  EXPECT_NEAR(plan.angular_planning_limit, 0.5, 1e-12);
  EXPECT_NEAR(plan.angular_limited_speed, 0.261603551593539, 1e-12);
  EXPECT_NEAR(plan.reference_speed, 0.261603551593539, 1e-12);
  EXPECT_NEAR(plan.raw_linear_command, 0.245826927003835, 1e-12);
  EXPECT_NEAR(plan.raw_angular_command, 0.5, 1e-12);
  EXPECT_NEAR(plan.required_angular_at_cruise, 0.974757408478156, 1e-12);
}

TEST(LegacyCarrotVelocityPlanner, zeroDistanceReturnsFiniteZeroCommand)
{
  const auto plan = planLegacyCarrotVelocity(0.0, 0.0, baselineConfig());

  EXPECT_TRUE(allFieldsAreFinite(plan));
  EXPECT_DOUBLE_EQ(plan.raw_linear_command, 0.0);
  EXPECT_DOUBLE_EQ(plan.raw_angular_command, 0.0);
  EXPECT_DOUBLE_EQ(plan.reference_speed, 0.0);
  EXPECT_EQ(plan.speed_limit_reason, SpeedLimitReason::kZeroCarrotDistance);
}

TEST(LegacyCarrotVelocityPlanner, rejectsInvalidConfigurationAndGeometry)
{
  auto invalid_config = baselineConfig();
  invalid_config.model_k_v = 0.0;
  EXPECT_THROW(
    planLegacyCarrotVelocity(0.3, 0.0, invalid_config), std::invalid_argument);

  invalid_config = baselineConfig();
  invalid_config.angular_speed_reserve = invalid_config.max_angular_speed;
  EXPECT_THROW(
    planLegacyCarrotVelocity(0.3, 0.0, invalid_config), std::invalid_argument);

  EXPECT_THROW(
    planLegacyCarrotVelocity(
      std::numeric_limits<double>::quiet_NaN(), 0.0, baselineConfig()),
    std::domain_error);
}

TEST(LegacyCarrotVelocityPlanner, unconstrainedTinySteeringRatioKeepsFiniteCruisePlan)
{
  auto config = baselineConfig();
  config.model_k_v = 1e308;
  config.model_k_w = 2e-10;

  const auto plan = planLegacyCarrotVelocity(0.3, 0.1, config);

  EXPECT_TRUE(allFieldsAreFinite(plan));
  EXPECT_EQ(plan.speed_limit_reason, SpeedLimitReason::kCruise);
  EXPECT_NEAR(plan.reference_speed, 0.51, 1e-12);
}

TEST(LegacyCarrotVelocityPlanner, acceptsFiniteStrictlyPositiveParametersBelowMathEpsilon)
{
  auto config = baselineConfig();
  config.model_k_w = 5e-11;

  const auto plan = planLegacyCarrotVelocity(0.3, 0.0, config);

  EXPECT_TRUE(allFieldsAreFinite(plan));
  EXPECT_EQ(plan.speed_limit_reason, SpeedLimitReason::kCruise);
}

TEST(LegacyCarrotVelocityPlanner, activationValidationDoesNotChangeNoModelConfiguration)
{
  EXPECT_NO_THROW(
    validateLegacyCarrotVelocityPlannerActivation(false, 0.3, baselineConfig()));
}

TEST(LegacyCarrotVelocityPlanner, activationValidationRejectsInvalidModelSpeedLimit)
{
  EXPECT_THROW(
    validateLegacyCarrotVelocityPlannerActivation(
      true, std::numeric_limits<double>::quiet_NaN(), baselineConfig()),
    std::invalid_argument);
  EXPECT_THROW(
    validateLegacyCarrotVelocityPlannerActivation(true, 0.3, baselineConfig()),
    std::invalid_argument);
}

}  // namespace
}  // namespace pid_tracking_controller
