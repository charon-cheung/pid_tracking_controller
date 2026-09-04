#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "holonomic_control_law.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

HolonomicControlInput makeInput()
{
  HolonomicControlInput input;
  input.reference_x = 1.0;
  input.path_tangent = 0.0;
  input.target_yaw = 0.0;
  input.reference_velocity = 0.4;
  return input;
}

HolonomicControlGains makeGains()
{
  HolonomicControlGains gains;
  gains.kp_x = 1.0;
  gains.kp_y = 2.0;
  gains.kp_yaw = 3.0;
  return gains;
}

TEST(HolonomicControlLaw, straightAlignedReferenceProducesForwardFeedforward)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.vx_ff, 0.4, 1e-12);
  EXPECT_NEAR(command.vy_ff, 0.0, 1e-12);
  EXPECT_NEAR(command.vx_fb, 0.0, 1e-12);
  EXPECT_NEAR(command.vy_fb, 0.0, 1e-12);
  EXPECT_NEAR(command.omega, 0.0, 1e-12);
  EXPECT_NEAR(command.vx, 0.4, 1e-12);
  EXPECT_NEAR(command.vy, 0.0, 1e-12);
}

TEST(HolonomicControlLaw, worldForwardMotionBecomesBodyLateralAtNinetyDegrees)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.robot_yaw = 0.5 * kPi;
  input.target_yaw = input.robot_yaw;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.vx_ff, 0.0, 1e-12);
  EXPECT_NEAR(command.vy_ff, -0.4, 1e-12);
  EXPECT_NEAR(command.omega, 0.0, 1e-12);
}

TEST(HolonomicControlLaw, forwardPathProgressMayProduceNegativeBodyForwardVelocity)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.robot_yaw = kPi;
  input.target_yaw = input.robot_yaw;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.vx_ff, -0.4, 1e-12);
  EXPECT_NEAR(command.vy_ff, 0.0, 1e-12);
  EXPECT_NEAR(command.omega, 0.0, 1e-12);
}

TEST(HolonomicControlLaw, lateralPositionErrorDoesNotEnterYawChannel)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.robot_y = 0.1;
  input.reference_y = 0.0;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.ex_body, 0.0, 1e-12);
  EXPECT_NEAR(command.ey_body, -0.1, 1e-12);
  EXPECT_NEAR(command.vy_fb, -0.2, 1e-12);
  EXPECT_NEAR(command.yaw_error, 0.0, 1e-12);
  EXPECT_NEAR(command.omega_fb, 0.0, 1e-12);
  EXPECT_NEAR(command.omega, 0.0, 1e-12);
}

TEST(HolonomicControlLaw, targetYawErrorProducesOnlyYawFeedback)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.reference_velocity = 0.0;
  input.target_yaw = 0.2;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.vx, 0.0, 1e-12);
  EXPECT_NEAR(command.vy, 0.0, 1e-12);
  EXPECT_NEAR(command.omega_ff, 0.0, 1e-12);
  EXPECT_NEAR(command.omega_fb, 0.6, 1e-12);
  EXPECT_NEAR(command.omega, 0.6, 1e-12);
}

TEST(HolonomicControlLaw, pathTangentAndTargetYawRemainIndependent)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.target_yaw = 0.5 * kPi;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.vx_ff, 0.4, 1e-12);
  EXPECT_NEAR(command.vy_ff, 0.0, 1e-12);
  EXPECT_NEAR(command.omega_ff, 0.0, 1e-12);
  EXPECT_NEAR(command.omega_fb, 1.5 * kPi, 1e-12);
}

TEST(HolonomicControlLaw, yawErrorUsesShortestAngleAcrossPiBoundary)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.reference_velocity = 0.0;
  input.robot_yaw = 179.0 * kPi / 180.0;
  input.target_yaw = -179.0 * kPi / 180.0;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.yaw_error, 2.0 * kPi / 180.0, 1e-12);
  EXPECT_NEAR(command.omega, 6.0 * kPi / 180.0, 1e-12);
}

TEST(HolonomicControlLaw, yawSpatialRateProducesFeedforward)
{
  auto input = makeInput();
  input.robot_x = input.reference_x;
  input.target_yaw_rate_per_meter = -0.5;

  const auto command = computeHolonomicRawCommand(input, makeGains());

  EXPECT_NEAR(command.omega_ff, -0.2, 1e-12);
  EXPECT_NEAR(command.omega_fb, 0.0, 1e-12);
  EXPECT_NEAR(command.omega, -0.2, 1e-12);
}

TEST(HolonomicControlLaw, rejectsNonfiniteInputsAndInvalidGains)
{
  auto invalid_input = makeInput();
  invalid_input.robot_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    computeHolonomicRawCommand(invalid_input, makeGains()),
    std::domain_error);

  invalid_input = makeInput();
  invalid_input.reference_velocity = -0.1;
  EXPECT_THROW(
    computeHolonomicRawCommand(invalid_input, makeGains()),
    std::domain_error);

  auto invalid_gains = makeGains();
  invalid_gains.kp_y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    computeHolonomicRawCommand(makeInput(), invalid_gains),
    std::invalid_argument);

  invalid_gains = makeGains();
  invalid_gains.kp_yaw = -1.0;
  EXPECT_THROW(
    computeHolonomicRawCommand(makeInput(), invalid_gains),
    std::invalid_argument);
}

}  // namespace
}  // namespace pid_tracking_controller
