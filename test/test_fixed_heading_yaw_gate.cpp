#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "fixed_heading_yaw_gate.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kDegree = 0.017453292519943295;

HolonomicTrackingReference fixedReference()
{
  HolonomicTrackingReference reference;
  reference.mode = HolonomicReferenceMode::kTranslation;
  return reference;
}

HolonomicRawCommand commandWithError(double error)
{
  HolonomicRawCommand command;
  command.yaw_error = error;
  command.omega = command.omega_fb = error;
  command.vx = 0.7;
  command.vy = -0.3;
  return command;
}

TEST(FixedHeadingYawGate, holdsBetweenThresholdsInBothDirections)
{
  const FixedHeadingYawGateConfig config;
  for (const double sign : {-1.0, 1.0}) {
    FixedHeadingYawGate gate;
    const auto check = [&](double error, double expected) {
        auto command = commandWithError(sign * error);
        gate.apply(fixedReference(), command);
        EXPECT_DOUBLE_EQ(command.omega, sign * expected);
        EXPECT_DOUBLE_EQ(command.omega_fb, sign * expected);
        EXPECT_DOUBLE_EQ(command.vx, 0.7);
        EXPECT_DOUBLE_EQ(command.vy, -0.3);
      };
    check(1.5 * kDegree, 0.0);
    check(config.enter_error, config.enter_error);
    check(1.5 * kDegree, 1.5 * kDegree);
    check(config.exit_error, 0.0);
    check(1.5 * kDegree, 0.0);
  }
}

TEST(FixedHeadingYawGate, phaseAndChangingHeadingBypassAndResetCorrection)
{
  for (const auto mode : {HolonomicReferenceMode::kRotationPending,
    HolonomicReferenceMode::kTerminal, HolonomicReferenceMode::kTranslation})
  {
    FixedHeadingYawGate gate;
    auto command = commandWithError(3.0 * kDegree);
    gate.apply(fixedReference(), command);
    auto reference = fixedReference();
    reference.mode = mode;
    // Even a small intentional heading rate must bypass the gate.
    reference.current.target_yaw_rate_per_meter =
      mode == HolonomicReferenceMode::kTranslation ? 1e-6 : 0.0;
    command = commandWithError(0.5 * kDegree);
    command.omega_ff = 0.0001;
    command.omega += command.omega_ff;
    const auto expected = command.omega;
    gate.apply(reference, command);
    EXPECT_DOUBLE_EQ(command.omega, expected);
    EXPECT_DOUBLE_EQ(command.omega_ff, 0.0001);
    command = commandWithError(1.5 * kDegree);
    gate.apply(fixedReference(), command);
    EXPECT_DOUBLE_EQ(command.omega, 0.0);
  }
}

TEST(FixedHeadingYawGate, resetAndNewFixedTargetClearOldCorrection)
{
  FixedHeadingYawGate gate;
  auto command = commandWithError(3.0 * kDegree);
  gate.apply(fixedReference(), command);
  gate.reset();
  command = commandWithError(1.5 * kDegree);
  gate.apply(fixedReference(), command);
  EXPECT_DOUBLE_EQ(command.omega, 0.0);
  command = commandWithError(3.0 * kDegree);
  gate.apply(fixedReference(), command);
  auto reference = fixedReference();
  reference.current.target_yaw = 1.0;
  command = commandWithError(1.5 * kDegree);
  gate.apply(reference, command);
  EXPECT_DOUBLE_EQ(command.omega, 0.0);
}

TEST(FixedHeadingYawGate, wrappedTargetAndSignedErrorsPreserveHysteresis)
{
  FixedHeadingYawGate gate;
  auto reference = fixedReference();
  reference.current.target_yaw = std::acos(-1.0);
  auto command = commandWithError(3.0 * kDegree);
  gate.apply(reference, command);
  reference.current.target_yaw = -std::acos(-1.0);
  command = commandWithError(-1.5 * kDegree);
  gate.apply(reference, command);
  EXPECT_DOUBLE_EQ(command.omega, -1.5 * kDegree);
  command = commandWithError(-0.5 * kDegree);
  gate.apply(reference, command);
  EXPECT_DOUBLE_EQ(command.omega, 0.0);
}

TEST(FixedHeadingYawGate, configurableThresholdsAndDisableTakeEffect)
{
  FixedHeadingYawGate gate({true, 0.1, 0.05});
  auto command = commandWithError(0.06);
  gate.apply(fixedReference(), command);
  EXPECT_DOUBLE_EQ(command.omega, 0.0);
  command = commandWithError(0.11);
  gate.apply(fixedReference(), command);
  EXPECT_DOUBLE_EQ(command.omega, 0.11);
  FixedHeadingYawGate disabled({false, 0.1, 0.05});
  command = commandWithError(0.01);
  disabled.apply(fixedReference(), command);
  EXPECT_DOUBLE_EQ(command.omega, 0.01);
}

TEST(FixedHeadingYawGate, rejectsInvalidThresholds)
{
  for (const auto config : {FixedHeadingYawGateConfig{true, 0.02, 0.02},
    FixedHeadingYawGateConfig{true, 0.01, 0.02},
    FixedHeadingYawGateConfig{true, 0.02, 0.0},
    FixedHeadingYawGateConfig{true, 4.0, 0.01},
    FixedHeadingYawGateConfig{true, std::numeric_limits<double>::quiet_NaN(), 0.01},
    FixedHeadingYawGateConfig{true, 0.02, std::numeric_limits<double>::infinity()}})
  {
    EXPECT_THROW((void)FixedHeadingYawGate(config), std::invalid_argument);
  }
}

}  // namespace
}  // namespace pid_tracking_controller
