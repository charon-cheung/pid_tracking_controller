#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "rotation_completion_tracker.hpp"

namespace pid_tracking_controller
{
namespace
{

RotationCompletionTrackerConfig makeConfig()
{
  RotationCompletionTrackerConfig config;
  config.rotation_yaw_tolerance = 0.10;
  config.rotation_omega_tolerance = 0.20;
  config.rotation_xy_speed_tolerance = 0.50;
  config.rotation_settle_time = 0.30;
  return config;
}

TEST(RotationCompletionTracker, requiresAllInclusiveConditionsContinuously)
{
  RotationCompletionTracker tracker(makeConfig());

  const auto boundary = tracker.update(0.10, 0.30, 0.40, -0.20, 0.10);
  EXPECT_TRUE(boundary.yaw_within_tolerance);
  EXPECT_TRUE(boundary.omega_within_tolerance);
  EXPECT_TRUE(boundary.xy_speed_within_tolerance);
  EXPECT_TRUE(boundary.all_conditions);
  EXPECT_NEAR(boundary.settled_duration, 0.10, 1e-12);
  EXPECT_FALSE(boundary.acknowledge_candidate);

  const auto yaw_violation = tracker.update(0.1001, 0.0, 0.0, 0.0, 0.10);
  EXPECT_FALSE(yaw_violation.yaw_within_tolerance);
  EXPECT_TRUE(yaw_violation.omega_within_tolerance);
  EXPECT_TRUE(yaw_violation.xy_speed_within_tolerance);
  EXPECT_FALSE(yaw_violation.all_conditions);
  EXPECT_DOUBLE_EQ(yaw_violation.settled_duration, 0.0);

  const auto omega_violation = tracker.update(0.0, 0.0, 0.0, 0.2001, 0.10);
  EXPECT_TRUE(omega_violation.yaw_within_tolerance);
  EXPECT_FALSE(omega_violation.omega_within_tolerance);
  EXPECT_TRUE(omega_violation.xy_speed_within_tolerance);
  EXPECT_DOUBLE_EQ(omega_violation.settled_duration, 0.0);

  const auto xy_violation = tracker.update(0.0, 0.3001, 0.40, 0.0, 0.10);
  EXPECT_TRUE(xy_violation.yaw_within_tolerance);
  EXPECT_TRUE(xy_violation.omega_within_tolerance);
  EXPECT_FALSE(xy_violation.xy_speed_within_tolerance);
  EXPECT_DOUBLE_EQ(xy_violation.settled_duration, 0.0);

  EXPECT_FALSE(tracker.update(0.0, 0.0, 0.0, 0.0, 0.15).acknowledge_candidate);
  const auto settled = tracker.update(0.0, 0.0, 0.0, 0.0, 0.15);
  EXPECT_NEAR(settled.settled_duration, 0.30, 1e-12);
  EXPECT_TRUE(settled.acknowledge_candidate);
}

TEST(RotationCompletionTracker, resetAndZeroSettleTimeHaveExplicitBehavior)
{
  auto config = makeConfig();
  config.rotation_settle_time = 0.20;
  RotationCompletionTracker tracker(config);
  EXPECT_FALSE(tracker.update(0.0, 0.0, 0.0, 0.0, 0.10).acknowledge_candidate);
  tracker.reset();
  const auto reset_sample = tracker.update(0.0, 0.0, 0.0, 0.0, 0.10);
  EXPECT_NEAR(reset_sample.settled_duration, 0.10, 1e-12);
  EXPECT_FALSE(reset_sample.acknowledge_candidate);

  config.rotation_settle_time = 0.0;
  RotationCompletionTracker immediate(config);
  const auto immediate_sample = immediate.update(0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(immediate_sample.all_conditions);
  EXPECT_DOUBLE_EQ(immediate_sample.settled_duration, 0.0);
  EXPECT_TRUE(immediate_sample.acknowledge_candidate);
}

TEST(RotationCompletionTracker, rejectsInvalidConfiguration)
{
  auto expect_invalid = [](RotationCompletionTrackerConfig config) {
      EXPECT_THROW(RotationCompletionTracker tracker(config), std::invalid_argument);
    };
  auto config = makeConfig();
  config.rotation_yaw_tolerance = -0.01;
  expect_invalid(config);
  config = makeConfig();
  config.rotation_omega_tolerance = std::numeric_limits<double>::infinity();
  expect_invalid(config);
  config = makeConfig();
  config.rotation_xy_speed_tolerance = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(config);
  config = makeConfig();
  config.rotation_settle_time = -0.01;
  expect_invalid(config);
}

TEST(RotationCompletionTracker, rejectsInvalidInputBeforeMutatingDuration)
{
  auto config = makeConfig();
  config.rotation_settle_time = 0.20;
  RotationCompletionTracker tracker(config);
  ASSERT_FALSE(tracker.update(0.0, 0.0, 0.0, 0.0, 0.10).acknowledge_candidate);

  EXPECT_THROW(
    tracker.update(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.10),
    std::domain_error);
  EXPECT_THROW(tracker.update(0.0, 0.0, 0.0, 0.0, -0.10), std::domain_error);
  EXPECT_THROW(
    tracker.update(0.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.10),
    std::domain_error);
  EXPECT_THROW(tracker.update(4.0, 0.0, 0.0, 0.0, 0.10), std::domain_error);

  const auto settled = tracker.update(0.0, 0.0, 0.0, 0.0, 0.10);
  EXPECT_NEAR(settled.settled_duration, 0.20, 1e-12);
  EXPECT_TRUE(settled.acknowledge_candidate);
}

TEST(RotationCompletionTracker, continuousHugeDtSaturatesDurationWithoutThrowing)
{
  auto config = makeConfig();
  config.rotation_settle_time = std::numeric_limits<double>::max();
  RotationCompletionTracker tracker(config);

  const auto first = tracker.update(
    0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::max());
  EXPECT_EQ(first.settled_duration, std::numeric_limits<double>::max());
  EXPECT_TRUE(first.acknowledge_candidate);

  const auto second = tracker.update(
    0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::max());
  EXPECT_EQ(second.settled_duration, std::numeric_limits<double>::max());
  EXPECT_TRUE(second.acknowledge_candidate);
}

}  // namespace
}  // namespace pid_tracking_controller
