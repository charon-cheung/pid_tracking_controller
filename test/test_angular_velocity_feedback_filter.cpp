#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "angular_velocity_feedback_filter.hpp"

namespace pid_tracking_controller
{

TEST(AngularVelocityFeedbackFilter, smoothsImpulseAndRetainsSustainedSmallRotation)
{
  AngularVelocityFeedbackFilter filter(0.05);
  filter.update(0.0, 1000000000LL);
  filter.update(0.1, 1010000000LL);
  ASSERT_TRUE(filter.sample(1010000000LL));
  EXPECT_NEAR(filter.sample(1010000000LL)->filtered, 0.0181269246922, 1e-12);
  EXPECT_DOUBLE_EQ(filter.sample(1010000000LL)->raw, 0.1);
  for (int i = 2; i <= 101; ++i) {
    filter.update(-0.02, 1000000000LL + i * 10000000LL);
  }
  EXPECT_NEAR(filter.sample(2010000000LL)->filtered, -0.02, 1e-9);
}

TEST(AngularVelocityFeedbackFilter, usesSampleIntervalsNotReadFrequency)
{
  AngularVelocityFeedbackFilter filter(0.05);
  filter.update(0.0, 1000000000LL);
  filter.update(0.5, 1020000000LL);
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(filter.sample(1020000000LL)->filtered, 0.164839976982, 1e-12);
  }
  filter.update(0.5, 1050000000LL);
  EXPECT_NEAR(filter.sample(1050000000LL)->filtered, 0.316060279414, 1e-12);
}

TEST(AngularVelocityFeedbackFilter, reseedsOnDuplicateBackwardAndLongGapStamps)
{
  AngularVelocityFeedbackFilter filter(0.05);
  filter.update(0.6, 1000000000LL);
  EXPECT_DOUBLE_EQ(filter.sample(1000000000LL)->filtered, 0.6);
  filter.update(-0.3, 1000000000LL);
  EXPECT_DOUBLE_EQ(filter.sample(1000000000LL)->filtered, -0.3);
  filter.update(0.2, 990000000LL);
  EXPECT_DOUBLE_EQ(filter.sample(990000000LL)->filtered, 0.2);
  filter.update(-0.4, 1100000000LL);
  EXPECT_DOUBLE_EQ(filter.sample(1100000000LL)->filtered, -0.4);
  filter.reset();
  EXPECT_FALSE(filter.sample(1100000000LL));
}

TEST(AngularVelocityFeedbackFilter, rejectsInvalidFutureAndStaleSamples)
{
  AngularVelocityFeedbackFilter filter(0.05);
  EXPECT_FALSE(filter.sample(1000000000LL));
  filter.update(0.1, 1000000000LL);
  EXPECT_TRUE(filter.sample(1100000000LL));
  EXPECT_FALSE(filter.sample(1100000001LL));
  EXPECT_FALSE(filter.sample(999999999LL));
  filter.update(std::numeric_limits<double>::quiet_NaN(), 1010000000LL);
  EXPECT_FALSE(filter.sample(1010000000LL));
  filter.update(0.2, 1020000000LL);
  EXPECT_DOUBLE_EQ(filter.sample(1020000000LL)->filtered, 0.2);
  filter.update(0.1, 0);
  EXPECT_FALSE(filter.sample(1020000000LL));
  filter.update(0.1, -1);
  EXPECT_FALSE(filter.sample(1020000000LL));
  EXPECT_THROW(AngularVelocityFeedbackFilter(0.0), std::invalid_argument);
  EXPECT_THROW(AngularVelocityFeedbackFilter(-0.1), std::invalid_argument);
  EXPECT_THROW(AngularVelocityFeedbackFilter(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

}  // namespace pid_tracking_controller
