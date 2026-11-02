#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "translation_tracking_guard.hpp"

namespace pid_tracking_controller
{
namespace
{

TEST(TranslationTrackingGuard, requiresSustainedGrowingOutwardErrorInATurn)
{
  for (const double sign : {-1.0, 1.0}) {
    TranslationTrackingGuard guard;
    EXPECT_FALSE(guard.update(sign * 0.04, sign * 0.08, 1.5, 0.1));
    EXPECT_FALSE(guard.update(sign * 0.05, sign * 0.08, 1.5, 0.1));
    EXPECT_FALSE(guard.update(sign * 0.06, sign * 0.08, 1.5, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.07, sign * 0.08, 1.5, 0.1));
    // Passing the turn or beginning correction must not immediately reaccelerate.
    EXPECT_TRUE(guard.update(sign * 0.06, -sign * 0.08, 0.0, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, -sign * 0.08, 0.0, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.02, 0.0, 0.0, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, 0.0, 0.0, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, 0.0, 0.0, 0.1));
    EXPECT_FALSE(guard.update(sign * 0.01, 0.0, 0.0, 0.1));
  }
}

TEST(TranslationTrackingGuard, ignoresStartupLagStraightMotionInwardMotionAndNoise)
{
  for (int scenario = 0; scenario < 5; ++scenario) {
    TranslationTrackingGuard guard;
    for (int i = 0; i < 10; ++i) {
      const double error = scenario == 4 ? 0.04 : 0.01 * i;
      const double normal = scenario == 0 ? 0.0 : (scenario == 1 ? -0.1 : 0.1);
      const double rate = scenario == 2 ? 0.0 : 1.5;
      // An isolated outward sample every other cycle must not accumulate confirmation.
      const double measured = scenario == 3 && i % 2 == 0 ? 0.0 : normal;
      EXPECT_FALSE(guard.update(error, measured, rate, 0.1));
    }
  }
}

TEST(TranslationTrackingGuard, resetAndInvalidInputs)
{
  TranslationTrackingGuard guard;
  for (int i = 0; i < 5; ++i) {
    guard.update(0.04 + 0.01 * i, 0.1, 1.5, 0.1);
  }
  guard.reset();
  EXPECT_FALSE(guard.update(0.1, 0.1, 1.5, 0.1));
  EXPECT_THROW(guard.update(0.1, 0.1, 1.5, 0.0), std::domain_error);
  EXPECT_THROW(guard.update(0.1, std::numeric_limits<double>::quiet_NaN(), 1.5, 0.1),
    std::domain_error);
  TranslationTrackingGuardConfig config;
  config.exit_error = config.enter_error;
  EXPECT_THROW(TranslationTrackingGuard{config}, std::invalid_argument);
  config = {};
  config.confirm_time = -0.1;
  EXPECT_THROW(TranslationTrackingGuard{config}, std::invalid_argument);
}

}  // namespace
}  // namespace pid_tracking_controller
