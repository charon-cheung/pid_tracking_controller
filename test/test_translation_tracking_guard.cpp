#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "translation_tracking_guard.hpp"

namespace pid_tracking_controller
{
namespace
{

TEST(TranslationTrackingGuard, requiresSustainedGrowingOutwardError)
{
  for (const double sign : {-1.0, 1.0}) {
    TranslationTrackingGuard guard;
    EXPECT_FALSE(guard.update(sign * 0.04, sign * 0.08, 0.1));
    EXPECT_FALSE(guard.update(sign * 0.05, sign * 0.08, 0.1));
    EXPECT_FALSE(guard.update(sign * 0.06, sign * 0.08, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.07, sign * 0.08, 0.1));
    // Beginning correction must not immediately reaccelerate.
    EXPECT_TRUE(guard.update(sign * 0.06, -sign * 0.08, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, -sign * 0.08, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, -sign * 0.08, 0.2));
    // Leaving the release band resets confirmation; 0.39 s is not enough.
    EXPECT_TRUE(guard.update(sign * 0.02, 0.0, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, 0.0, 0.1));
    EXPECT_TRUE(guard.update(sign * 0.01, 0.0, 0.2));
    EXPECT_TRUE(guard.update(sign * 0.01, 0.0, 0.19));
    EXPECT_FALSE(guard.update(sign * 0.01, 0.0, 0.011));
  }
}

TEST(TranslationTrackingGuard, observedPositionGrowthConfirmsEntryDespiteVelocityDisagreement)
{
  // Consecutive logged observations from Nov 3 P5 and P20, at about 0.133 s spacing.
  // They reproduce the missed entry, not a full-rate closed-loop bag replay.
  const double errors[2][4] = {
    {0.023276, 0.039943, 0.051860, 0.058744},
    {-0.022124, -0.031872, -0.040379, -0.045795}};
  const double normals[2][4] = {
    {0.060738, 0.049528, 0.007447, 0.000709},
    {-0.035109, -0.038601, -0.015921, 0.002521}};
  for (int scenario = 0; scenario < 2; ++scenario) {
    SCOPED_TRACE(scenario);
    TranslationTrackingGuard guard;
    for (int i = 0; i < 3; ++i) {
      EXPECT_FALSE(guard.update(errors[scenario][i], normals[scenario][i], 0.133));
    }
    EXPECT_TRUE(guard.update(errors[scenario][3], normals[scenario][3], 0.133));
  }
}

TEST(TranslationTrackingGuard, sustainedPositionGrowthWorksOnEitherSideAtDifferentSampleRates)
{
  for (const double sign : {-1.0, 1.0}) {
    for (const double dt : {0.01, 0.05, 0.1}) {
      for (const double measured : {0.0, -0.1}) {
        TranslationTrackingGuard guard;
        EXPECT_FALSE(guard.update(sign * 0.04, sign * measured, dt));
        // Error grows at 0.03 m/s, despite zero or inward velocity feedback.
        bool held = false;
        for (int i = 1; i <= static_cast<int>(0.4 / dt); ++i) {
          held = guard.update(sign * (0.04 + 0.03 * i * dt), sign * measured, dt);
          if ((i - 1) * dt < 0.149) {
            EXPECT_FALSE(held);
          }
        }
        EXPECT_TRUE(held);
      }
    }
  }
}

TEST(TranslationTrackingGuard, outwardVelocityStillConfirmsSlowPositionGrowth)
{
  TranslationTrackingGuard guard;
  EXPECT_FALSE(guard.update(0.0400, 0.08, 0.1));
  EXPECT_FALSE(guard.update(0.0405, 0.08, 0.1));
  EXPECT_FALSE(guard.update(0.0410, 0.08, 0.1));
  EXPECT_TRUE(guard.update(0.0415, 0.08, 0.1));
}

TEST(TranslationTrackingGuard, ignoresSmallErrorRecoveryIsolatedJumpsAndSlowUnconfirmedGrowth)
{
  // Constant large errors, single pose jumps, alternating noise and switching sides
  // must not be mistaken for sustained outward drift.
  const double errors[][5] = {
    {0.002, 0.006, 0.010, 0.014, 0.018},
    {0.080, 0.075, 0.070, 0.065, 0.060},
    {0.040, 0.040, 0.040, 0.040, 0.040},
    {0.010, 0.080, 0.080, 0.080, 0.080},
    {0.040, 0.041, 0.040, 0.041, 0.040},
    {0.040, -0.050, 0.060, -0.070, 0.080}};
  for (const auto & samples : errors) {
    for (const double normal : {-0.1, 0.0, 0.1}) {
      TranslationTrackingGuard guard;
      for (const double error : samples) {
        EXPECT_FALSE(guard.update(error, normal, 0.1));
      }
    }
  }
  for (const double dt : {0.05, 0.1}) {
    TranslationTrackingGuard guard;
    for (int i = 0; i < 10; ++i) {
      // 0.01 m/s error growth is below the fallback threshold; isolated outward
      // velocity samples alone cannot bridge the missing confirmation intervals.
      EXPECT_FALSE(guard.update(0.04 + 0.01 * i * dt, i % 2 ? 0.1 : 0.0, dt));
    }
  }
}

TEST(TranslationTrackingGuard, releaseConfirmationDoesNotDelayEntry)
{
  TranslationTrackingGuardConfig config;
  config.release_confirm_time = 0.8;
  TranslationTrackingGuard guard(config);
  EXPECT_FALSE(guard.update(0.04, 0.08, 0.1));
  EXPECT_FALSE(guard.update(0.05, 0.08, 0.1));
  EXPECT_FALSE(guard.update(0.06, 0.08, 0.1));
  EXPECT_TRUE(guard.update(0.07, 0.08, 0.1));
  EXPECT_TRUE(guard.update(0.01, -0.08, 0.1));
  EXPECT_TRUE(guard.update(0.01, -0.08, 0.5));
  EXPECT_FALSE(guard.update(0.01, -0.08, 0.31));
}

TEST(TranslationTrackingGuard, resetAndInvalidInputs)
{
  TranslationTrackingGuard guard;
  for (int i = 0; i < 5; ++i) {
    guard.update(0.04 + 0.01 * i, 0.1, 0.1);
  }
  guard.reset();
  EXPECT_FALSE(guard.update(0.1, 0.1, 0.1));
  EXPECT_THROW(guard.update(0.1, 0.1, 0.0), std::domain_error);
  EXPECT_THROW(guard.update(0.1, std::numeric_limits<double>::quiet_NaN(), 0.1),
    std::domain_error);
  TranslationTrackingGuardConfig config;
  config.exit_error = config.enter_error;
  EXPECT_THROW(TranslationTrackingGuard{config}, std::invalid_argument);
  config = {};
  config.confirm_time = -0.1;
  EXPECT_THROW(TranslationTrackingGuard{config}, std::invalid_argument);
  for (const double duration : {0.0, -0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    config = {};
    config.release_confirm_time = duration;
    EXPECT_THROW(TranslationTrackingGuard{config}, std::invalid_argument);
  }
}

}  // namespace
}  // namespace pid_tracking_controller
