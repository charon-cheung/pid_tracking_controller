#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "motion_observation.hpp"

namespace pid_tracking_controller
{
namespace
{
using M = MotionObservation;
std_msgs::msg::Header header(int sec = 10)
{
  std_msgs::msg::Header h; h.stamp.sec = sec; h.frame_id = "odom"; return h;
}
geometry_msgs::msg::Twist velocity(double x = .4)
{
  geometry_msgs::msg::Twist t; t.linear.x = x; t.linear.y = -.2; t.angular.z = .1; return t;
}

TEST(MotionObservation, RetainsOriginalTimesAndAllBodyVelocityAxes)
{
  M m("base"); m.activate(); m.history(10000000000LL, 7);
  const auto receipt = M::Wall::now();
  m.record(header(), velocity(), "base", receipt);
  const auto h = m.history(10200000000LL, 7);
  ASSERT_EQ(h.odometry.size, 1u);
  const auto & s = h.odometry.at(0);
  EXPECT_STREQ(h.state, "ready"); EXPECT_EQ(h.revision, 7u);
  EXPECT_EQ(s.stamp_ns, 10000000000LL); EXPECT_EQ(s.received, receipt);
  EXPECT_DOUBLE_EQ(s.vx, .4); EXPECT_DOUBLE_EQ(s.vy, -.2); EXPECT_DOUBLE_EQ(s.omega, .1);
  EXPECT_STREQ(s.payload, "valid"); EXPECT_STREQ(s.frame, "base");
}

TEST(MotionObservation, InvalidSamplesRemainVisibleAfterRecovery)
{
  M m("base"); m.activate(); m.history(10000000000LL, 1);
  m.record(header(), velocity(), "map");
  auto bad = velocity(); bad.linear.y = std::numeric_limits<double>::quiet_NaN();
  m.record(header(), bad, "base");
  m.record(header(11), velocity(), "base");
  m.record(header(), velocity(), "base");
  m.record(header(), velocity(), "base");
  auto malformed = header(); malformed.stamp.nanosec = 1000000000U;
  m.record(malformed, velocity(), "base");
  m.record(header(0), velocity(), "base");
  m.record(header(), velocity(), "");
  const auto h = m.history(10000000000LL, 1);
  ASSERT_EQ(h.odometry.size, 8u);
  const char * reasons[] = {"frame_mismatch", "nonfinite_velocity", "valid", "header_regressed",
    "valid", "invalid_stamp", "invalid_stamp", "frame_mismatch"};
  for (size_t i = 0; i < h.odometry.size; ++i) {
    EXPECT_STREQ(h.odometry.at(i).payload, reasons[i]);
  }
  EXPECT_EQ(h.odometry.at(2).stamp_ns, 11000000000LL);  // Do not rewrite future odometry.
}

TEST(MotionObservation, BoundedHistoryRetainsChronologicalSamples)
{
  M m("base"); m.activate(); m.history(10000000000LL, 7);
  for (size_t i = 0; i < M::kHistoryCapacity + 7; ++i) {
    auto h = header(); h.stamp.nanosec = i;
    m.record(h, velocity(i), "base");
  }
  const auto h = m.history(10100000000LL, 7);
  ASSERT_EQ(h.odometry.size, M::kHistoryCapacity); EXPECT_EQ(h.odometry.overwritten, 7u);
  for (size_t i = 0; i < h.odometry.size; ++i) {
    EXPECT_EQ(h.odometry.at(i).count, i + 8); EXPECT_DOUBLE_EQ(h.odometry.at(i).vx, i + 7);
  }
}

TEST(MotionObservation, PlanAndClockResetDoNotDependOnCandidateRecordingOrLogging)
{
  M m("base"); m.activate(); const auto before = M::Wall::now();
  auto old = m.history(10000000000LL, 7);
  m.record(header(), velocity(), "base");
  auto next = m.history(10000000000LL, 8);
  EXPECT_GT(next.epoch, old.epoch); EXPECT_EQ(next.revision, 8u);
  EXPECT_EQ(next.odometry.size, 0u);
  EXPECT_STREQ(m.history(10000000000LL, 7).state, "plan_revision_regressed");
  m.record(header(), velocity(), "base");
  auto reset = m.history(9000000000LL, 8);
  EXPECT_GT(reset.epoch, next.epoch); EXPECT_EQ(reset.odometry.size, 0u);
  m.record(header(9), velocity(), "base");
  reset = m.history(9000000000LL, 8);
  ASSERT_EQ(reset.odometry.size, 1u); EXPECT_STREQ(reset.odometry.at(0).payload, "valid");
  m.deactivate(); m.record(header(), velocity(), "base");
  EXPECT_STREQ(m.history(10000000000LL, 8).state, "inactive");
  m.activate(); m.history(10000000000LL, 9);
  m.record(header(), velocity(), "base", before);
  auto active = m.history(10000000000LL, 9);
  EXPECT_GT(active.epoch, reset.epoch); EXPECT_EQ(active.odometry.size, 0u);
  m.record(header(), velocity(), "base");
  EXPECT_EQ(m.history(10000000000LL, 9).odometry.size, 1u);
}

TEST(MotionObservation, ConcurrentCopiesNeverMixVelocityComponents)
{
  M m("base"); m.activate(); m.history(10000000000LL, 1);
  std::thread writer([&]() {
    for (int i = 1; i < 10000; ++i) {
      auto t = velocity(i); t.linear.y = -i; t.angular.z = i * 2.;
      m.record(header(), t, "base");
    }
  });
  for (int i = 0; i < 10000; ++i) {
    const auto h = m.history(10000000000LL, 1);
    if (std::string(h.state) != "ready") {continue;}
    for (size_t j = 0; j < h.odometry.size; ++j) {
      const auto & s = h.odometry.at(j);
      EXPECT_EQ(s.vy, -s.vx); EXPECT_EQ(s.omega, s.vx * 2.);
      EXPECT_LE(s.received, h.captured);
    }
  }
  writer.join();
}
}  // namespace
}  // namespace pid_tracking_controller
