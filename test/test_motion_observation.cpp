#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "motion_observation.hpp"

namespace pid_tracking_controller
{
namespace
{
using M = MotionObservation;
using namespace std::chrono_literals;
std_msgs::msg::Header header(int sec = 10)
{
  std_msgs::msg::Header h; h.stamp.sec = sec; h.frame_id = "base"; return h;
}
geometry_msgs::msg::Twist velocity(double x = .4)
{
  geometry_msgs::msg::Twist t; t.linear.x = x; t.linear.y = -.2; t.angular.z = .1; return t;
}

TEST(MotionObservation, IndependentSamplesKeepAllAxesAndOriginalTimes)
{
  M m("base"); m.activate(); const auto now = M::Wall::now();
  m.record(M::Candidate, header(), velocity(), "base", 7, now);
  m.record(M::Published, header(), velocity(.3), "", 0, now + 10ms);
  m.record(M::Smoothed, header(9), velocity(.2), "base", 0, now + 20ms);
  m.record(M::Odometry, header(), velocity(.1), "base", 0, now + 30ms);
  auto v = m.snapshot(10200000000LL, now + 50ms);
  for (const auto & sample : v) {EXPECT_STREQ(sample.state, "valid");}
  EXPECT_EQ(v[0].sample.revision, 7u);
  EXPECT_EQ(v[0].sample.stamp_ns, 10000000000LL);
  EXPECT_EQ(v[2].sample.stamp_ns, 9000000000LL);
  EXPECT_DOUBLE_EQ(v[2].sample.vx, .2); EXPECT_DOUBLE_EQ(v[2].sample.vy, -.2);
  EXPECT_DOUBLE_EQ(v[2].sample.omega, .1); EXPECT_NEAR(v[2].header_age, 1.2, 1e-9);
  EXPECT_NEAR(v[2].receipt_age, .03, 1e-9);
  EXPECT_STREQ(v[1].sample.frame, "empty_assumed_base");
}

TEST(MotionObservation, RepeatedSmootherStampDoesNotRefreshUpstreamHeaderAge)
{
  M m("base"); m.activate(); const auto now = M::Wall::now();
  m.record(M::Smoothed, header(), velocity(), "base", 0, now);
  m.record(M::Smoothed, header(), velocity(), "base", 0, now + 900ms);
  m.record(M::Published, header(), velocity(), "base", 0, now + 900ms);
  auto v = m.snapshot(11000000000LL, now + 1s);
  EXPECT_STREQ(v[2].state, "valid"); EXPECT_NEAR(v[2].header_age, 1., 1e-9);
  EXPECT_NEAR(v[2].receipt_age, .1, 1e-9); EXPECT_EQ(v[2].sample.repeated, 1u);
  EXPECT_STREQ(v[1].state, "header_stale");
  EXPECT_STREQ(m.snapshot(12000000000LL, now + 2s)[2].state, "receipt_stale");
}

TEST(MotionObservation, InvalidEvidenceStaysLocalAndRecovers)
{
  M m("base"); m.activate(); auto now = M::Wall::now();
  m.record(M::Candidate, header(), velocity(), "base");
  m.record(M::Odometry, header(), velocity(), "map");
  EXPECT_STREQ(m.snapshot(10000000000LL)[3].state, "frame_mismatch");
  auto bad = velocity(); bad.linear.y = std::numeric_limits<double>::quiet_NaN();
  m.record(M::Odometry, header(), bad, "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[3].state, "nonfinite_velocity");
  m.record(M::Odometry, header(11), velocity(), "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[3].state, "header_future");
  m.record(M::Odometry, header(), velocity(), "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[3].state, "header_regressed");
  m.record(M::Odometry, header(), velocity(), "base");
  auto v = m.snapshot(10000000000LL);
  EXPECT_STREQ(v[3].state, "valid"); EXPECT_EQ(v[3].sample.regressed, 1u);
  EXPECT_STREQ(v[0].state, "valid"); EXPECT_STREQ(v[1].state, "missing");
  EXPECT_EQ(v[0].sample.count, 1u);
  auto malformed = header(); malformed.stamp.nanosec = 1000000000U;
  m.record(M::Odometry, malformed, velocity(), "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[3].state, "invalid_stamp");
  m.record(M::Odometry, header(0), velocity(), "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[3].state, "invalid_stamp");
  EXPECT_STREQ(m.snapshot(11000000000LL, now + 1s)[0].state, "receipt_stale");
}

TEST(MotionObservation, ReactivationAndClockRollbackDiscardPreviousEvidence)
{
  M m("base"); m.activate(); const auto before = M::Wall::now();
  m.record(M::Candidate, header(), velocity(), "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[0].state, "valid");
  m.report(rclcpp::get_logger("motion_test"), 10000000000LL);
  m.report(rclcpp::get_logger("motion_test"), 9000000000LL);
  EXPECT_STREQ(m.snapshot(9000000000LL)[0].state, "missing");
  m.record(M::Candidate, header(9), velocity(), "base");
  EXPECT_STREQ(m.snapshot(9000000000LL)[0].state, "valid");
  m.deactivate(); m.record(M::Candidate, header(), velocity(), "base");
  m.activate(); m.record(M::Candidate, header(), velocity(), "base", 0, before);
  EXPECT_STREQ(m.snapshot(10000000000LL)[0].state, "missing");
  m.record(M::Candidate, header(), velocity(), "base");
  EXPECT_STREQ(m.snapshot(10000000000LL)[0].state, "valid");
}

TEST(MotionObservation, ConcurrentCopiesNeverMixVelocityComponents)
{
  M m("base"); m.activate();
  std::thread writer([&]() {
    for (int i = 1; i < 10000; ++i) {
      auto t = velocity(i); t.linear.y = -i; t.angular.z = i * 2.;
      m.record(M::Candidate, header(), t, "base");
    }
  });
  bool receipt_clock_invalid = false;
  for (int i = 0; i < 10000; ++i) {
    const auto view = m.snapshot(10000000000LL)[0]; const auto & s = view.sample;
    receipt_clock_invalid |= std::string(view.state) == "receipt_clock_invalid";
    EXPECT_EQ(s.vy, -s.vx); EXPECT_EQ(s.omega, s.vx * 2.);
  }
  writer.join();
  EXPECT_FALSE(receipt_clock_invalid);
}
}  // namespace
}  // namespace pid_tracking_controller
