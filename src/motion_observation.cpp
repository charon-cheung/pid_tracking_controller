#include "motion_observation.hpp"

#include <cmath>
#include <cstring>
#include "rclcpp/logging.hpp"

namespace pid_tracking_controller
{
void MotionObservation::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_ = {}; last_report_ = {}; reported_at_ = {}; last_ros_now_ = 0;
  report_id_ = reported_drops_ = 0; dropped_ = 0;
  activated_at_ = Wall::now(); active_ = true;
}

void MotionObservation::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false; samples_ = {};
}

void MotionObservation::record(Stage stage, const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Twist & twist, const std::string & velocity_frame,
  uint64_t revision, Wall::time_point received)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {++dropped_; return;}
  if (!active_ || received < activated_at_) {return;}
  auto & s = samples_[stage];
  const int64_t stamp = int64_t(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
  const bool backwards = s.count && stamp < s.stamp_ns;
  if (s.count && stamp == s.stamp_ns) {++s.repeated;}
  if (backwards) {++s.regressed;}
  s.frame = velocity_frame == base_frame_ ? "base" :
    (velocity_frame.empty() && stage != Odometry) ? "empty_assumed_base" : "mismatch";
  s.payload = !std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
    !std::isfinite(twist.angular.z) ? "nonfinite_velocity" :
    std::strcmp(s.frame, "mismatch") == 0 ? "frame_mismatch" :
    stamp <= 0 || header.stamp.nanosec >= 1000000000U ? "invalid_stamp" :
    backwards ? "header_regressed" : "valid";
  if (std::strcmp(s.payload, "valid") != 0) {++s.invalid;}
  s.vx = twist.linear.x; s.vy = twist.linear.y; s.omega = twist.angular.z;
  s.stamp_ns = stamp; s.received = received; s.revision = revision; ++s.count;
}

std::array<MotionObservation::View, 4> MotionObservation::snapshot(int64_t ros_now, Wall::time_point now)
{
  std::array<View, 4> result;
  std::lock_guard<std::mutex> lock(mutex_);
  // Evaluate after acquiring the cache: a concurrent receipt must not look future-dated.
  if (now == Wall::time_point{}) {now = Wall::now();}
  for (size_t i = 0; i < samples_.size(); ++i) {
    auto & v = result[i]; v.sample = samples_[i];
    if (!v.sample.count) {continue;}
    v.header_age = (ros_now - v.sample.stamp_ns) * 1e-9;
    v.receipt_age = std::chrono::duration<double>(now - v.sample.received).count();
    // 0.6 s / 0.1 s are diagnostic freshness/future tolerances, never control limits.
    // Smoothed headers may retain the upstream command time across many publications.
    v.state = std::strcmp(v.sample.payload, "valid") != 0 ? v.sample.payload :
      v.receipt_age < 0 ? "receipt_clock_invalid" : v.receipt_age > .6 ? "receipt_stale" :
      v.header_age < -.1 ? "header_future" :
      i != Smoothed && v.header_age > .6 ? "header_stale" : "valid";
  }
  return result;
}

void MotionObservation::report(const rclcpp::Logger & logger, int64_t ros_now)
{
  const auto now = Wall::now();
  const bool clock_reset = last_ros_now_ && ros_now < last_ros_now_;
  // Only the worker advances the clock epoch; snapshot readers never mutate it.
  if (clock_reset) {std::lock_guard<std::mutex> lock(mutex_); samples_ = {};}
  last_ros_now_ = ros_now;
  const auto views = snapshot(ros_now);
  const auto drops = dropped_.load();
  bool changed = clock_reset || drops != reported_drops_;
  const auto moving = [](const Sample & s) {
      return std::hypot(s.vx, s.vy) > .02 || std::abs(s.omega) > .02;
    };
  for (size_t i = 0; i < views.size(); ++i) {
    const auto & a = views[i]; const auto & b = last_report_[i];
    changed = changed || std::strcmp(a.state, b.state) != 0 ||
      (a.sample.invalid != b.sample.invalid && now - reported_at_ >= std::chrono::seconds(1)) ||
      a.sample.revision != b.sample.revision ||
      moving(a.sample) != moving(b.sample) ||
      std::hypot(a.sample.vx - b.sample.vx, a.sample.vy - b.sample.vy) >= .1 ||
      std::abs(a.sample.omega - b.sample.omega) >= .1;
  }
  if (report_id_ && !changed && now - reported_at_ < std::chrono::seconds(5)) {return;}
  ++report_id_; reported_at_ = now; last_report_ = views; reported_drops_ = drops;
  RCLCPP_INFO(logger, "PID_MOTION snapshot=%lu mode=observe_only action=none alignment=asynchronous "
    "ros_now=%.9f clock_reset=%d dropped=%lu plan_revision=%lu",
    report_id_, ros_now * 1e-9, clock_reset, drops, views[Candidate].sample.revision);
  const char * names[] = {"candidate", "published", "smoothed", "odometry"};
  for (size_t i = 0; i < views.size(); ++i) {
    const auto & v = views[i]; const auto & s = v.sample;
    RCLCPP_INFO(logger, "PID_MOTION snapshot=%lu stage=%s state=%s count=%lu "
      "vx=%.6f vy=%.6f omega=%.6f frame=%s stamp=%.9f header_age=%.3f receipt_age=%.3f "
      "receipt_steady=%.9f repeated=%lu regressed=%lu invalid=%lu",
      report_id_, names[i], v.state, s.count, s.vx, s.vy, s.omega, s.frame, s.stamp_ns * 1e-9,
      v.header_age, v.receipt_age, std::chrono::duration<double>(s.received.time_since_epoch()).count(),
      s.repeated, s.regressed, s.invalid);
  }
}
}  // namespace pid_tracking_controller
