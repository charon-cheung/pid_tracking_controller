#include "motion_observation.hpp"

#include <cmath>

namespace pid_tracking_controller
{
void MotionObservation::resetHistory(uint64_t revision, const char * reason)
{
  history_.odometry = {}; ++history_.epoch; history_.revision = revision;
  history_.reset_reason = reason;
}

void MotionObservation::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = {}; last_ros_now_ = 0; dropped_ = 0;
  resetHistory(0, "activated");
  activated_at_ = Wall::now(); active_ = true;
}

void MotionObservation::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false; latest_ = {}; resetHistory(0, "deactivated");
}

void MotionObservation::record(const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Twist & twist, const std::string & velocity_frame,
  Wall::time_point received)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {++dropped_; return;}
  if (!active_ || received < activated_at_) {return;}
  auto & s = latest_;
  const int64_t stamp = int64_t(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
  const bool backwards = s.count && stamp < s.stamp_ns;
  const bool base = velocity_frame == base_frame_;
  s.frame = base ? "base" : "mismatch";
  s.payload = !std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
    !std::isfinite(twist.angular.z) ? "nonfinite_velocity" :
    !base ? "frame_mismatch" :
    stamp <= 0 || header.stamp.nanosec >= 1000000000U ? "invalid_stamp" :
    backwards ? "header_regressed" : "valid";
  s.vx = twist.linear.x; s.vy = twist.linear.y; s.omega = twist.angular.z;
  s.stamp_ns = stamp; s.received = received; ++s.count;
  auto & series = history_.odometry;
  series.samples[series.next] = s;
  series.next = (series.next + 1) % kHistoryCapacity;
  if (series.size < kHistoryCapacity) {++series.size;} else {++series.overwritten;}
}

MotionObservation::History MotionObservation::history(int64_t ros_now, uint64_t revision)
{
  History out;
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {out.state = "history_busy"; return out;}
  if (active_) {
    if (last_ros_now_ && ros_now < last_ros_now_) {
      latest_ = {}; resetHistory(revision, "clock_rollback"); activated_at_ = Wall::now();
    } else if (revision > history_.revision) {
      resetHistory(revision, "plan_changed");
    }
    last_ros_now_ = ros_now;
  }
  out = history_; out.state = !active_ ? "inactive" :
    revision < history_.revision ? "plan_revision_regressed" : "ready";
  out.dropped = dropped_.load(); out.captured = Wall::now();
  return out;
}
}  // namespace pid_tracking_controller
