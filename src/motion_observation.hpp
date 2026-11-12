#ifndef PID_TRACKING_CONTROLLER__MOTION_OBSERVATION_HPP_
#define PID_TRACKING_CONTROLLER__MOTION_OBSERVATION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace pid_tracking_controller
{
// 2-A: bounded odometry evidence. Candidate commands arrive directly from control.
class MotionObservation
{
public:
  using Wall = std::chrono::steady_clock;
  struct Sample
  {
    double vx{0}, vy{0}, omega{0};
    int64_t stamp_ns{0};
    Wall::time_point received{};
    uint64_t count{0};
    const char * payload{"missing"};
    const char * frame{"unknown"};
  };
  static constexpr size_t kHistoryCapacity = 64;
  struct Series
  {
    std::array<Sample, kHistoryCapacity> samples{};
    size_t size{0}, next{0};
    uint64_t overwritten{0};
    const Sample & at(size_t index) const
    {return samples[(next + kHistoryCapacity - size + index) % kHistoryCapacity];}
  };
  struct History
  {
    Series odometry;
    const char * state{"inactive"};
    const char * reset_reason{"initial"};
    uint64_t epoch{0}, revision{0}, dropped{0};
    Wall::time_point captured{};
  };
  explicit MotionObservation(std::string base_frame) : base_frame_(std::move(base_frame)) {}
  void activate();
  void deactivate();
  // Busy writers drop evidence; the control cycle checks the drop count.
  void record(const std_msgs::msg::Header & header, const geometry_msgs::msg::Twist & twist,
    const std::string & velocity_frame, Wall::time_point received = Wall::now());
  // One control-thread reader; reset on plan/clock changes even with logging disabled.
  // Fixed-size copy and try-lock, never waits for an odometry writer.
  History history(int64_t ros_now, uint64_t revision);

private:
  void resetHistory(uint64_t revision, const char * reason);  // mutex_ held
  const std::string base_frame_;
  std::mutex mutex_;
  Sample latest_;
  History history_;
  bool active_{false};
  Wall::time_point activated_at_{};
  int64_t last_ros_now_{0};
  std::atomic<uint64_t> dropped_{0};
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__MOTION_OBSERVATION_HPP_
