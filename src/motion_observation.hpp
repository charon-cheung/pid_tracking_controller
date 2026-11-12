#ifndef PID_TRACKING_CONTROLLER__MOTION_OBSERVATION_HPP_
#define PID_TRACKING_CONTROLLER__MOTION_OBSERVATION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/logger.hpp"

namespace pid_tracking_controller
{
// Asynchronous evidence only. No command, collision or readiness decision API.
class MotionObservation
{
public:
  using Wall = std::chrono::steady_clock;
  enum Stage {Candidate, Published, Smoothed, Odometry};
  struct Sample
  {
    double vx{0}, vy{0}, omega{0};
    int64_t stamp_ns{0};
    Wall::time_point received{};
    uint64_t count{0}, repeated{0}, regressed{0}, invalid{0}, revision{0};
    const char * payload{"missing"};
    const char * frame{"unknown"};
  };
  struct View
  {
    Sample sample;
    const char * state{"missing"};
    double header_age{-1}, receipt_age{-1};
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
    std::array<Series, 4> stages{};
    const char * state{"inactive"};
    uint64_t epoch{0}, revision{0}, dropped{0};
    std::array<uint64_t, 4> attempts{}, drops{};
    Wall::time_point last_drop{}, captured{};
  };
  explicit MotionObservation(std::string base_frame) : base_frame_(std::move(base_frame)) {}
  void activate();
  void deactivate();
  // Fixed-size copy and try-lock: a busy observer drops evidence, never waits in control.
  void record(Stage stage, const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Twist & twist, const std::string & velocity_frame,
    uint64_t revision = 0, Wall::time_point received = Wall::now());
  std::array<View, 4> snapshot(int64_t ros_now, Wall::time_point now = {});
  History history();  // Bounded try-lock copy for the control cycle or observation timer.
  void report(const rclcpp::Logger & logger, int64_t ros_now);

private:
  void resetHistory(uint64_t revision);  // mutex_ held
  const std::string base_frame_;
  std::mutex mutex_;
  std::array<Sample, 4> samples_{};
  History history_;
  bool active_{false};
  Wall::time_point activated_at_{};
  int64_t last_ros_now_{0};
  std::atomic<uint64_t> dropped_{0};
  std::array<std::atomic<uint64_t>, 4> attempts_{}, drops_{};
  std::atomic<int64_t> last_drop_ns_{0};
  // Only used by the observation timer, reset before activation.
  std::array<View, 4> last_report_{};
  Wall::time_point reported_at_{};
  uint64_t report_id_{0}, reported_drops_{0};
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__MOTION_OBSERVATION_HPP_
