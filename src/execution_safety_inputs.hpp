#ifndef PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_INPUTS_HPP_
#define PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_INPUTS_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <utility>

#include "execution_safety.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/buffer.h"

namespace pid_tracking_controller
{
struct SafetyObservationSource
{
  std::string name;
  double max_age{0.6};  // Positive seconds, even if the layer disables its own expiry.
};

struct SafetyInputOptions
{
  std::string map_frame;
  std::string base_frame;
  std::vector<SafetyObservationSource> sources;
  double max_age{0.6};
  double pose_max_age{0.3};
  double map_update_period{0.1};
  std::size_t max_cells{1000000};
  double budget_seconds{0.01};
};

struct SafetyInputSnapshot
{
  std_msgs::msg::Header header;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> map;
  nav2_costmap_2d::Footprint footprint;  // Body frame, including configured padding.
  SweepPose pose;  // Map frame, at the supplied robot pose timestamp.
};

struct SafetyInputResult
{
  std::optional<SafetyInputSnapshot> snapshot;
  std::string reason;
  std::string source;
};

// Standalone 1D collector. The 1E adapter must feed FULL costmap messages (not
// service responses stamped at request time), the padded footprint and EVERY
// enabled marking/clearing source. Source payload validation precedes recordObservation.
// Reset/reconstruct on lifecycle, source or frame configuration changes. Inputs
// are immutable ROS messages; callers must not mutate them after recording.
// Ready means usable geometric inputs, NOT a clear sweep or proof that the layer
// ingested a particular observation. Velocity decisions remain outside this class.
class ExecutionSafetyInputs
{
public:
  using Wall = std::chrono::steady_clock;
  explicit ExecutionSafetyInputs(const SafetyInputOptions & options);
  void recordMap(
    nav2_msgs::msg::Costmap::ConstSharedPtr map,
    Wall::time_point received = Wall::now());
  void recordFootprint(
    geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint,
    Wall::time_point received = Wall::now());
  void recordObservation(
    const std::string & source, const std_msgs::msg::Header & header,
    bool valid, Wall::time_point received = Wall::now());
  void reset();

  // One capture caller; record* may run concurrently. Never wait for callbacks,
  // a mutex or TF. Busy/invalid/expired inputs produce no snapshot. Receipt times
  // must use steady_clock; now and all message headers use the same ROS clock.
  // pose_received is the actual arrival time of this stamped pose, not the time
  // of capture. Reusing a pose must not renew its receipt time.
  SafetyInputResult capture(
    const geometry_msgs::msg::PoseStamped & pose,
    const rclcpp::Time & now, tf2_ros::Buffer & tf, Wall::time_point pose_received,
    Wall::time_point wall_now = Wall::now());

private:
  struct Observation
  {
    SafetyObservationSource source;
    std::deque<std::pair<std_msgs::msg::Header, Wall::time_point>> samples;
    int64_t last_stamp{0};
    Wall::time_point received{};
  };
  void clear();  // mutex_ held by caller
  SafetyInputOptions options_;
  std::mutex mutex_;
  nav2_msgs::msg::Costmap::ConstSharedPtr map_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;
  Wall::time_point map_received_{}, footprint_received_{};
  int64_t map_stamp_{0}, footprint_stamp_{0}, last_now_{0};
  std::vector<Observation> observations_;
  std::string last_log_key_;  // Only capture() uses diagnostic state.
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_INPUTS_HPP_
