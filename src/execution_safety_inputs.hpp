#ifndef PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_INPUTS_HPP_
#define PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_INPUTS_HPP_

#include <atomic>
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
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "path_risk_trigger.hpp"
#include "tf2_ros/buffer.h"

namespace pid_tracking_controller
{
struct SafetyObservationSource
{
  std::string name;
  double max_age{0.6};  // Positive seconds, even if the layer disables its own expiry.
  std::string topic, type;
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
  // Last witnessed successful update, not the snapshot read time or an exact map version.
  std_msgs::msg::Header header;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> map;
  nav2_costmap_2d::Footprint footprint;  // Body frame, including configured padding.
  int64_t pose_stamp_ns{0};
  SweepPose pose;  // Map frame, at the supplied robot pose timestamp.
};

struct SafetyInputResult
{
  std::optional<SafetyInputSnapshot> snapshot;
  std::string reason;
  std::string source;
};

// Collects and validates execution inputs. Grid data comes directly from the
// configured local costmap under its mutex, not a costmap topic. The small
// published_footprint message remains an immutable padded footprint and update
// witness: Jazzy publishes it after updateMap, but exposes no update timestamp
// or synchronized Costmap2DROS footprint getter. Never renew this witness on read.
// Ready means usable inputs, not a clear sweep or proof of source ingestion.
class ExecutionSafetyInputs : public std::enable_shared_from_this<ExecutionSafetyInputs>
{
public:
  using Wall = std::chrono::steady_clock;
  explicit ExecutionSafetyInputs(const SafetyInputOptions & options);
  // Uses the controller's existing executor; no worker, executor or polling thread.
  void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap);
  void recordPathRisk(msg::PathRiskState::ConstSharedPtr risk);
  PathRiskTrigger pathRisk(const msg::TrackingProgress & current, const rclcpp::Time & now);
  void recordFootprint(
    geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint,
    Wall::time_point received = Wall::now());
  void recordObservation(
    const std::string & source, const std_msgs::msg::Header & header,
    bool valid, Wall::time_point received = Wall::now());
  void reset();

  // One capture caller; record* may run concurrently. Never wait for callbacks,
  // a mutex or TF. A busy map may reuse unexpired, previously validated grid
  // evidence; other invalid/expired inputs produce no snapshot. Receipt times
  // must use steady_clock; now and all message headers use the same ROS clock.
  // pose_received is the actual arrival time of this stamped pose, not the time
  // of capture. Reusing a pose must not renew its receipt time.
  SafetyInputResult capture(
    const geometry_msgs::msg::PoseStamped & pose,
    const rclcpp::Time & now, tf2_ros::Buffer & tf, nav2_costmap_2d::LayeredCostmap & costmap,
    Wall::time_point pose_received,
    Wall::time_point wall_now = Wall::now());


private:
  struct Observation
  {
    SafetyObservationSource source;
    std::deque<std::pair<std_msgs::msg::Header, Wall::time_point>> samples;
    int64_t last_stamp{0};
    Wall::time_point received{};
  };
  struct CachedMap
  {
    const nav2_costmap_2d::LayeredCostmap * owner{nullptr};
    std::unique_ptr<nav2_costmap_2d::Costmap2D> map;
    geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
    Wall::time_point received{};
  };
  void clear();  // mutex_ held by caller
  SafetyInputOptions options_;
  std::mutex mutex_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;
  Wall::time_point footprint_received_{};
  int64_t footprint_stamp_{0}, last_now_{0};
  msg::PathRiskState::ConstSharedPtr path_risk_;
  Wall::time_point risk_received_{};
  std::atomic<bool> configuration_changed_{false};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::vector<Observation> observations_;
  uint64_t reset_epoch_{0};  // mutex_ held; reset invalidates the capture-owned cache.
  uint64_t cache_epoch_{0};
  CachedMap cached_map_;  // Only the single capture caller reads/writes this cache.
  std::string last_log_key_;  // Only capture() uses diagnostic state.
};
SafetyInputOptions safetyInputOptions(nav2_costmap_2d::Costmap2DROS & costmap);
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_INPUTS_HPP_
