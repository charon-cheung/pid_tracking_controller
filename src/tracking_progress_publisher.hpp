#ifndef PID_TRACKING_CONTROLLER__TRACKING_PROGRESS_PUBLISHER_HPP_
#define PID_TRACKING_CONTROLLER__TRACKING_PROGRESS_PUBLISHER_HPP_

#include "pid_tracking_controller/msg/tracking_progress.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "path_progress_tracker.hpp"

namespace pid_tracking_controller
{
// Optional observation channel; owns no tracking or command decisions.
class TrackingProgressPublisher
{
public:
  explicit TrackingProgressPublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void activate();
  void deactivate();
  void invalidate();
  void loaded(const nav_msgs::msg::Path & path, uint64_t revision, PreparedPathPtr prepared);
  void update(
    const PathProgressReference & progress,
    const geometry_msgs::msg::PoseStamped & pose, bool rotating);

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<msg::TrackingProgress>::SharedPtr publisher_;
  msg::TrackingProgress state_;
  PreparedPathPtr prepared_;
};
}  // namespace pid_tracking_controller
#endif
