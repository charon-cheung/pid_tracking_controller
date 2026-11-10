#include "tracking_progress_publisher.hpp"
#include <iomanip>
#include <random>
#include <sstream>
#include "pid_tracking_controller/path_identity.hpp"

namespace pid_tracking_controller
{
TrackingProgressPublisher::TrackingProgressPublisher(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: clock_(node->get_clock()), publisher_(node->create_publisher<msg::TrackingProgress>(
    "pid_tracking_controller/tracking_progress", rclcpp::QoS(1).reliable()))
{
  std::random_device random;
  std::ostringstream id;
  for (int i = 0; i < 4; ++i) {
    id << std::hex << std::setw(8) << std::setfill('0') << random();
  }
  state_.controller_instance = id.str();
}
void TrackingProgressPublisher::activate() {publisher_->on_activate(); invalidate();}
void TrackingProgressPublisher::deactivate() {invalidate(); publisher_->on_deactivate();}
void TrackingProgressPublisher::invalidate()
{
  state_.valid = false;
  state_.header.stamp = clock_->now();
  if (publisher_->is_activated()) {publisher_->publish(state_);}
}
void TrackingProgressPublisher::loaded(
  const nav_msgs::msg::Path & path, uint64_t revision, PreparedPathPtr prepared)
{
  prepared_ = std::move(prepared);
  state_.path_header = path.header;
  state_.header.frame_id = path.header.frame_id;
  state_.path_fingerprint = path_identity::fingerprintPath(path);
  state_.plan_revision = revision;
  state_.loaded_at = clock_->now();
  invalidate();
}
void TrackingProgressPublisher::update(
  const PathProgressReference & progress,
  const geometry_msgs::msg::PoseStamped & pose, bool rotating)
{
  state_.header.stamp = clock_->now();
  // The prepared cursor also advances through pure rotations and skipped
  // duplicate poses; the legacy translation projection fields do not.
  const bool has_path = prepared_ && !prepared_->poses.empty();
  const bool terminal = has_path && progress.cursor.segment_index >= prepared_->segments.size();
  state_.segment_index = !has_path ? 0 : terminal ? prepared_->poses.size() - 1 :
    prepared_->segments[progress.cursor.segment_index].start_pose_index;
  state_.segment_fraction = terminal ? 0 : progress.cursor.segment_fraction;
  state_.rotating = rotating;
  state_.robot_pose = pose;
  state_.valid = has_path && state_.path_fingerprint != 0 && state_.plan_revision != 0;
  if (publisher_->is_activated()) {publisher_->publish(state_);}
}
}  // namespace pid_tracking_controller
