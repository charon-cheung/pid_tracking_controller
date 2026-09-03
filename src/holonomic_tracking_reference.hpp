#ifndef PID_TRACKING_CONTROLLER__HOLONOMIC_TRACKING_REFERENCE_HPP_
#define PID_TRACKING_CONTROLLER__HOLONOMIC_TRACKING_REFERENCE_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "path_progress_tracker.hpp"
#include "prepared_path.hpp"

namespace pid_tracking_controller
{

enum class HolonomicReferenceMode
{
  kTranslation,
  kRotationPending,
  kTerminal
};

struct HolonomicPathSample
{
  geometry_msgs::msg::Point position;
  PathCursor cursor;
  double path_tangent{0.0};
  double target_yaw{0.0};
  double target_yaw_rate_per_meter{0.0};
};

struct HolonomicTrackingReference
{
  HolonomicReferenceMode mode{HolonomicReferenceMode::kTerminal};
  HolonomicPathSample current;
  HolonomicPathSample preview;
  bool preview_clamped_by_rotation{false};
  double rotation_target_yaw{0.0};
};

HolonomicTrackingReference buildHolonomicTrackingReference(
  const PreparedPath & path,
  const PathProgressReference & progress,
  double lookahead_distance);

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__HOLONOMIC_TRACKING_REFERENCE_HPP_
