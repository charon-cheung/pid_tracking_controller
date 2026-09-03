#ifndef PID_TRACKING_CONTROLLER__PREPARED_PATH_HPP_
#define PID_TRACKING_CONTROLLER__PREPARED_PATH_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace pid_tracking_controller
{

enum class PathSegmentType
{
  kTranslation,
  kRotation
};

struct PreparedPathSegment
{
  PathSegmentType type{PathSegmentType::kTranslation};
  std::size_t start_pose_index{0};
  std::size_t end_pose_index{0};
  double translation_s_begin{0.0};
  double translation_s_end{0.0};
  double translation_length{0.0};
  double path_tangent{0.0};
  double target_yaw_begin{0.0};
  double target_yaw_delta{0.0};
  double target_yaw_rate_per_meter{0.0};
};

struct PreparedPath
{
  std::string frame_id;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  std::vector<PreparedPathSegment> segments;
  double total_translation_length{0.0};
};

using PreparedPathPtr = std::shared_ptr<const PreparedPath>;

PreparedPathPtr preparePath(const nav_msgs::msg::Path & path);

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__PREPARED_PATH_HPP_
