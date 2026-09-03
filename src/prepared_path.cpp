#include "prepared_path.hpp"

#include <cmath>
#include <stdexcept>

#include "angles/angles.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kSamePositionTolerance = 1e-4;
constexpr double kSameYawTolerance = 1e-6;
constexpr double kQuaternionNormEpsilon = 1e-12;

bool positionIsFinite(const geometry_msgs::msg::Point & position)
{
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z);
}

bool quaternionIsFinite(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

double normalizeQuaternion(geometry_msgs::msg::Quaternion & quaternion)
{
  if (!quaternionIsFinite(quaternion)) {
    throw std::invalid_argument("path quaternions must be finite");
  }

  const double norm = std::hypot(
    std::hypot(quaternion.x, quaternion.y),
    std::hypot(quaternion.z, quaternion.w));
  if (!std::isfinite(norm) || norm <= kQuaternionNormEpsilon) {
    throw std::invalid_argument("path quaternions must have nonzero finite norm");
  }

  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;

  const tf2::Quaternion tf_quaternion(
    quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  const double yaw = angles::normalize_angle(tf2::getYaw(tf_quaternion));
  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("path quaternion yaw must be finite");
  }
  return yaw;
}

}  // namespace

PreparedPathPtr preparePath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw std::invalid_argument("path must not be empty");
  }

  auto prepared = std::make_shared<PreparedPath>();
  prepared->frame_id = path.header.frame_id;
  prepared->poses = path.poses;

  std::vector<double> target_yaws;
  target_yaws.reserve(prepared->poses.size());
  for (auto & pose : prepared->poses) {
    if (!positionIsFinite(pose.pose.position)) {
      throw std::invalid_argument("path positions must be finite");
    }
    target_yaws.push_back(normalizeQuaternion(pose.pose.orientation));
    if (prepared->frame_id.empty() && !pose.header.frame_id.empty()) {
      prepared->frame_id = pose.header.frame_id;
    }
  }

  prepared->segments.reserve(prepared->poses.size() - 1u);
  double translation_arc_length = 0.0;
  for (std::size_t index = 0; index + 1u < prepared->poses.size(); ++index) {
    const auto & start = prepared->poses[index].pose.position;
    const auto & end = prepared->poses[index + 1u].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double translation_length = std::hypot(dx, dy);
    if (!std::isfinite(translation_length)) {
      throw std::invalid_argument("path translation lengths must be finite");
    }

    const double target_yaw_delta = angles::normalize_angle(
      target_yaws[index + 1u] - target_yaws[index]);
    if (translation_length <= kSamePositionTolerance &&
      std::abs(target_yaw_delta) <= kSameYawTolerance)
    {
      continue;
    }

    PreparedPathSegment segment;
    segment.start_pose_index = index;
    segment.end_pose_index = index + 1u;
    segment.translation_s_begin = translation_arc_length;
    segment.target_yaw_begin = target_yaws[index];
    segment.target_yaw_delta = target_yaw_delta;

    if (translation_length > kSamePositionTolerance) {
      segment.type = PathSegmentType::kTranslation;
      segment.translation_length = translation_length;
      translation_arc_length += translation_length;
      if (!std::isfinite(translation_arc_length)) {
        throw std::invalid_argument("path cumulative translation length must be finite");
      }
      segment.translation_s_end = translation_arc_length;
      segment.path_tangent = std::atan2(dy, dx);
      segment.target_yaw_rate_per_meter = target_yaw_delta / translation_length;
    } else {
      segment.type = PathSegmentType::kRotation;
      segment.translation_s_end = translation_arc_length;
    }

    prepared->segments.push_back(segment);
  }

  prepared->total_translation_length = translation_arc_length;
  return prepared;
}

}  // namespace pid_tracking_controller
