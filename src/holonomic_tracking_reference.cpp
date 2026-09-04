#include "holonomic_tracking_reference.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "angles/angles.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kGeometryEpsilon = 1e-9;

bool pointIsFinite(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & orientation = pose.pose.orientation;
  const tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);
  return angles::normalize_angle(tf2::getYaw(quaternion));
}

HolonomicPathSample sampleTranslation(
  const PreparedPath & path,
  std::size_t segment_index,
  double segment_fraction,
  double translation_arc_length,
  const geometry_msgs::msg::Point * position_override = nullptr)
{
  if (segment_index >= path.segments.size()) {
    throw std::out_of_range("holonomic reference cursor segment is out of range");
  }
  const auto & segment = path.segments[segment_index];
  if (segment.type != PathSegmentType::kTranslation) {
    throw std::invalid_argument("holonomic path sample requires a translation segment");
  }
  if (!std::isfinite(segment_fraction) ||
    segment_fraction < -kGeometryEpsilon || segment_fraction > 1.0 + kGeometryEpsilon)
  {
    throw std::domain_error("holonomic reference segment fraction is invalid");
  }

  const double fraction = std::clamp(segment_fraction, 0.0, 1.0);
  const auto & start = path.poses[segment.start_pose_index].pose.position;
  const auto & end = path.poses[segment.end_pose_index].pose.position;

  HolonomicPathSample sample;
  if (position_override) {
    sample.position = *position_override;
  } else {
    sample.position.x = start.x + fraction * (end.x - start.x);
    sample.position.y = start.y + fraction * (end.y - start.y);
    sample.position.z = start.z + fraction * (end.z - start.z);
  }
  sample.cursor.segment_index = segment_index;
  sample.cursor.segment_fraction = fraction;
  sample.cursor.translation_arc_length = translation_arc_length;
  sample.path_tangent = segment.path_tangent;
  sample.target_yaw = angles::normalize_angle(
    segment.target_yaw_begin + fraction * segment.target_yaw_delta);
  sample.target_yaw_rate_per_meter = segment.target_yaw_rate_per_meter;
  return sample;
}

HolonomicPathSample sampleSinglePose(const PreparedPath & path)
{
  HolonomicPathSample sample;
  sample.position = path.poses.front().pose.position;
  sample.target_yaw = poseYaw(path.poses.front());
  return sample;
}

HolonomicPathSample sampleRotationBoundary(
  const PreparedPath & path,
  std::size_t rotation_segment_index,
  std::size_t prepared_segment_lower_bound)
{
  const auto & rotation = path.segments[rotation_segment_index];
  for (std::size_t index = rotation_segment_index;
    index > prepared_segment_lower_bound; --index)
  {
    const auto & segment = path.segments[index - 1u];
    if (segment.type == PathSegmentType::kTranslation &&
      std::abs(segment.translation_s_end - rotation.translation_s_begin) <=
      kGeometryEpsilon)
    {
      return sampleTranslation(
        path, index - 1u, 1.0, rotation.translation_s_begin);
    }
  }

  HolonomicPathSample sample;
  sample.position = path.poses[rotation.start_pose_index].pose.position;
  sample.cursor.segment_index = rotation_segment_index;
  sample.cursor.translation_arc_length = rotation.translation_s_begin;
  sample.target_yaw = angles::normalize_angle(rotation.target_yaw_begin);
  return sample;
}

HolonomicPathSample sampleTerminal(const PreparedPath & path)
{
  HolonomicPathSample sample;
  sample.position = path.poses.back().pose.position;
  sample.cursor.segment_index = path.segments.size();
  sample.cursor.translation_arc_length = path.total_translation_length;
  sample.target_yaw = poseYaw(path.poses.back());
  for (std::size_t index = path.segments.size(); index > 0u; --index) {
    const auto & segment = path.segments[index - 1u];
    if (segment.type == PathSegmentType::kTranslation) {
      sample.path_tangent = segment.path_tangent;
      break;
    }
  }
  return sample;
}

std::optional<std::size_t> firstRotationThroughPreview(
  const PreparedPath & path,
  const PathCursor & current_cursor,
  std::size_t prepared_segment_lower_bound,
  double preview_arc_length)
{
  for (std::size_t index = std::max(
      current_cursor.segment_index + 1u, prepared_segment_lower_bound);
    index < path.segments.size(); ++index)
  {
    const auto & segment = path.segments[index];
    if (segment.translation_s_begin > preview_arc_length + kGeometryEpsilon) {
      break;
    }
    if (segment.type == PathSegmentType::kRotation &&
      segment.translation_s_begin + kGeometryEpsilon >=
      current_cursor.translation_arc_length)
    {
      return index;
    }
  }
  return std::nullopt;
}

std::optional<std::size_t> firstReachedRotation(
  const PreparedPath & path,
  const PathProgressReference & progress)
{
  if (progress.prepared_segment_lower_bound >= path.segments.size()) {
    return std::nullopt;
  }

  for (std::size_t index = progress.prepared_segment_lower_bound;
    index < path.segments.size(); ++index)
  {
    const auto & segment = path.segments[index];
    if (!translationArcHasReachedRotationBoundary(
        progress.cursor.translation_arc_length, segment.translation_s_begin))
    {
      return std::nullopt;
    }
    if (segment.type != PathSegmentType::kRotation) {
      continue;
    }
    if (index <= progress.cursor.segment_index) {
      return index;
    }
    if (progress.cursor.segment_index < path.segments.size() &&
      path.segments[progress.cursor.segment_index].type == PathSegmentType::kTranslation &&
      index == progress.cursor.segment_index + 1u)
    {
      return index;
    }
    return std::nullopt;
  }
  return std::nullopt;
}

double rotationRunTargetYaw(
  const PreparedPath & path,
  std::size_t first_rotation_index)
{
  const double boundary = path.segments[first_rotation_index].translation_s_begin;
  std::size_t last_rotation_index = first_rotation_index;
  while (last_rotation_index + 1u < path.segments.size()) {
    const auto & next = path.segments[last_rotation_index + 1u];
    if (next.type != PathSegmentType::kRotation ||
      std::abs(next.translation_s_begin - boundary) > kGeometryEpsilon ||
      std::abs(next.translation_s_end - boundary) > kGeometryEpsilon)
    {
      break;
    }
    ++last_rotation_index;
  }

  const auto & last_rotation = path.segments[last_rotation_index];
  return angles::normalize_angle(
    last_rotation.target_yaw_begin + last_rotation.target_yaw_delta);
}

HolonomicPathSample sampleAtArcLength(
  const PreparedPath & path,
  const PathCursor & current_cursor,
  double translation_arc_length)
{
  for (std::size_t index = current_cursor.segment_index;
    index < path.segments.size(); ++index)
  {
    const auto & segment = path.segments[index];
    if (segment.type != PathSegmentType::kTranslation) {
      continue;
    }
    if (translation_arc_length > segment.translation_s_end + kGeometryEpsilon) {
      continue;
    }
    if (translation_arc_length + kGeometryEpsilon < segment.translation_s_begin) {
      break;
    }

    const double fraction = std::clamp(
      (translation_arc_length - segment.translation_s_begin) /
      segment.translation_length,
      0.0, 1.0);
    return sampleTranslation(path, index, fraction, translation_arc_length);
  }

  for (std::size_t index = path.segments.size(); index > 0u; --index) {
    const auto & segment = path.segments[index - 1u];
    if (segment.type == PathSegmentType::kTranslation) {
      return sampleTranslation(
        path, index - 1u, 1.0, path.total_translation_length);
    }
  }
  throw std::logic_error("prepared path contains no translation segment");
}

bool sampleIsFinite(const HolonomicPathSample & sample)
{
  return pointIsFinite(sample.position) &&
         std::isfinite(sample.cursor.segment_fraction) &&
         std::isfinite(sample.cursor.translation_arc_length) &&
         std::isfinite(sample.path_tangent) &&
         std::isfinite(sample.target_yaw) &&
         std::isfinite(sample.target_yaw_rate_per_meter);
}

bool referenceIsFinite(const HolonomicTrackingReference & reference)
{
  return sampleIsFinite(reference.current) && sampleIsFinite(reference.preview) &&
         std::isfinite(reference.rotation_target_yaw);
}

}  // namespace

HolonomicTrackingReference buildHolonomicTrackingReference(
  const PreparedPath & path,
  const PathProgressReference & progress,
  double lookahead_distance)
{
  if (path.poses.empty()) {
    throw std::invalid_argument("prepared path must not be empty");
  }
  if (!pointIsFinite(progress.projection) || !std::isfinite(lookahead_distance) ||
    lookahead_distance < 0.0)
  {
    throw std::domain_error("holonomic reference inputs must be finite");
  }
  if (progress.prepared_segment_lower_bound > path.segments.size()) {
    throw std::invalid_argument("prepared segment lower bound is out of range");
  }

  HolonomicTrackingReference reference;
  if (progress.prepared_segment_lower_bound == path.segments.size()) {
    reference.current = sampleTerminal(path);
    reference.preview = reference.current;
    reference.mode = HolonomicReferenceMode::kTerminal;
    if (!referenceIsFinite(reference)) {
      throw std::domain_error("holonomic reference contains non-finite values");
    }
    return reference;
  }

  if (path.total_translation_length <= kGeometryEpsilon) {
    const auto pending_rotation = firstReachedRotation(path, progress);
    if (pending_rotation) {
      reference.current = sampleRotationBoundary(
        path, *pending_rotation, progress.prepared_segment_lower_bound);
      reference.preview = reference.current;
      reference.mode = HolonomicReferenceMode::kRotationPending;
      reference.preview_clamped_by_rotation = true;
      reference.rotation_target_yaw = rotationRunTargetYaw(path, *pending_rotation);
      reference.pending_rotation_segment_index = pending_rotation;
    } else {
      reference.current = sampleSinglePose(path);
      reference.preview = reference.current;
    }
    if (!referenceIsFinite(reference)) {
      throw std::domain_error("holonomic reference contains non-finite values");
    }
    return reference;
  }

  const auto & cursor = progress.cursor;
  if (cursor.segment_index < path.segments.size() &&
    path.segments[cursor.segment_index].type == PathSegmentType::kRotation)
  {
    const auto pending_rotation = firstReachedRotation(path, progress);
    if (!pending_rotation) {
      throw std::invalid_argument("rotation cursor has no pending rotation");
    }
    reference.current = sampleRotationBoundary(
      path, *pending_rotation, progress.prepared_segment_lower_bound);
    reference.preview = reference.current;
    reference.mode = HolonomicReferenceMode::kRotationPending;
    reference.preview_clamped_by_rotation = true;
    reference.rotation_target_yaw = rotationRunTargetYaw(path, *pending_rotation);
    reference.pending_rotation_segment_index = pending_rotation;
    if (!referenceIsFinite(reference)) {
      throw std::domain_error("holonomic reference contains non-finite values");
    }
    return reference;
  }
  if (cursor.segment_index >= path.segments.size() ||
    path.segments[cursor.segment_index].type != PathSegmentType::kTranslation ||
    !std::isfinite(cursor.translation_arc_length))
  {
    throw std::invalid_argument("accepted progress cursor is not a translation segment");
  }

  reference.current = sampleTranslation(
    path, cursor.segment_index, cursor.segment_fraction,
    cursor.translation_arc_length, &progress.projection);

  const double desired_preview_arc_length = std::min(
    cursor.translation_arc_length + lookahead_distance,
    path.total_translation_length);
  const auto rotation_index = firstRotationThroughPreview(
    path, cursor, progress.prepared_segment_lower_bound, desired_preview_arc_length);
  if (rotation_index) {
    const auto & rotation = path.segments[*rotation_index];
    const std::size_t preceding_segment_index = *rotation_index - 1u;
    reference.preview = sampleTranslation(
      path, preceding_segment_index, 1.0, rotation.translation_s_begin);
    reference.preview_clamped_by_rotation = true;
    reference.rotation_target_yaw = rotationRunTargetYaw(path, *rotation_index);
  } else {
    reference.preview = sampleAtArcLength(
      path, cursor, desired_preview_arc_length);
  }

  const auto pending_rotation = firstReachedRotation(path, progress);
  const bool at_current_segment_end =
    cursor.segment_fraction >= 1.0 - kGeometryEpsilon;
  if (pending_rotation) {
    reference.mode = HolonomicReferenceMode::kRotationPending;
    reference.preview = sampleRotationBoundary(
      path, *pending_rotation, progress.prepared_segment_lower_bound);
    reference.preview_clamped_by_rotation = true;
    reference.rotation_target_yaw = rotationRunTargetYaw(path, *pending_rotation);
    reference.pending_rotation_segment_index = pending_rotation;
  } else if (at_current_segment_end &&
    cursor.segment_index + 1u == path.segments.size())
  {
    reference.mode = HolonomicReferenceMode::kTerminal;
  } else {
    reference.mode = HolonomicReferenceMode::kTranslation;
  }

  if (!referenceIsFinite(reference)) {
    throw std::domain_error("holonomic reference contains non-finite values");
  }
  return reference;
}

}  // namespace pid_tracking_controller
