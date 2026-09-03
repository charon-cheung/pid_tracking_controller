#include "path_progress_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/geometry/vec2d.h"
#include "common/math/math_helper.h"

namespace pid_tracking_controller
{
namespace
{

struct InterpolationResult
{
  geometry_msgs::msg::Point point;
  std::size_t prepared_segment_index{0};
  std::size_t source_segment_index{0};
  double segment_fraction{0.0};
};

bool pointIsFinite(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double planarDistance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

InterpolationResult interpolateAtArcLength(
  const PreparedPath & path,
  double arc_length)
{
  if (path.total_translation_length <= rmp::common::math::kMathEpsilon) {
    return {path.poses.front().pose.position, 0u, 0u, 0.0};
  }

  const double clamped_arc_length = std::clamp(
    arc_length, 0.0, path.total_translation_length);
  for (std::size_t index = 0; index < path.segments.size(); ++index) {
    const auto & segment = path.segments[index];
    if (segment.type != PathSegmentType::kTranslation) {
      continue;
    }
    if (clamped_arc_length > segment.translation_s_end +
      rmp::common::math::kMathEpsilon)
    {
      continue;
    }

    const double fraction = std::clamp(
      (clamped_arc_length - segment.translation_s_begin) / segment.translation_length,
      0.0, 1.0);
    const auto & start = path.poses[segment.start_pose_index].pose.position;
    const auto & end = path.poses[segment.end_pose_index].pose.position;
    geometry_msgs::msg::Point point;
    point.x = start.x + fraction * (end.x - start.x);
    point.y = start.y + fraction * (end.y - start.y);
    point.z = start.z + fraction * (end.z - start.z);
    return {point, index, segment.start_pose_index, fraction};
  }

  for (std::size_t index = path.segments.size(); index > 0u; --index) {
    const auto & segment = path.segments[index - 1u];
    if (segment.type == PathSegmentType::kTranslation) {
      return {
        path.poses[segment.end_pose_index].pose.position,
        index - 1u,
        segment.start_pose_index,
        1.0};
    }
  }
  return {path.poses.front().pose.position, 0u, 0u, 0.0};
}

double curvatureAtSegment(
  const std::vector<geometry_msgs::msg::PoseStamped> & path,
  std::size_t segment_index)
{
  if (segment_index + 2u >= path.size()) {
    return 0.0;
  }

  const auto & first = path[segment_index].pose.position;
  const auto & middle = path[segment_index + 1u].pose.position;
  std::size_t next_index = segment_index + 2u;
  while (next_index < path.size() &&
    planarDistance(middle, path[next_index].pose.position) <=
    rmp::common::math::kMathEpsilon)
  {
    ++next_index;
  }
  if (next_index >= path.size()) {
    return 0.0;
  }

  const auto & last = path[next_index].pose.position;
  const double curvature = rmp::common::math::arcCenter(
    {first.x, first.y}, {middle.x, middle.y}, {last.x, last.y}, false);
  return std::isfinite(curvature) ? curvature : 0.0;
}

bool referenceIsFinite(const PathProgressReference & reference)
{
  return pointIsFinite(reference.projection) && pointIsFinite(reference.carrot) &&
         std::isfinite(reference.cursor.segment_fraction) &&
         std::isfinite(reference.cursor.translation_arc_length) &&
         std::isfinite(reference.segment_fraction) &&
         std::isfinite(reference.progress_arc_length) &&
         std::isfinite(reference.carrot_arc_length) &&
         std::isfinite(reference.remaining_path_length) &&
         std::isfinite(reference.progress_budget) &&
         std::isfinite(reference.tangent) && std::isfinite(reference.curvature) &&
         std::isfinite(reference.signed_lateral_error);
}

}  // namespace

void PathProgressTracker::setPath(PreparedPathPtr path)
{
  if (!path || path->poses.empty()) {
    throw std::invalid_argument("path must not be empty");
  }

  std::vector<double> nonzero_segment_lengths;
  nonzero_segment_lengths.reserve(path->segments.size());
  for (const auto & segment : path->segments) {
    if (segment.type == PathSegmentType::kTranslation) {
      nonzero_segment_lengths.push_back(segment.translation_length);
    }
  }

  double representative_spacing = 0.0;
  if (!nonzero_segment_lengths.empty()) {
    std::sort(nonzero_segment_lengths.begin(), nonzero_segment_lengths.end());
    const std::size_t middle = nonzero_segment_lengths.size() / 2u;
    representative_spacing = nonzero_segment_lengths[middle];
    if (nonzero_segment_lengths.size() % 2u == 0u) {
      representative_spacing = 0.5 *
        (nonzero_segment_lengths[middle - 1u] + nonzero_segment_lengths[middle]);
    }
  }

  path_ = std::move(path);
  progress_arc_length_ = 0.0;
  maximum_progress_budget_ = 2.0 * representative_spacing;
  progress_budget_ = maximum_progress_budget_;
  previous_robot_position_valid_ = false;
  previous_accepted_tangent_valid_ = false;
}

PathProgressReference PathProgressTracker::update(
  const geometry_msgs::msg::Point & robot_position,
  double lookahead_distance)
{
  if (!hasPath()) {
    throw std::logic_error("path progress tracker has no path");
  }
  if (!pointIsFinite(robot_position) || !std::isfinite(lookahead_distance) ||
    lookahead_distance < 0.0)
  {
    throw std::domain_error("robot position and lookahead distance must be finite");
  }

  if (previous_robot_position_valid_ && previous_accepted_tangent_valid_) {
    const double displacement_x = robot_position.x - previous_robot_position_.x;
    const double displacement_y = robot_position.y - previous_robot_position_.y;
    const double forward_displacement = std::max(
      0.0,
      displacement_x * previous_accepted_tangent_x_ +
      displacement_y * previous_accepted_tangent_y_);
    progress_budget_ = std::min(
      maximum_progress_budget_,
      progress_budget_ + forward_displacement);
  }

  PathProgressReference reference;
  reference.frame_id = path_->frame_id;
  if (path_->total_translation_length <= rmp::common::math::kMathEpsilon) {
    reference.projection = path_->poses.front().pose.position;
    reference.carrot = path_->poses.front().pose.position;
    reference.remaining_path_length = 0.0;
    reference.progress_budget = progress_budget_;
    reference.signed_lateral_error = planarDistance(robot_position, reference.projection);
    previous_robot_position_ = robot_position;
    previous_robot_position_valid_ = true;
    if (!referenceIsFinite(reference)) {
      throw std::domain_error("path progress tracker produced a non-finite reference");
    }
    return reference;
  }

  const double admissible_end = std::min(
    path_->total_translation_length, progress_arc_length_ + progress_budget_);
  bool projection_found = false;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_arc_length = progress_arc_length_;
  std::size_t best_prepared_segment_index = 0u;
  double best_segment_fraction = 0.0;
  geometry_msgs::msg::Point best_projection;

  for (std::size_t index = 0; index < path_->segments.size(); ++index) {
    const auto & segment = path_->segments[index];
    if (segment.type != PathSegmentType::kTranslation ||
      segment.translation_s_end + rmp::common::math::kMathEpsilon <
      progress_arc_length_ ||
      segment.translation_s_begin > admissible_end + rmp::common::math::kMathEpsilon)
    {
      continue;
    }

    const double allowed_start = std::max(
      segment.translation_s_begin, progress_arc_length_);
    const double allowed_end = std::min(segment.translation_s_end, admissible_end);
    if (allowed_start > allowed_end + rmp::common::math::kMathEpsilon) {
      continue;
    }

    const auto & start = path_->poses[segment.start_pose_index].pose.position;
    const auto & end = path_->poses[segment.end_pose_index].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    const double minimum_fraction = std::clamp(
      (allowed_start - segment.translation_s_begin) / segment.translation_length,
      0.0, 1.0);
    const double maximum_fraction = std::clamp(
      (allowed_end - segment.translation_s_begin) / segment.translation_length,
      0.0, 1.0);
    const double unconstrained_fraction =
      ((robot_position.x - start.x) * dx + (robot_position.y - start.y) * dy) /
      length_squared;
    const double fraction = std::clamp(
      unconstrained_fraction, minimum_fraction, maximum_fraction);

    geometry_msgs::msg::Point projection;
    projection.x = start.x + fraction * dx;
    projection.y = start.y + fraction * dy;
    projection.z = start.z + fraction * (end.z - start.z);
    const double error_x = robot_position.x - projection.x;
    const double error_y = robot_position.y - projection.y;
    const double distance_squared = error_x * error_x + error_y * error_y;
    const double candidate_arc_length = segment.translation_s_begin +
      fraction * segment.translation_length;
    if (!projection_found ||
      distance_squared < best_distance_squared - rmp::common::math::kMathEpsilon ||
      (std::abs(distance_squared - best_distance_squared) <=
      rmp::common::math::kMathEpsilon && candidate_arc_length < best_arc_length))
    {
      projection_found = true;
      best_distance_squared = distance_squared;
      best_arc_length = candidate_arc_length;
      best_prepared_segment_index = index;
      best_segment_fraction = fraction;
      best_projection = projection;
    }
  }

  if (!projection_found) {
    throw std::domain_error("path progress tracker could not project robot position");
  }

  const double advance = std::max(0.0, best_arc_length - progress_arc_length_);
  progress_arc_length_ = std::max(progress_arc_length_, best_arc_length);
  progress_budget_ = std::max(0.0, progress_budget_ - advance);

  const auto & accepted_segment = path_->segments[best_prepared_segment_index];
  const auto & accepted_start =
    path_->poses[accepted_segment.start_pose_index].pose.position;
  const auto & accepted_end =
    path_->poses[accepted_segment.end_pose_index].pose.position;
  previous_accepted_tangent_x_ = std::cos(accepted_segment.path_tangent);
  previous_accepted_tangent_y_ = std::sin(accepted_segment.path_tangent);
  previous_accepted_tangent_valid_ = true;
  previous_robot_position_ = robot_position;
  previous_robot_position_valid_ = true;

  reference.projection = best_projection;
  reference.cursor.segment_index = best_prepared_segment_index;
  reference.cursor.segment_fraction = best_segment_fraction;
  reference.cursor.translation_arc_length = progress_arc_length_;
  reference.segment_index = accepted_segment.start_pose_index;
  reference.segment_fraction = best_segment_fraction;
  reference.progress_arc_length = progress_arc_length_;
  reference.carrot_arc_length = std::min(
    progress_arc_length_ + lookahead_distance, path_->total_translation_length);
  reference.remaining_path_length = std::max(
    0.0, path_->total_translation_length - reference.progress_arc_length);
  reference.progress_budget = progress_budget_;
  const auto carrot = interpolateAtArcLength(
    *path_, reference.carrot_arc_length);
  reference.carrot = carrot.point;

  const auto & projection_start = accepted_start;
  const auto & projection_end = accepted_end;
  const double projection_dx = projection_end.x - projection_start.x;
  const double projection_dy = projection_end.y - projection_start.y;
  const double relative_x = robot_position.x - reference.projection.x;
  const double relative_y = robot_position.y - reference.projection.y;
  const double cross = projection_dx * relative_y - projection_dy * relative_x;
  const double lateral_distance = std::sqrt(best_distance_squared);
  reference.signed_lateral_error = cross < 0.0 ? -lateral_distance : lateral_distance;

  const auto & carrot_segment = path_->segments[carrot.prepared_segment_index];
  reference.tangent = carrot_segment.path_tangent;
  reference.curvature = curvatureAtSegment(
    path_->poses, carrot.source_segment_index);

  if (!referenceIsFinite(reference)) {
    throw std::domain_error("path progress tracker produced a non-finite reference");
  }
  return reference;
}

bool PathProgressTracker::hasPath() const
{
  return path_ && !path_->poses.empty();
}

}  // namespace pid_tracking_controller
