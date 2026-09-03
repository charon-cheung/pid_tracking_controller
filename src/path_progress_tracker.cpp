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
  std::size_t segment_index{0};
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
  const std::vector<geometry_msgs::msg::PoseStamped> & path,
  const std::vector<double> & cumulative_lengths,
  double total_path_length,
  double arc_length)
{
  if (path.size() == 1u || total_path_length <= rmp::common::math::kMathEpsilon) {
    return {path.front().pose.position, 0u, 0.0};
  }

  const double clamped_arc_length = std::clamp(arc_length, 0.0, total_path_length);
  for (std::size_t index = 0; index + 1u < path.size(); ++index) {
    const double segment_length = cumulative_lengths[index + 1u] -
      cumulative_lengths[index];
    if (segment_length <= rmp::common::math::kMathEpsilon) {
      continue;
    }
    if (clamped_arc_length > cumulative_lengths[index + 1u] +
      rmp::common::math::kMathEpsilon)
    {
      continue;
    }

    const double fraction = std::clamp(
      (clamped_arc_length - cumulative_lengths[index]) / segment_length,
      0.0, 1.0);
    const auto & start = path[index].pose.position;
    const auto & end = path[index + 1u].pose.position;
    geometry_msgs::msg::Point point;
    point.x = start.x + fraction * (end.x - start.x);
    point.y = start.y + fraction * (end.y - start.y);
    point.z = start.z + fraction * (end.z - start.z);
    return {point, index, fraction};
  }

  for (std::size_t index = path.size() - 1u; index > 0u; --index) {
    if (cumulative_lengths[index] - cumulative_lengths[index - 1u] >
      rmp::common::math::kMathEpsilon)
    {
      return {path[index].pose.position, index - 1u, 1.0};
    }
  }
  return {path.front().pose.position, 0u, 0.0};
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
         std::isfinite(reference.segment_fraction) &&
         std::isfinite(reference.progress_arc_length) &&
         std::isfinite(reference.carrot_arc_length) &&
         std::isfinite(reference.tangent) && std::isfinite(reference.curvature) &&
         std::isfinite(reference.signed_lateral_error);
}

}  // namespace

void PathProgressTracker::setPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw std::invalid_argument("path must not be empty");
  }
  for (const auto & pose : path.poses) {
    if (!pointIsFinite(pose.pose.position)) {
      throw std::invalid_argument("path positions must be finite");
    }
  }

  std::vector<double> cumulative_lengths(path.poses.size(), 0.0);
  std::vector<double> nonzero_segment_lengths;
  nonzero_segment_lengths.reserve(path.poses.size() - 1u);
  for (std::size_t index = 0; index + 1u < path.poses.size(); ++index) {
    const double segment_length = planarDistance(
      path.poses[index].pose.position, path.poses[index + 1u].pose.position);
    cumulative_lengths[index + 1u] = cumulative_lengths[index] + segment_length;
    if (segment_length > rmp::common::math::kMathEpsilon) {
      nonzero_segment_lengths.push_back(segment_length);
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

  path_ = path.poses;
  cumulative_lengths_ = std::move(cumulative_lengths);
  frame_id_ = path.header.frame_id;
  if (frame_id_.empty()) {
    for (const auto & pose : path_) {
      if (!pose.header.frame_id.empty()) {
        frame_id_ = pose.header.frame_id;
        break;
      }
    }
  }
  total_path_length_ = cumulative_lengths_.back();
  progress_arc_length_ = 0.0;
  maximum_progress_budget_ = 2.0 * representative_spacing;
  progress_budget_ = maximum_progress_budget_;
  previous_robot_position_valid_ = false;
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

  if (previous_robot_position_valid_) {
    progress_budget_ = std::min(
      maximum_progress_budget_,
      progress_budget_ + planarDistance(robot_position, previous_robot_position_));
  }

  PathProgressReference reference;
  reference.frame_id = frame_id_;
  if (total_path_length_ <= rmp::common::math::kMathEpsilon) {
    reference.projection = path_.front().pose.position;
    reference.carrot = path_.front().pose.position;
    reference.signed_lateral_error = planarDistance(robot_position, reference.projection);
    previous_robot_position_ = robot_position;
    previous_robot_position_valid_ = true;
    if (!referenceIsFinite(reference)) {
      throw std::domain_error("path progress tracker produced a non-finite reference");
    }
    return reference;
  }

  const double admissible_end = std::min(
    total_path_length_, progress_arc_length_ + progress_budget_);
  bool projection_found = false;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_arc_length = progress_arc_length_;
  std::size_t best_segment_index = 0u;
  double best_segment_fraction = 0.0;
  geometry_msgs::msg::Point best_projection;

  for (std::size_t index = 0; index + 1u < path_.size(); ++index) {
    const double segment_start = cumulative_lengths_[index];
    const double segment_end = cumulative_lengths_[index + 1u];
    const double segment_length = segment_end - segment_start;
    if (segment_length <= rmp::common::math::kMathEpsilon ||
      segment_end + rmp::common::math::kMathEpsilon < progress_arc_length_ ||
      segment_start > admissible_end + rmp::common::math::kMathEpsilon)
    {
      continue;
    }

    const double allowed_start = std::max(segment_start, progress_arc_length_);
    const double allowed_end = std::min(segment_end, admissible_end);
    if (allowed_start > allowed_end + rmp::common::math::kMathEpsilon) {
      continue;
    }

    const auto & start = path_[index].pose.position;
    const auto & end = path_[index + 1u].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    const double minimum_fraction = std::clamp(
      (allowed_start - segment_start) / segment_length, 0.0, 1.0);
    const double maximum_fraction = std::clamp(
      (allowed_end - segment_start) / segment_length, 0.0, 1.0);
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
    const double candidate_arc_length = segment_start + fraction * segment_length;
    if (!projection_found ||
      distance_squared < best_distance_squared - rmp::common::math::kMathEpsilon ||
      (std::abs(distance_squared - best_distance_squared) <=
      rmp::common::math::kMathEpsilon && candidate_arc_length < best_arc_length))
    {
      projection_found = true;
      best_distance_squared = distance_squared;
      best_arc_length = candidate_arc_length;
      best_segment_index = index;
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
  previous_robot_position_ = robot_position;
  previous_robot_position_valid_ = true;

  reference.projection = best_projection;
  reference.segment_index = best_segment_index;
  reference.segment_fraction = best_segment_fraction;
  reference.progress_arc_length = progress_arc_length_;
  reference.carrot_arc_length = std::min(
    progress_arc_length_ + lookahead_distance, total_path_length_);
  const auto carrot = interpolateAtArcLength(
    path_, cumulative_lengths_, total_path_length_, reference.carrot_arc_length);
  reference.carrot = carrot.point;

  const auto & projection_start = path_[best_segment_index].pose.position;
  const auto & projection_end = path_[best_segment_index + 1u].pose.position;
  const double projection_dx = projection_end.x - projection_start.x;
  const double projection_dy = projection_end.y - projection_start.y;
  const double relative_x = robot_position.x - reference.projection.x;
  const double relative_y = robot_position.y - reference.projection.y;
  const double cross = projection_dx * relative_y - projection_dy * relative_x;
  const double lateral_distance = std::sqrt(best_distance_squared);
  reference.signed_lateral_error = cross < 0.0 ? -lateral_distance : lateral_distance;

  const auto & carrot_start = path_[carrot.segment_index].pose.position;
  const auto & carrot_end = path_[carrot.segment_index + 1u].pose.position;
  reference.tangent = std::atan2(
    carrot_end.y - carrot_start.y, carrot_end.x - carrot_start.x);
  reference.curvature = curvatureAtSegment(path_, carrot.segment_index);

  if (!referenceIsFinite(reference)) {
    throw std::domain_error("path progress tracker produced a non-finite reference");
  }
  return reference;
}

bool PathProgressTracker::hasPath() const
{
  return !path_.empty();
}

}  // namespace pid_tracking_controller
