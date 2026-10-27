#include "holonomic_acceleration_envelope.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "angles/angles.h"

namespace pid_tracking_controller
{
namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

struct TangentKnot
{
  double s;
  double angle;
};

struct EnvelopePoint
{
  double s;
  double body_angle;
  double speed_squared;
};

struct AccelerationCoefficients
{
  double cx;
  double cy;
  double dx;
  double dy;
};

double maxAbsComponent(double begin, double end, bool sine)
{
  const double offset = sine ? 0.5 * kPi : 0.0;
  const double low = std::min(begin, end);
  const double high = std::max(begin, end);
  if (std::ceil((low - offset) / kPi) <= std::floor((high - offset) / kPi)) {
    return 1.0;
  }
  return sine ? std::max(std::abs(std::sin(begin)), std::abs(std::sin(end))) :
         std::max(std::abs(std::cos(begin)), std::abs(std::cos(end)));
}

void boundComponent(double component, double limit, double & speed)
{
  if (std::abs(component) > kEpsilon) {
    speed = std::min(speed, limit / std::abs(component));
  }
}

}  // namespace

double translationAccelerationSpeedLimit(
  const PreparedPath & path, const HolonomicTrackingReference & reference,
  double robot_yaw, double dt, double external_speed_limit,
  const HolonomicSpeedPlannerConfig & config)
{
  const auto & cursor = reference.current.cursor;
  const std::size_t first = cursor.segment_index;
  std::size_t end = first;
  while (end < path.segments.size() &&
    path.segments[end].type == PathSegmentType::kTranslation)
  {
    ++end;
  }

  // Midpoint interpolation gives a continuous nominal direction field instead
  // of differentiating a tiny remaining distance to a piecewise tangent jump.
  // Keep the preceding translation midpoint, but never cross a rotation boundary.
  const std::size_t knot_begin = first > 0u &&
    path.segments[first - 1u].type == PathSegmentType::kTranslation ? first - 1u : first;
  std::vector<TangentKnot> knots;
  knots.reserve(end - knot_begin);
  double tangent = path.segments[knot_begin].path_tangent;
  for (std::size_t index = knot_begin; index < end; ++index) {
    const auto & segment = path.segments[index];
    if (index > knot_begin) {
      tangent += angles::normalize_angle(
        segment.path_tangent - path.segments[index - 1u].path_tangent);
    }
    knots.push_back({
      0.5 * (segment.translation_s_begin + segment.translation_s_end), tangent});
  }

  std::size_t knot_index = 0u;
  const auto tangentAt = [&knots, &knot_index](double s) {
      while (knot_index + 1u < knots.size() && knots[knot_index + 1u].s < s) {
        ++knot_index;
      }
      if (s <= knots.front().s) {
        return knots.front().angle;
      }
      if (knot_index + 1u == knots.size()) {
        return knots.back().angle;
      }
      const auto & before = knots[knot_index];
      const auto & after = knots[knot_index + 1u];
      const double fraction = (s - before.s) / (after.s - before.s);
      return before.angle + fraction * (after.angle - before.angle);
    };

  std::vector<EnvelopePoint> points;
  points.reserve(2u * (end - first) + 1u);
  double predicted_yaw = robot_yaw;
  double last_target_yaw = reference.current.target_yaw;
  const auto appendPoint = [&](double s, std::size_t index, bool boundary) {
      const auto & segment = path.segments[index];
      const double fraction = std::clamp(
        (s - segment.translation_s_begin) / segment.translation_length, 0.0, 1.0);
      const double target_yaw = segment.target_yaw_begin + fraction * segment.target_yaw_delta;
      predicted_yaw += angles::normalize_angle(target_yaw - last_target_yaw);
      last_target_yaw = target_yaw;
      const double body_angle = tangentAt(s) - predicted_yaw;
      const double actual_angle = segment.path_tangent - predicted_yaw;
      double speed = std::min({
          config.nominal_tracking_speed, config.max_xy_speed, external_speed_limit});
      boundComponent(std::cos(body_angle), config.max_vx, speed);
      boundComponent(std::sin(body_angle), config.max_vy, speed);
      boundComponent(std::cos(actual_angle), config.max_vx, speed);
      boundComponent(std::sin(actual_angle), config.max_vy, speed);
      boundComponent(segment.target_yaw_rate_per_meter,
        config.max_omega - config.yaw_feedback_reserve, speed);

      if (boundary && index + 1u < end) {
        const auto & next = path.segments[index + 1u];
        const double next_angle = next.path_tangent - predicted_yaw;
        boundComponent(std::cos(next_angle), config.max_vx, speed);
        boundComponent(std::sin(next_angle), config.max_vy, speed);
        boundComponent(next.target_yaw_rate_per_meter,
          config.max_omega - config.yaw_feedback_reserve, speed);
        // The control law uses the unsmoothed segment tangent. Even if the
        // nominal field is smooth, a sparse corner must be reachable in a cycle.
        boundComponent(std::cos(next_angle) - std::cos(actual_angle),
          config.max_ax * dt, speed);
        boundComponent(std::sin(next_angle) - std::sin(actual_angle),
          config.max_ay * dt, speed);
      }
      points.push_back({s, body_angle, speed * speed});
    };

  appendPoint(cursor.translation_arc_length, first,
    cursor.translation_arc_length >= path.segments[first].translation_s_end - kEpsilon);
  for (std::size_t index = first; index < end; ++index) {
    const auto & segment = path.segments[index];
    const double middle = 0.5 * (segment.translation_s_begin + segment.translation_s_end);
    if (middle > points.back().s + kEpsilon) {
      appendPoint(middle, index, false);
    }
    if (segment.translation_s_end > points.back().s + kEpsilon) {
      appendPoint(segment.translation_s_end, index, true);
    }
  }

  std::vector<AccelerationCoefficients> coefficients;
  coefficients.reserve(points.size() - 1u);
  for (std::size_t index = 0; index + 1u < points.size(); ++index) {
    auto & before = points[index];
    auto & after = points[index + 1u];
    const double ds = after.s - before.s;
    const double cx = maxAbsComponent(before.body_angle, after.body_angle, false);
    const double cy = maxAbsComponent(before.body_angle, after.body_angle, true);
    // Each interval has linear nominal tangent and target yaw. Do not normalize
    // their difference again: opposing path/yaw rotations can exceed pi together.
    const double rate = std::abs(after.body_angle - before.body_angle) / ds;
    const AccelerationCoefficients limits{cx, cy, cy * rate, cx * rate};
    coefficients.push_back(limits);
    double squared_limit = std::numeric_limits<double>::infinity();
    if (limits.dx > kEpsilon) {
      squared_limit = std::min(squared_limit, config.max_ax / limits.dx);
    }
    if (limits.dy > kEpsilon) {
      squared_limit = std::min(squared_limit, config.max_ay / limits.dy);
    }
    before.speed_squared = std::min(before.speed_squared, squared_limit);
    after.speed_squared = std::min(after.speed_squared, squared_limit);
  }

  for (std::size_t index = points.size() - 1u; index > 0u; --index) {
    auto & before = points[index - 1u];
    const auto & after = points[index];
    const auto & limits = coefficients[index - 1u];
    const double twice_ds = 2.0 * (after.s - before.s);
    double reachable = after.speed_squared + config.a_ref_down * twice_ds;
    const auto constrain = [&](double c, double d, double acceleration) {
        const double denominator = c + twice_ds * d;
        if (denominator > kEpsilon) {
          // C*|a_ref| + D*v^2 <= axis limit, with
          // |a_ref| = (v_before^2 - v_after^2)/(2*ds).
          reachable = std::min(reachable,
            (twice_ds * acceleration + c * after.speed_squared) / denominator);
        }
      };
    constrain(limits.cx, limits.dx, config.max_ax);
    constrain(limits.cy, limits.dy, config.max_ay);
    before.speed_squared = std::min(before.speed_squared, reachable);
  }
  return std::sqrt(std::max(0.0, points.front().speed_squared));
}

}  // namespace pid_tracking_controller
