#include "holonomic_speed_planner.hpp"

#include "holonomic_acceleration_envelope.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "rclcpp/rclcpp.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kGeometryEpsilon = 1e-9;
constexpr double kCursorTolerance = 1e-8;
constexpr double kAxisEpsilon = 1e-12;
constexpr double kRateEpsilon = 1e-12;
constexpr double kRateWindowSpacingMultiplier = 4.0;

struct RateSample
{
  double target_yaw_rate{0.0};
  double path_tangent_rate{0.0};
};

struct SpeedCandidate
{
  double arc_length{0.0};
  std::size_t segment_index{0u};
};

struct SpeedInterval
{
  long double lower{0.0L};
  long double upper{std::numeric_limits<long double>::infinity()};
};

void constrainAffineAxis(
  SpeedInterval & interval, long double tangent, long double bias,
  long double lower, long double upper)
{
  if (tangent == 0.0L) {
    if (bias < lower || bias > upper) {
      interval.upper = -1.0L;
    }
    return;
  }
  const long double first = (lower - bias) / tangent;
  const long double second = (upper - bias) / tangent;
  interval.lower = std::max(interval.lower, std::min(first, second));
  interval.upper = std::min(interval.upper, std::max(first, second));
}

SpeedInterval feedbackSpeedInterval(
  long double tx, long double ty, const HolonomicTranslationFeedback & feedback,
  const HolonomicSpeedPlannerConfig & config, long double combined_limit)
{
  SpeedInterval interval;
  constrainAffineAxis(interval, tx, feedback.vx, -config.max_vx, config.max_vx);
  constrainAffineAxis(interval, ty, feedback.vy, -config.max_vy, config.max_vy);

  // Intersect |v*t + feedback| <= combined_limit with the signed axis intervals.
  // A positive lower bound matters when FF can cancel opposing feedback.
  const long double tangent_norm = std::hypot(tx, ty);
  const long double along = (tx * feedback.vx + ty * feedback.vy) / tangent_norm;
  const long double normal = std::abs(tx * feedback.vy - ty * feedback.vx) / tangent_norm;
  if (normal > combined_limit) {
    interval.upper = -1.0L;
    return interval;
  }
  const long double center = -along / tangent_norm;
  const long double radius = std::sqrt(
    (combined_limit - normal) * (combined_limit + normal)) / tangent_norm;
  interval.lower = std::max(interval.lower, center - radius);
  interval.upper = std::min(interval.upper, center + radius);
  return interval;
}

bool configIsValid(const HolonomicSpeedPlannerConfig & config)
{
  const double normal_acceleration = std::min(config.max_ax, config.max_ay);
  return std::isfinite(config.nominal_tracking_speed) &&
         config.nominal_tracking_speed >= 0.0 &&
         std::isfinite(config.max_vx) && config.max_vx > 0.0 &&
         std::isfinite(config.max_vy) && config.max_vy > 0.0 &&
         std::isfinite(config.max_xy_speed) && config.max_xy_speed > 0.0 &&
         std::isfinite(config.max_omega) && config.max_omega > 0.0 &&
         std::isfinite(config.max_ax) && config.max_ax > 0.0 &&
         std::isfinite(config.max_ay) && config.max_ay > 0.0 &&
         std::isfinite(config.a_ref_up) && config.a_ref_up > 0.0 &&
         config.a_ref_up <= normal_acceleration &&
         std::isfinite(config.a_ref_down) && config.a_ref_down > 0.0 &&
         config.a_ref_down <= normal_acceleration &&
         std::isfinite(config.stopping_deceleration) && config.stopping_deceleration > 0.0 &&
         config.stopping_deceleration <= normal_acceleration &&
         std::isfinite(config.stopping_delay) && config.stopping_delay >= 0.0 &&
         std::isfinite(config.stopping_min_speed) && config.stopping_min_speed >= 0.0 &&
         std::isfinite(config.yaw_feedback_reserve) &&
         config.yaw_feedback_reserve >= 0.0 &&
         config.yaw_feedback_reserve < config.max_omega &&
         std::isfinite(config.configured_min_rate_window) &&
         config.configured_min_rate_window >= 0.0 &&
         std::isfinite(config.max_translation_curve_accel) &&
         config.max_translation_curve_accel > 0.0;
}

bool pointIsFinite(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

double representativeSpacing(const PreparedPath & path)
{
  std::vector<double> lengths;
  lengths.reserve(path.segments.size());
  for (const auto & segment : path.segments) {
    if (segment.type == PathSegmentType::kTranslation &&
      segment.translation_length > kGeometryEpsilon)
    {
      lengths.push_back(segment.translation_length);
    }
  }
  if (lengths.empty()) {
    return 0.0;
  }

  std::sort(lengths.begin(), lengths.end());
  const std::size_t middle = lengths.size() / 2u;
  if (lengths.size() % 2u == 0u) {
    return 0.5 * (lengths[middle - 1u] + lengths[middle]);
  }
  return lengths[middle];
}

std::size_t firstRotationAfter(
  const PreparedPath & path,
  std::size_t current_segment_index)
{
  for (std::size_t index = current_segment_index + 1u;
    index < path.segments.size(); ++index)
  {
    if (path.segments[index].type == PathSegmentType::kRotation) {
      return index;
    }
  }
  return path.segments.size();
}

std::size_t translationSegmentAt(
  const PreparedPath & path,
  std::size_t begin_index,
  std::size_t end_index,
  double arc_length)
{
  for (std::size_t index = begin_index; index < end_index; ++index) {
    const auto & segment = path.segments[index];
    if (segment.type == PathSegmentType::kTranslation &&
      arc_length <= segment.translation_s_end + kGeometryEpsilon)
    {
      return index;
    }
  }
  throw std::logic_error("speed planner could not locate translation segment");
}

RateSample spatialRates(
  const PreparedPath & path,
  const SpeedCandidate & candidate,
  std::size_t end_segment_index,
  double horizon_arc_length,
  double rate_window_length)
{
  const double window_end = std::min(
    candidate.arc_length + rate_window_length, horizon_arc_length);
  const double actual_window_length = window_end - candidate.arc_length;
  if (actual_window_length <= kGeometryEpsilon) {
    return {};
  }

  double target_yaw_delta = 0.0;
  double tangent_delta = 0.0;
  double previous_tangent = path.segments[candidate.segment_index].path_tangent;
  for (std::size_t index = candidate.segment_index;
    index < end_segment_index; ++index)
  {
    const auto & segment = path.segments[index];
    if (segment.type != PathSegmentType::kTranslation ||
      segment.translation_s_begin > window_end + kGeometryEpsilon)
    {
      break;
    }

    if (index != candidate.segment_index) {
      tangent_delta += std::abs(angles::normalize_angle(
        segment.path_tangent - previous_tangent));
      previous_tangent = segment.path_tangent;
    }

    const double overlap_begin = std::max(
      candidate.arc_length, segment.translation_s_begin);
    const double overlap_end = std::min(window_end, segment.translation_s_end);
    if (overlap_end > overlap_begin) {
      target_yaw_delta += segment.target_yaw_rate_per_meter *
        (overlap_end - overlap_begin);
    }
  }

  return {
    target_yaw_delta / actual_window_length,
    tangent_delta / actual_window_length};
}

double propagatedSpeed(double local_speed, double deceleration, double distance)
{
  if (std::isinf(local_speed)) {
    return local_speed;
  }
  return std::sqrt(std::max(
      0.0, local_speed * local_speed + 2.0 * deceleration * distance));
}

void addCandidate(
  std::vector<SpeedCandidate> & candidates,
  const PreparedPath & path,
  std::size_t begin_index,
  std::size_t end_index,
  double current_arc_length,
  double horizon_arc_length,
  double arc_length,
  std::optional<std::size_t> segment_index = std::nullopt)
{
  if (arc_length <= current_arc_length + kGeometryEpsilon ||
    arc_length >= horizon_arc_length - kGeometryEpsilon)
  {
    return;
  }
  for (const auto & candidate : candidates) {
    if (std::abs(candidate.arc_length - arc_length) <= kGeometryEpsilon) {
      return;
    }
  }
  candidates.push_back({
      arc_length,
      segment_index.value_or(
        translationSegmentAt(path, begin_index, end_index, arc_length))});
}

void applyLimit(
  double limit,
  HolonomicSpeedLimitReason reason,
  double & target,
  HolonomicSpeedLimitReason & target_reason)
{
  if (limit < target - kGeometryEpsilon) {
    target = limit;
    target_reason = reason;
  }
}

void validateCursor(
  const PreparedPath & path,
  const HolonomicPathSample & current)
{
  const auto & cursor = current.cursor;
  if (cursor.segment_index >= path.segments.size() ||
    path.segments[cursor.segment_index].type != PathSegmentType::kTranslation ||
    !std::isfinite(cursor.segment_fraction) ||
    cursor.segment_fraction < -kCursorTolerance ||
    cursor.segment_fraction > 1.0 + kCursorTolerance ||
    !std::isfinite(cursor.translation_arc_length) ||
    !pointIsFinite(current.position))
  {
    throw std::invalid_argument("speed planner reference cursor is invalid");
  }

  const auto & segment = path.segments[cursor.segment_index];
  const double fraction = std::clamp(cursor.segment_fraction, 0.0, 1.0);
  const double expected_arc_length = segment.translation_s_begin +
    fraction * segment.translation_length;
  if (cursor.translation_arc_length <
    segment.translation_s_begin - kCursorTolerance ||
    cursor.translation_arc_length > segment.translation_s_end + kCursorTolerance ||
    std::abs(cursor.translation_arc_length - expected_arc_length) >
    kCursorTolerance)
  {
    throw std::invalid_argument("speed planner reference cursor is inconsistent with path");
  }

  const auto & start = path.poses[segment.start_pose_index].pose.position;
  const auto & end = path.poses[segment.end_pose_index].pose.position;
  const double expected_x = start.x + fraction * (end.x - start.x);
  const double expected_y = start.y + fraction * (end.y - start.y);
  const double expected_z = start.z + fraction * (end.z - start.z);
  if (std::abs(current.position.x - expected_x) > kCursorTolerance ||
    std::abs(current.position.y - expected_y) > kCursorTolerance ||
    std::abs(current.position.z - expected_z) > kCursorTolerance)
  {
    throw std::invalid_argument("speed planner reference position is inconsistent with path");
  }
}

}  // namespace

double convertExternalSpeedLimitToAbsolute(
  double speed_limit,
  bool percentage,
  double max_xy_speed)
{
  if (!std::isfinite(speed_limit) || speed_limit < 0.0 ||
    !std::isfinite(max_xy_speed) || max_xy_speed <= 0.0)
  {
    throw std::invalid_argument("speed limits must be finite and nonnegative");
  }
  if (percentage && speed_limit > 100.0) {
    throw std::invalid_argument("percentage speed limit must be between 0 and 100");
  }
  if (speed_limit == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return percentage ? max_xy_speed * (speed_limit / 100.0) : speed_limit;
}

HolonomicSpeedPlanner::HolonomicSpeedPlanner(
  const HolonomicSpeedPlannerConfig & config)
: config_(config), tracking_guard_(config.tracking_guard)
{
  if (!configIsValid(config_)) {
    throw std::invalid_argument("holonomic speed planner configuration is invalid");
  }
}

void HolonomicSpeedPlanner::setPath(PreparedPathPtr path)
{
  if (!path || path->poses.empty()) {
    throw std::invalid_argument("speed planner path must not be empty");
  }
  path_ = std::move(path);
  tracking_guard_.reset();
  representative_path_spacing_ = representativeSpacing(*path_);
  rate_window_length_ = std::max(
    config_.configured_min_rate_window,
    kRateWindowSpacingMultiplier * representative_path_spacing_);
}

void HolonomicSpeedPlanner::seed(double reference_velocity)
{
  if (!std::isfinite(reference_velocity) || reference_velocity < 0.0) {
    throw std::domain_error("speed planner seed must be finite and nonnegative");
  }
  previous_reference_velocity_ = reference_velocity;
  tracking_guard_.reset();
}

void HolonomicSpeedPlanner::commitSafetyOutput(double actual_progress_speed)
{
  if (!std::isfinite(actual_progress_speed) || actual_progress_speed < 0 ||
    actual_progress_speed > config_.max_xy_speed)
  {throw std::domain_error("safety progress speed is invalid");}
  previous_reference_velocity_ = actual_progress_speed;
}

void HolonomicSpeedPlanner::logObstacleState(
  std::optional<double> distance, const HolonomicSpeedPlan & plan, double observed_speed)
{
  const unsigned state = !distance ? 0 : plan.obstacle_stop_unreachable ? 3 :
    plan.obstacle_speed_limit == 0 ? 2 : 1;
  if (state == obstacle_log_state_) {return;}
  const char * names[] = {"released", "active", "zero_cap", "unreachable"};
  RCLCPP_INFO(rclcpp::get_logger("pid_tracking_controller.safety"),
    "PID_SAFETY obstacle event=%s distance_m=%.6f cap_mps=%.6f observed_mps=%.6f "
    "reference_mps=%.6f", names[state], distance.value_or(-1.0),
    plan.obstacle_speed_limit, observed_speed, plan.reference_velocity);
  obstacle_log_state_ = state;
}

HolonomicSpeedPlan HolonomicSpeedPlanner::plan(
  const HolonomicTrackingReference & reference,
  double robot_yaw,
  double dt,
  double external_speed_limit,
  double observed_speed,
  HolonomicTranslationFeedback feedback,
  std::optional<HolonomicBodyCommand> previous_command,
  std::optional<HolonomicTrackingObservation> observation,
  std::optional<double> obstacle_stop_distance)
{
  if (!path_) {
    throw std::logic_error("speed planner has no path");
  }
  if (obstacle_stop_distance &&
    (!std::isfinite(*obstacle_stop_distance) || *obstacle_stop_distance < 0))
  {throw std::domain_error("obstacle stop distance must be finite and nonnegative");}
  if (!std::isfinite(robot_yaw) || !std::isfinite(dt) || dt <= 0.0 ||
    std::isnan(external_speed_limit) || external_speed_limit < 0.0 ||
    !std::isfinite(observed_speed) || observed_speed < 0.0 ||
    !std::isfinite(feedback.vx) || !std::isfinite(feedback.vy) ||
    (previous_command && (!std::isfinite(previous_command->vx) ||
    !std::isfinite(previous_command->vy))) ||
    (observation && (!std::isfinite(observation->lateral_error) ||
    !std::isfinite(observation->measured_vx) || !std::isfinite(observation->measured_vy))))
  {
    throw std::domain_error("speed planner inputs are invalid");
  }

  HolonomicSpeedPlan plan;
  SpeedInterval feedback_interval;
  plan.external_speed_limit = external_speed_limit;
  plan.representative_path_spacing = representative_path_spacing_;
  plan.rate_window_length = rate_window_length_;
  if (obstacle_stop_distance) {
    const long double output_speed = previous_command ?
      std::hypot(static_cast<long double>(previous_command->vx),
      static_cast<long double>(previous_command->vy)) : 0.0L;
    const long double motion_speed = std::max<long double>(observed_speed, output_speed);
    const long double delay = std::max<long double>(motion_speed, previous_reference_velocity_) *
      config_.stopping_delay;
    const long double remaining = std::max(0.0L, *obstacle_stop_distance - delay);
    const long double braking_cap = std::sqrt(2.0L * config_.stopping_deceleration * remaining);
    const long double cap = std::min<long double>(config_.max_xy_speed, braking_cap);
    plan.obstacle_speed_limit = static_cast<double>(cap);
    if (plan.obstacle_speed_limit > cap) {
      plan.obstacle_speed_limit = std::nextafter(plan.obstacle_speed_limit, 0.0);
    }
    plan.obstacle_stop_unreachable = motion_speed > braking_cap;
  }

  if (reference.mode == HolonomicReferenceMode::kRotationPending ||
    reference.mode == HolonomicReferenceMode::kTerminal)
  {
    plan.speed_limit_reason =
      reference.mode == HolonomicReferenceMode::kRotationPending ?
      HolonomicSpeedLimitReason::kRotation : HolonomicSpeedLimitReason::kTerminal;
    if (reference.mode == HolonomicReferenceMode::kRotationPending) {
      plan.stopping_speed_limit = 0.0;
    }
    plan.stopping_speed_limit = std::min(plan.stopping_speed_limit, plan.obstacle_speed_limit);
    previous_reference_velocity_ = 0.0;
    tracking_guard_.reset();
    logObstacleState(obstacle_stop_distance, plan, observed_speed);
    return plan;
  } else {
    const auto & cursor = reference.current.cursor;
    validateCursor(*path_, reference.current);

    const auto & current_segment = path_->segments[cursor.segment_index];
    const double delta = angles::normalize_angle(
      current_segment.path_tangent - robot_yaw);
    const double abs_cosine = std::abs(std::cos(delta));
    const double abs_sine = std::abs(std::sin(delta));
    const double vx_limit = abs_cosine <= kAxisEpsilon ?
      std::numeric_limits<double>::infinity() : config_.max_vx / abs_cosine;
    const double vy_limit = abs_sine <= kAxisEpsilon ?
      std::numeric_limits<double>::infinity() : config_.max_vy / abs_sine;
    const double body_axis_limit = std::min(vx_limit, vy_limit);
    plan.axis_speed_limit = body_axis_limit;

    const std::size_t rotation_index = firstRotationAfter(
      *path_, cursor.segment_index);
    const bool rotation_ahead = rotation_index < path_->segments.size();
    const double current_arc_length = cursor.translation_arc_length;
    const double horizon_arc_length = rotation_ahead ?
      path_->segments[rotation_index].translation_s_begin :
      path_->total_translation_length;

    plan.local_path_tangent_rate_per_meter = spatialRates(
      *path_, {current_arc_length, cursor.segment_index}, rotation_index,
      horizon_arc_length, rate_window_length_).path_tangent_rate;

    const double omega_available = config_.max_omega - config_.yaw_feedback_reserve;
    const auto yawFeedforwardSpeed = [omega_available](double rate) {
        return std::abs(rate) <= kRateEpsilon ?
          std::numeric_limits<double>::infinity() : omega_available / std::abs(rate);
      };
    plan.yaw_ff_speed_limit = yawFeedforwardSpeed(current_segment.target_yaw_rate_per_meter);

    std::vector<SpeedCandidate> candidates;
    candidates.push_back({current_arc_length, cursor.segment_index});
    for (std::size_t index = cursor.segment_index;
      index < rotation_index && index < path_->segments.size(); ++index)
    {
      const auto & segment = path_->segments[index];
      if (segment.type != PathSegmentType::kTranslation) {
        break;
      }
      // Window averages can hide short yaw changes or cancel opposing rates.
      // Also constrain the actual segment rate used by the control-law FF.
      plan.yaw_speed_limit = std::min(
        plan.yaw_speed_limit,
        propagatedSpeed(
          yawFeedforwardSpeed(segment.target_yaw_rate_per_meter), config_.a_ref_down,
          std::max(0.0, segment.translation_s_begin - current_arc_length)));
      addCandidate(
        candidates, *path_, cursor.segment_index, rotation_index,
        current_arc_length, horizon_arc_length, segment.translation_s_begin,
        index);
      addCandidate(
        candidates, *path_, cursor.segment_index, rotation_index,
        current_arc_length, horizon_arc_length,
        segment.translation_s_begin - rate_window_length_);
      addCandidate(
        candidates, *path_, cursor.segment_index, rotation_index,
        current_arc_length, horizon_arc_length,
        segment.translation_s_end - rate_window_length_);
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const auto & first, const auto & second) {
        return first.arc_length < second.arc_length;
      });
    if (horizon_arc_length > current_arc_length + kGeometryEpsilon) {
      candidates.push_back({
          horizon_arc_length,
          translationSegmentAt(
            *path_, cursor.segment_index, rotation_index,
            horizon_arc_length)});
    }

    const double normal_acceleration =
      std::min({config_.max_translation_curve_accel, config_.max_ax, config_.max_ay});
    // Between event points each spatial rate is monotone. Applying the larger
    // endpoint magnitude from the earlier distance is conservative over the
    // complete interval, including an interior propagated-speed minimum.
    for (std::size_t index = 0; index + 1u < candidates.size(); ++index) {
      const auto & interval_begin = candidates[index];
      const auto & interval_end = candidates[index + 1u];
      const auto begin_rates = spatialRates(
        *path_, interval_begin, rotation_index, horizon_arc_length,
        rate_window_length_);
      const SpeedCandidate left_end{
        interval_end.arc_length,
        translationSegmentAt(
          *path_, cursor.segment_index, rotation_index,
          interval_end.arc_length)};
      const auto end_rates = spatialRates(
        *path_, left_end, rotation_index, horizon_arc_length,
        rate_window_length_);
      const double abs_yaw_rate = std::max(
        std::abs(begin_rates.target_yaw_rate),
        std::abs(end_rates.target_yaw_rate));
      const double abs_tangent_rate = std::max(
        std::abs(begin_rates.path_tangent_rate),
        std::abs(end_rates.path_tangent_rate));
      plan.max_abs_target_yaw_rate_per_meter = std::max(
        plan.max_abs_target_yaw_rate_per_meter, abs_yaw_rate);
      plan.max_abs_path_tangent_rate_per_meter = std::max(
        plan.max_abs_path_tangent_rate_per_meter, abs_tangent_rate);

      const double yaw_local_limit = abs_yaw_rate <= kRateEpsilon ?
        std::numeric_limits<double>::infinity() : omega_available / abs_yaw_rate;
      const double tangent_local_limit = abs_tangent_rate <= kRateEpsilon ?
        std::numeric_limits<double>::infinity() :
        std::sqrt(normal_acceleration / abs_tangent_rate);
      const double distance = interval_begin.arc_length - current_arc_length;
      plan.yaw_speed_limit = std::min(
        plan.yaw_speed_limit,
        propagatedSpeed(yaw_local_limit, config_.a_ref_down, distance));
      plan.tangent_speed_limit = std::min(
        plan.tangent_speed_limit,
        propagatedSpeed(tangent_local_limit, config_.a_ref_down, distance));
    }

    plan.acceleration_speed_limit = translationAccelerationSpeedLimit(
      *path_, reference, robot_yaw, dt, external_speed_limit, config_);

    plan.stopping_distance = std::max(
      0.0, horizon_arc_length - current_arc_length);
    // Reserve distance consumed before a reduced command can affect the robot.
    // No fixed spatial offset: as speed falls the full remaining distance becomes
    // available again, so this cannot strand the tracker before a rotation boundary.
    plan.stopping_delay_distance =
      std::max(observed_speed, previous_reference_velocity_) * config_.stopping_delay;
    const double stopping_distance = std::max(
      0.0, plan.stopping_distance - plan.stopping_delay_distance);
    plan.stopping_speed_limit = std::sqrt(
      2.0 * config_.stopping_deceleration * stopping_distance);
    if (plan.stopping_distance > 0.0) {
      plan.stopping_speed_limit = std::max(
        plan.stopping_speed_limit, config_.stopping_min_speed);
    }

    plan.stopping_speed_limit = std::min(plan.stopping_speed_limit, plan.obstacle_speed_limit);

    plan.reference_velocity_target = config_.nominal_tracking_speed;
    plan.speed_limit_reason = HolonomicSpeedLimitReason::kNominal;
    applyLimit(
      external_speed_limit, HolonomicSpeedLimitReason::kExternal,
      plan.reference_velocity_target, plan.speed_limit_reason);
    applyLimit(
      body_axis_limit, HolonomicSpeedLimitReason::kBodyAxis,
      plan.reference_velocity_target, plan.speed_limit_reason);
    applyLimit(
      config_.max_xy_speed, HolonomicSpeedLimitReason::kCombinedSpeed,
      plan.reference_velocity_target, plan.speed_limit_reason);
    applyLimit(
      plan.yaw_speed_limit, HolonomicSpeedLimitReason::kYawRate,
      plan.reference_velocity_target, plan.speed_limit_reason);
    applyLimit(
      plan.tangent_speed_limit, HolonomicSpeedLimitReason::kTangentRate,
      plan.reference_velocity_target, plan.speed_limit_reason);
    applyLimit(
      plan.acceleration_speed_limit, HolonomicSpeedLimitReason::kAccelerationPreview,
      plan.reference_velocity_target, plan.speed_limit_reason);
    applyLimit(
      plan.stopping_speed_limit, obstacle_stop_distance &&
      plan.obstacle_speed_limit <= plan.stopping_speed_limit ?
      HolonomicSpeedLimitReason::kObstacleStop : HolonomicSpeedLimitReason::kStoppingDistance,
      plan.reference_velocity_target, plan.speed_limit_reason);

    if (feedback.vx != 0.0 || feedback.vy != 0.0) {
      feedback_interval = feedbackSpeedInterval(
        std::cos(delta), std::sin(delta), feedback, config_,
        std::min({config_.max_xy_speed, external_speed_limit, plan.stopping_speed_limit}));
      if (feedback_interval.lower >
        std::min(feedback_interval.upper, static_cast<long double>(plan.reference_velocity_target)))
      {
        // Keep feedback for the command limiter; only remove infeasible feedforward.
        plan.feedback_speed_limit = 0.0;
        plan.feedback_unreachable = true;
      } else {
        plan.feedback_speed_limit = static_cast<double>(feedback_interval.upper);
        if (plan.feedback_speed_limit > feedback_interval.upper) {
          plan.feedback_speed_limit = std::nextafter(plan.feedback_speed_limit, 0.0);
        }
      }
      applyLimit(
        plan.feedback_speed_limit, HolonomicSpeedLimitReason::kFeedbackBudget,
        plan.reference_velocity_target, plan.speed_limit_reason);
    }
  }

  if (observation) {
    const double tangent = reference.current.path_tangent - robot_yaw;
    const double normal_speed = -std::sin(tangent) * observation->measured_vx +
      std::cos(tangent) * observation->measured_vy;
    plan.tracking_acceleration_held = tracking_guard_.update(
      observation->lateral_error, normal_speed, dt);
    if (plan.tracking_acceleration_held) {
      plan.tracking_speed_limit = previous_reference_velocity_;
      applyLimit(plan.tracking_speed_limit, HolonomicSpeedLimitReason::kTrackingResponse,
        plan.reference_velocity_target, plan.speed_limit_reason);
    }
  } else {
    tracking_guard_.reset();
  }

  const double lower = std::max(
    0.0, previous_reference_velocity_ - config_.a_ref_down * dt);
  const double upper = std::min(
    previous_reference_velocity_ + config_.a_ref_up * dt, plan.tracking_speed_limit);
  plan.reference_velocity = std::clamp(
    plan.reference_velocity_target, lower, upper);
  plan.reference_velocity = std::min(
    plan.reference_velocity,
    std::min({external_speed_limit, plan.axis_speed_limit, config_.max_xy_speed,
      plan.yaw_ff_speed_limit, plan.stopping_speed_limit, plan.feedback_speed_limit}));
  // Only ordinary translation uses this rectangle. A previous command outside
  // the stopping cap belongs to the limiter's separate braking/deceleration policy.
  if (previous_command &&
    std::hypot(previous_command->vx, previous_command->vy) <= plan.stopping_speed_limit)
  {
    const double tangent = reference.current.path_tangent - robot_yaw;
    const long double tx = std::cos(tangent);
    const long double ty = std::sin(tangent);
    SpeedInterval reachable;
    reachable.lower = std::max(static_cast<long double>(lower), feedback_interval.lower);
    reachable.upper = std::min({static_cast<long double>(upper), feedback_interval.upper,
      static_cast<long double>(external_speed_limit),
      static_cast<long double>(plan.axis_speed_limit),
      static_cast<long double>(config_.max_xy_speed),
      static_cast<long double>(plan.yaw_ff_speed_limit),
      static_cast<long double>(plan.stopping_speed_limit),
      static_cast<long double>(plan.feedback_speed_limit)});
    const long double dx = static_cast<long double>(config_.max_ax) * dt;
    const long double dy = static_cast<long double>(config_.max_ay) * dt;
    constrainAffineAxis(reachable, tx, feedback.vx,
      static_cast<long double>(previous_command->vx) - dx,
      static_cast<long double>(previous_command->vx) + dx);
    constrainAffineAxis(reachable, ty, feedback.vy,
      static_cast<long double>(previous_command->vy) - dy,
      static_cast<long double>(previous_command->vy) + dy);
    if (reachable.lower <= reachable.upper) {
      const long double candidate = std::clamp(
        static_cast<long double>(plan.reference_velocity), reachable.lower, reachable.upper);
      double selected = static_cast<double>(candidate);
      if (selected < reachable.lower) {
        selected = std::nextafter(selected, std::numeric_limits<double>::infinity());
      } else if (selected > reachable.upper) {
        selected = std::nextafter(selected, -std::numeric_limits<double>::infinity());
      }
      if (selected >= reachable.lower && selected <= reachable.upper) {
        if (selected != plan.reference_velocity) {
          plan.speed_limit_reason = HolonomicSpeedLimitReason::kAccelerationBudget;
        }
        plan.reference_velocity = selected;
      } else {
        plan.acceleration_unreachable = true;
      }
    } else {
      // No scalar FF can satisfy every axis this cycle. Keep the planned
      // deceleration and let the unchanged final limiter handle the transition.
      plan.acceleration_unreachable = true;
    }
  }
  // Do not jump upward to a positive feasibility bound; preserve reference acceleration.
  plan.feedback_unreachable = plan.feedback_unreachable ||
    static_cast<long double>(plan.reference_velocity) < feedback_interval.lower;
  previous_reference_velocity_ = plan.reference_velocity;
  logObstacleState(obstacle_stop_distance, plan, observed_speed);
  return plan;
}

}  // namespace pid_tracking_controller
