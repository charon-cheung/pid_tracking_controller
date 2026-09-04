#include "rotation_completion_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace pid_tracking_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool configIsValid(const RotationCompletionTrackerConfig & config)
{
  return std::isfinite(config.rotation_yaw_tolerance) &&
         config.rotation_yaw_tolerance >= 0.0 &&
         std::isfinite(config.rotation_omega_tolerance) &&
         config.rotation_omega_tolerance >= 0.0 &&
         std::isfinite(config.rotation_xy_speed_tolerance) &&
         config.rotation_xy_speed_tolerance >= 0.0 &&
         std::isfinite(config.rotation_settle_time) &&
         config.rotation_settle_time >= 0.0;
}

}  // namespace

RotationCompletionTracker::RotationCompletionTracker(
  const RotationCompletionTrackerConfig & config)
: config_(config)
{
  if (!configIsValid(config_)) {
    throw std::invalid_argument("invalid rotation completion tracker configuration");
  }
}

RotationCompletionStatus RotationCompletionTracker::update(
  double normalized_yaw_error,
  double measured_vx,
  double measured_vy,
  double measured_omega,
  double dt)
{
  if (!std::isfinite(normalized_yaw_error) ||
    std::abs(normalized_yaw_error) > kPi ||
    !std::isfinite(measured_vx) || !std::isfinite(measured_vy) ||
    !std::isfinite(measured_omega) || !std::isfinite(dt) || dt < 0.0)
  {
    throw std::domain_error("rotation completion inputs are invalid");
  }

  RotationCompletionStatus status;
  status.yaw_within_tolerance =
    std::abs(normalized_yaw_error) <= config_.rotation_yaw_tolerance;
  status.omega_within_tolerance =
    std::abs(measured_omega) <= config_.rotation_omega_tolerance;
  status.xy_speed_within_tolerance =
    std::hypot(measured_vx, measured_vy) <= config_.rotation_xy_speed_tolerance;
  status.all_conditions = status.yaw_within_tolerance &&
    status.omega_within_tolerance && status.xy_speed_within_tolerance;

  double next_settled_duration = 0.0;
  if (status.all_conditions) {
    const double maximum = std::numeric_limits<double>::max();
    next_settled_duration = settled_duration_ >= maximum - dt ?
      maximum : settled_duration_ + dt;
  }

  settled_duration_ = next_settled_duration;
  status.settled_duration = settled_duration_;
  status.acknowledge_candidate = status.all_conditions &&
    status.settled_duration >= config_.rotation_settle_time;
  return status;
}

void RotationCompletionTracker::reset()
{
  settled_duration_ = 0.0;
}

}  // namespace pid_tracking_controller
