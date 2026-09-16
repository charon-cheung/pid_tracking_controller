#include "rotation_entry_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace pid_tracking_controller
{

RotationEntryGate::RotationEntryGate(const RotationEntryGateConfig & config)
: config_(config)
{
  if (!std::isfinite(config.xy_speed_tolerance) || config.xy_speed_tolerance < 0.0 ||
    !std::isfinite(config.omega_tolerance) || config.omega_tolerance < 0.0 ||
    !std::isfinite(config.stop_confirm_time) || config.stop_confirm_time <= 0.0 ||
    !std::isfinite(config.braking_timeout) || config.braking_timeout <= config.stop_confirm_time)
  {
    throw std::invalid_argument("invalid rotation entry gate configuration");
  }
}

bool RotationEntryGate::stopped(const HolonomicBodyCommand & velocity) const
{
  if (!std::isfinite(velocity.vx) || !std::isfinite(velocity.vy) ||
    !std::isfinite(velocity.omega))
  {
    throw std::domain_error("rotation entry velocity must be finite");
  }
  return std::hypot(velocity.vx, velocity.vy) <= config_.xy_speed_tolerance &&
         std::abs(velocity.omega) <= config_.omega_tolerance;
}

bool RotationEntryGate::beginCycle(const HolonomicBodyCommand & measured)
{
  const bool measured_stopped = stopped(measured);
  if (rotating_) {
    return true;
  }
  if (ready_ && measured_stopped) {
    rotating_ = true;
    return true;
  }
  if (!measured_stopped) {
    ready_ = false;
    stopped_duration_ = 0.0;
    previous_stopped_ = false;
  }
  return false;
}

void RotationEntryGate::observeBraking(
  const HolonomicBodyCommand & measured, const HolonomicBodyCommand & command, double dt)
{
  const bool measured_stopped = stopped(measured);
  const bool command_stopped = stopped(command);
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::domain_error("rotation entry dt must be finite and nonnegative");
  }
  if (rotating_) {
    throw std::logic_error("cannot observe braking after rotation admission");
  }
  const double interval = observed_ ? dt : 0.0;
  // Bound accumulation and reject expiry before granting permission to turn.
  if (interval >= config_.braking_timeout - braking_duration_) {
    throw std::runtime_error("rotation braking timed out before a confirmed stop");
  }
  braking_duration_ += interval;
  const bool both_stopped = measured_stopped && command_stopped;
  stopped_duration_ = both_stopped && previous_stopped_ && dt > 0.0 ?
    stopped_duration_ + interval : 0.0;
  previous_stopped_ = both_stopped;
  observed_ = true;
  ready_ = stopped_duration_ >= config_.stop_confirm_time;
}

void RotationEntryGate::reset(bool preserve_braking)
{
  // Stop evidence is independent of path/yaw; retain it across replans while braking.
  if (preserve_braking && !rotating_) {
    return;
  }
  stopped_duration_ = 0.0;
  braking_duration_ = 0.0;
  observed_ = false;
  previous_stopped_ = false;
  ready_ = false;
  rotating_ = false;
}

}  // namespace pid_tracking_controller
