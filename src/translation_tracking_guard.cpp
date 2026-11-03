#include "translation_tracking_guard.hpp"

#include <cmath>
#include <stdexcept>

namespace pid_tracking_controller
{
namespace
{
constexpr double kMinErrorGrowthSpeed = 0.002;  // m/s; reject numerical/noise-sized growth.
}  // namespace

TranslationTrackingGuard::TranslationTrackingGuard(const TranslationTrackingGuardConfig & config)
: config_(config)
{
  if (!std::isfinite(config.enter_error) || !std::isfinite(config.exit_error) ||
    config.exit_error <= 0.0 || config.enter_error <= config.exit_error ||
    !std::isfinite(config.min_outward_speed) || config.min_outward_speed <= 0.0 ||
    !std::isfinite(config.confirm_time) || config.confirm_time <= 0.0 ||
    !std::isfinite(config.release_confirm_time) || config.release_confirm_time <= 0.0)
  {
    throw std::invalid_argument("invalid translation tracking guard configuration");
  }
}

void TranslationTrackingGuard::reset()
{
  previous_error_.reset();
  confirmation_duration_ = 0.0;
  previous_qualifying_ = false;
  holding_ = false;
}

bool TranslationTrackingGuard::update(
  double lateral_error, double measured_normal_speed, double dt)
{
  if (!std::isfinite(lateral_error) || !std::isfinite(measured_normal_speed) ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    throw std::domain_error("invalid translation tracking observation");
  }
  const double error = std::abs(lateral_error);
  const double outward_speed = lateral_error >=
    0.0 ? measured_normal_speed : -measured_normal_speed;
  const double error_growth = previous_error_ && lateral_error * *previous_error_ > 0.0 ?
    error - std::abs(*previous_error_) : 0.0;
  const bool growing = error_growth > kMinErrorGrowthSpeed * dt;
  // Position error can keep growing while odometry reports little or inward motion.
  // Accept sustained same-side position growth as an alternative drift observation;
  // the common confirmation timer still rejects an isolated pose jump.
  const bool qualifying = holding_ ? error <= config_.exit_error :
    (error >= config_.enter_error && growing &&
    (outward_speed >= config_.min_outward_speed ||
    error_growth >= config_.min_outward_speed * dt));
  // Only intervals bounded by two qualifying samples count towards confirmation.
  confirmation_duration_ = qualifying && previous_qualifying_ ? confirmation_duration_ + dt : 0.0;
  const double required_duration = holding_ ? config_.release_confirm_time : config_.confirm_time;
  if (confirmation_duration_ >= required_duration) {
    holding_ = !holding_;
    confirmation_duration_ = 0.0;
    previous_qualifying_ = false;
  } else {
    previous_qualifying_ = qualifying;
  }
  previous_error_ = lateral_error;
  return holding_;
}

}  // namespace pid_tracking_controller
