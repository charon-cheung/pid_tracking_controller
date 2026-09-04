#include "holonomic_command_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pid_tracking_controller
{
namespace
{

struct BaseProjection
{
  HolonomicBodyCommand command;
  bool vx_saturated{false};
  bool vy_saturated{false};
  bool xy_saturated{false};
  bool omega_saturated{false};
};

bool configIsValid(const HolonomicCommandLimiterConfig & config)
{
  return std::isfinite(config.max_vx) && config.max_vx > 0.0 &&
         std::isfinite(config.max_vy) && config.max_vy > 0.0 &&
         std::isfinite(config.max_xy_speed) && config.max_xy_speed > 0.0 &&
         std::isfinite(config.max_omega) && config.max_omega > 0.0 &&
         std::isfinite(config.max_ax) && config.max_ax > 0.0 &&
         std::isfinite(config.max_ay) && config.max_ay > 0.0 &&
         std::isfinite(config.max_alpha) && config.max_alpha > 0.0 &&
         std::isfinite(config.control_gap_threshold) &&
         config.control_gap_threshold > 0.0;
}

bool commandIsFinite(const HolonomicBodyCommand & command)
{
  return std::isfinite(command.vx) && std::isfinite(command.vy) &&
         std::isfinite(command.omega);
}

bool externalSpeedLimitIsValid(double external_speed_limit)
{
  return (std::isfinite(external_speed_limit) && external_speed_limit >= 0.0) ||
         (std::isinf(external_speed_limit) && external_speed_limit > 0.0);
}

long double translationSpeed(const HolonomicBodyCommand & command)
{
  return std::hypot(
    static_cast<long double>(command.vx),
    static_cast<long double>(command.vy));
}

bool projectTranslationToSpeed(
  HolonomicBodyCommand & command,
  double speed_limit)
{
  const long double wide_speed_limit = static_cast<long double>(speed_limit);
  const long double xy_speed = translationSpeed(command);
  if (xy_speed <= wide_speed_limit) {
    return false;
  }

  const long double scale = wide_speed_limit / xy_speed;
  command.vx = static_cast<double>(static_cast<long double>(command.vx) * scale);
  command.vy = static_cast<double>(static_cast<long double>(command.vy) * scale);
  while (translationSpeed(command) > wide_speed_limit) {
    if (std::abs(command.vx) >= std::abs(command.vy)) {
      command.vx = std::nextafter(command.vx, 0.0);
    } else {
      command.vy = std::nextafter(command.vy, 0.0);
    }
  }
  return true;
}

BaseProjection projectBaseLimits(
  const HolonomicBodyCommand & input,
  const HolonomicCommandLimiterConfig & config)
{
  BaseProjection projection;
  projection.vx_saturated = input.vx<-config.max_vx || input.vx> config.max_vx;
  projection.vy_saturated = input.vy<-config.max_vy || input.vy> config.max_vy;
  projection.omega_saturated =
    input.omega<-config.max_omega || input.omega> config.max_omega;
  projection.command.vx = std::clamp(input.vx, -config.max_vx, config.max_vx);
  projection.command.vy = std::clamp(input.vy, -config.max_vy, config.max_vy);
  projection.command.omega =
    std::clamp(input.omega, -config.max_omega, config.max_omega);

  projection.xy_saturated =
    projectTranslationToSpeed(projection.command, config.max_xy_speed);
  return projection;
}

bool applyExternalSpeedLimit(
  HolonomicBodyCommand & command,
  double external_speed_limit)
{
  return projectTranslationToSpeed(command, external_speed_limit);
}

bool translationCandidateIsSafe(
  const HolonomicBodyCommand & candidate,
  const HolonomicBodyCommand & previous,
  const HolonomicCommandLimiterConfig & config,
  long double max_delta_vx,
  long double max_delta_vy)
{
  return std::isfinite(candidate.vx) && std::isfinite(candidate.vy) &&
         std::abs(static_cast<long double>(candidate.vx) - previous.vx) <=
         max_delta_vx &&
         std::abs(static_cast<long double>(candidate.vy) - previous.vy) <=
         max_delta_vy &&
         std::abs(static_cast<long double>(candidate.vx)) <= config.max_vx &&
         std::abs(static_cast<long double>(candidate.vy)) <= config.max_vy &&
         translationSpeed(candidate) <= config.max_xy_speed;
}

double quantizeTowardPrevious(long double ideal, double previous)
{
  double candidate = static_cast<double>(ideal);
  if ((ideal > previous && static_cast<long double>(candidate) > ideal) ||
    (ideal < previous && static_cast<long double>(candidate) < ideal))
  {
    candidate = std::nextafter(candidate, previous);
  }
  return candidate;
}

void validateCommand(const HolonomicBodyCommand & command)
{
  if (!commandIsFinite(command)) {
    throw std::domain_error("holonomic command must be finite");
  }
}

void validateExternalSpeedLimit(double external_speed_limit)
{
  if (!externalSpeedLimitIsValid(external_speed_limit)) {
    throw std::domain_error(
            "external speed limit must be nonnegative finite or positive infinity");
  }
}

}  // namespace

HolonomicCommandLimiter::HolonomicCommandLimiter(
  const HolonomicCommandLimiterConfig & config)
: config_(config)
{
  if (!configIsValid(config_)) {
    throw std::invalid_argument("invalid holonomic command limiter configuration");
  }
}

void HolonomicCommandLimiter::seed(
  const HolonomicBodyCommand & measured,
  double external_speed_limit)
{
  validateCommand(measured);
  validateExternalSpeedLimit(external_speed_limit);
  previous_command_ = projectBaseLimits(measured, config_).command;
  applyExternalSpeedLimit(previous_command_, external_speed_limit);
  seeded_ = true;
}

HolonomicCommandLimitResult HolonomicCommandLimiter::limit(
  const HolonomicBodyCommand & raw,
  const HolonomicBodyCommand & measured,
  double dt,
  double external_speed_limit)
{
  validateCommand(raw);
  validateCommand(measured);
  validateExternalSpeedLimit(external_speed_limit);

  const auto target = projectBaseLimits(raw, config_);
  HolonomicCommandLimitResult result;
  result.projected_target = target.command;
  result.vx_saturated = target.vx_saturated;
  result.vy_saturated = target.vy_saturated;
  result.xy_saturated = target.xy_saturated;
  result.omega_saturated = target.omega_saturated;

  if (!seeded_ || !std::isfinite(dt) || dt <= 0.0 ||
    dt > config_.control_gap_threshold)
  {
    result.command = projectBaseLimits(measured, config_).command;
    result.external_limit_applied =
      applyExternalSpeedLimit(result.command, external_speed_limit);
    result.translation_lambda = 0.0;
    result.reseeded = true;
    previous_command_ = result.command;
    seeded_ = true;
    return result;
  }

  const long double delta_vx =
    static_cast<long double>(target.command.vx) - previous_command_.vx;
  const long double delta_vy =
    static_cast<long double>(target.command.vy) - previous_command_.vy;
  const long double max_delta_vx =
    static_cast<long double>(config_.max_ax) * dt;
  const long double max_delta_vy =
    static_cast<long double>(config_.max_ay) * dt;
  const long double lambda_x = delta_vx == 0.0L ?
    1.0L : max_delta_vx / std::abs(delta_vx);
  const long double lambda_y = delta_vy == 0.0L ?
    1.0L : max_delta_vy / std::abs(delta_vy);
  const long double translation_lambda =
    std::min({1.0L, lambda_x, lambda_y});
  result.translation_lambda = static_cast<double>(translation_lambda);
  result.ax_limited = lambda_x < 1.0L;
  result.ay_limited = lambda_y < 1.0L;
  result.command.vx = quantizeTowardPrevious(
    static_cast<long double>(previous_command_.vx) + translation_lambda * delta_vx,
    previous_command_.vx);
  result.command.vy = quantizeTowardPrevious(
    static_cast<long double>(previous_command_.vy) + translation_lambda * delta_vy,
    previous_command_.vy);
  if (!translationCandidateIsSafe(
      result.command, previous_command_, config_, max_delta_vx, max_delta_vy))
  {
    result.command.vx = previous_command_.vx;
    result.command.vy = previous_command_.vy;
    result.translation_lambda = 0.0;
  } else if (result.command.vx == previous_command_.vx &&
    result.command.vy == previous_command_.vy)
  {
    result.translation_lambda = 0.0;
  }

  const long double delta_omega =
    static_cast<long double>(target.command.omega) - previous_command_.omega;
  const long double max_delta_omega =
    static_cast<long double>(config_.max_alpha) * dt;
  result.alpha_limited = std::abs(delta_omega) > max_delta_omega;
  const double omega_candidate = quantizeTowardPrevious(
    static_cast<long double>(previous_command_.omega) + std::clamp(
      delta_omega, -max_delta_omega, max_delta_omega),
    previous_command_.omega);
  const long double actual_delta_omega = std::abs(
    static_cast<long double>(omega_candidate) - previous_command_.omega);
  if (std::isfinite(omega_candidate) &&
    actual_delta_omega <= max_delta_omega &&
    std::abs(static_cast<long double>(omega_candidate)) <= config_.max_omega)
  {
    result.command.omega = omega_candidate;
  } else {
    result.command.omega = previous_command_.omega;
  }

  result.external_limit_applied =
    applyExternalSpeedLimit(result.command, external_speed_limit);
  previous_command_ = result.command;
  return result;
}

}  // namespace pid_tracking_controller
