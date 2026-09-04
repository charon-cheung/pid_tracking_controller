#include "holonomic_control_law.hpp"

#include <cmath>
#include <stdexcept>

#include "angles/angles.h"

namespace pid_tracking_controller
{
namespace
{

bool inputIsFinite(const HolonomicControlInput & input)
{
  return std::isfinite(input.robot_x) &&
         std::isfinite(input.robot_y) &&
         std::isfinite(input.robot_yaw) &&
         std::isfinite(input.reference_x) &&
         std::isfinite(input.reference_y) &&
         std::isfinite(input.path_tangent) &&
         std::isfinite(input.target_yaw) &&
         std::isfinite(input.target_yaw_rate_per_meter) &&
         std::isfinite(input.reference_velocity);
}

bool gainsAreValid(const HolonomicControlGains & gains)
{
  return std::isfinite(gains.kp_x) && gains.kp_x >= 0.0 &&
         std::isfinite(gains.kp_y) && gains.kp_y >= 0.0 &&
         std::isfinite(gains.kp_yaw) && gains.kp_yaw >= 0.0;
}

bool commandIsFinite(const HolonomicRawCommand & command)
{
  return std::isfinite(command.ex_world) &&
         std::isfinite(command.ey_world) &&
         std::isfinite(command.ex_body) &&
         std::isfinite(command.ey_body) &&
         std::isfinite(command.yaw_error) &&
         std::isfinite(command.vx_ff) &&
         std::isfinite(command.vy_ff) &&
         std::isfinite(command.omega_ff) &&
         std::isfinite(command.vx_fb) &&
         std::isfinite(command.vy_fb) &&
         std::isfinite(command.omega_fb) &&
         std::isfinite(command.vx) &&
         std::isfinite(command.vy) &&
         std::isfinite(command.omega);
}

}  // namespace

HolonomicRawCommand computeHolonomicRawCommand(
  const HolonomicControlInput & input,
  const HolonomicControlGains & gains)
{
  if (!gainsAreValid(gains)) {
    throw std::invalid_argument("holonomic control gains must be finite and nonnegative");
  }
  if (!inputIsFinite(input) || input.reference_velocity < 0.0) {
    throw std::domain_error(
            "holonomic control input must be finite with nonnegative reference velocity");
  }

  HolonomicRawCommand command;
  command.ex_world = input.reference_x - input.robot_x;
  command.ey_world = input.reference_y - input.robot_y;

  const double cosine = std::cos(input.robot_yaw);
  const double sine = std::sin(input.robot_yaw);
  command.ex_body = cosine * command.ex_world + sine * command.ey_world;
  command.ey_body = -sine * command.ex_world + cosine * command.ey_world;

  const double vx_ff_world = input.reference_velocity * std::cos(input.path_tangent);
  const double vy_ff_world = input.reference_velocity * std::sin(input.path_tangent);
  command.vx_ff = cosine * vx_ff_world + sine * vy_ff_world;
  command.vy_ff = -sine * vx_ff_world + cosine * vy_ff_world;

  command.vx_fb = gains.kp_x * command.ex_body;
  command.vy_fb = gains.kp_y * command.ey_body;

  command.yaw_error = angles::normalize_angle(input.target_yaw - input.robot_yaw);
  command.omega_ff =
    input.target_yaw_rate_per_meter * input.reference_velocity;
  command.omega_fb = gains.kp_yaw * command.yaw_error;

  command.vx = command.vx_ff + command.vx_fb;
  command.vy = command.vy_ff + command.vy_fb;
  command.omega = command.omega_ff + command.omega_fb;

  if (!commandIsFinite(command)) {
    throw std::domain_error("holonomic control law produced non-finite output");
  }
  return command;
}

}  // namespace pid_tracking_controller
