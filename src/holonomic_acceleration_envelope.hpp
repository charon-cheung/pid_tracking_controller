#ifndef PID_TRACKING_CONTROLLER__HOLONOMIC_ACCELERATION_ENVELOPE_HPP_
#define PID_TRACKING_CONTROLLER__HOLONOMIC_ACCELERATION_ENVELOPE_HPP_

#include "holonomic_speed_planner.hpp"

namespace pid_tracking_controller
{

// Nominal translation preview; the current-cycle reachable set and final limiter
// still account for actual heading, position feedback and prior output.
double translationAccelerationSpeedLimit(
  const PreparedPath & path, const HolonomicTrackingReference & reference,
  double robot_yaw, double dt, double external_speed_limit,
  const HolonomicSpeedPlannerConfig & config);

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__HOLONOMIC_ACCELERATION_ENVELOPE_HPP_
