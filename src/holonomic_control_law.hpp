#ifndef PID_TRACKING_CONTROLLER__HOLONOMIC_CONTROL_LAW_HPP_
#define PID_TRACKING_CONTROLLER__HOLONOMIC_CONTROL_LAW_HPP_

namespace pid_tracking_controller
{

struct HolonomicControlInput
{
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_yaw{0.0};
  double reference_x{0.0};
  double reference_y{0.0};
  double path_tangent{0.0};
  double target_yaw{0.0};
  double target_yaw_rate_per_meter{0.0};
  double reference_velocity{0.0};
};

struct HolonomicControlGains
{
  double kp_x{0.0};
  double kp_y{0.0};
  double kp_yaw{0.0};
};

struct HolonomicRawCommand
{
  double ex_world{0.0};
  double ey_world{0.0};
  double ex_body{0.0};
  double ey_body{0.0};
  double yaw_error{0.0};
  double vx_ff{0.0};
  double vy_ff{0.0};
  double omega_ff{0.0};
  double vx_fb{0.0};
  double vy_fb{0.0};
  double omega_fb{0.0};
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

HolonomicRawCommand computeHolonomicRawCommand(
  const HolonomicControlInput & input,
  const HolonomicControlGains & gains);

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__HOLONOMIC_CONTROL_LAW_HPP_
