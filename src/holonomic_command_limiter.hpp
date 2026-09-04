#ifndef PID_TRACKING_CONTROLLER__HOLONOMIC_COMMAND_LIMITER_HPP_
#define PID_TRACKING_CONTROLLER__HOLONOMIC_COMMAND_LIMITER_HPP_

#include <limits>

namespace pid_tracking_controller
{

struct HolonomicBodyCommand
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct HolonomicCommandLimiterConfig
{
  double max_vx{0.8};
  double max_vy{0.8};
  double max_xy_speed{0.8};
  double max_omega{0.8};
  double max_ax{0.5};
  double max_ay{0.5};
  double max_alpha{1.0};
  double control_gap_threshold{0.5};
};

struct HolonomicCommandLimitResult
{
  // Raw command after base velocity projection, before slew and external limiting.
  HolonomicBodyCommand projected_target;
  // Final command returned and retained as limiter state.
  HolonomicBodyCommand command;
  // Common translation interpolation factor; zero when this call reseeds.
  double translation_lambda{1.0};
  // Saturation flags describe raw-command base projection.
  bool vx_saturated{false};
  bool vy_saturated{false};
  bool xy_saturated{false};
  bool omega_saturated{false};
  // Slew flags identify each axis whose own per-cycle allowance was exceeded.
  bool ax_limited{false};
  bool ay_limited{false};
  bool alpha_limited{false};
  // True when the per-call external cap changed this call's final translation.
  bool external_limit_applied{false};
  bool reseeded{false};
};

class HolonomicCommandLimiter
{
public:
  explicit HolonomicCommandLimiter(const HolonomicCommandLimiterConfig & config);

  void seed(
    const HolonomicBodyCommand & measured,
    double external_speed_limit = std::numeric_limits<double>::infinity());

  HolonomicCommandLimitResult limit(
    const HolonomicBodyCommand & raw,
    const HolonomicBodyCommand & measured,
    double dt,
    double external_speed_limit = std::numeric_limits<double>::infinity());

private:
  HolonomicCommandLimiterConfig config_;
  HolonomicBodyCommand previous_command_;
  bool seeded_{false};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__HOLONOMIC_COMMAND_LIMITER_HPP_
