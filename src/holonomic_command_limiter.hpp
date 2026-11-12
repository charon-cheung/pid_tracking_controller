#ifndef PID_TRACKING_CONTROLLER__HOLONOMIC_COMMAND_LIMITER_HPP_
#define PID_TRACKING_CONTROLLER__HOLONOMIC_COMMAND_LIMITER_HPP_

#include <limits>
#include <optional>

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
  // Used when the distance-based stopping cap requires translation braking.
  double max_decel_x{0.5};
  double max_decel_y{0.5};
  double max_alpha{1.0};
  double control_gap_threshold{0.5};
  // Unset preserves max_alpha; applies only after confirmed rotation admission.
  std::optional<double> rotation_max_alpha;
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
  // The cap was not reachable this cycle without exceeding command rate limits.
  bool stopping_limit_unreachable{false};
};

enum class HolonomicSafetyMode {kInactive, kBraking, kHold, kEmergency};

struct HolonomicBrakeResult
{
  HolonomicBodyCommand command;
  // Hold means zero COMMAND only. Emergency has no rate-bound guarantee.
  HolonomicSafetyMode mode{HolonomicSafetyMode::kEmergency};
};

class HolonomicCommandLimiter
{
public:
  explicit HolonomicCommandLimiter(const HolonomicCommandLimiterConfig & config);

  void seed(
    const HolonomicBodyCommand & measured,
    double external_speed_limit = std::numeric_limits<double>::infinity());

  // The speed planner uses the final output, including all downstream limits.
  std::optional<HolonomicBodyCommand> previousCommand() const
  {
    return seeded_ ? std::optional<HolonomicBodyCommand>{previous_command_} : std::nullopt;
  }

  // Computes a candidate, not a safety release; the downstream gate logs release.
  HolonomicCommandLimitResult limit(
    const HolonomicBodyCommand & raw,
    const HolonomicBodyCommand & measured,
    double dt,
    double external_speed_limit = std::numeric_limits<double>::infinity(),
    double stopping_speed_limit = std::numeric_limits<double>::infinity(),
    bool rotation_acceleration = false);

  // Explicit stop request. accepted_output MUST be the last issued command,
  // captured before limit() computes a candidate; nullopt means unknown history.
  // Zero command / measured velocity alone are not physical standstill confirmation.
  HolonomicBrakeResult brake(const HolonomicBodyCommand & measured, double dt,
    std::optional<HolonomicBodyCommand> accepted_output);
  // Used only after downstream safety validation; preserves the slew history.
  void commitSafetyOutput(const HolonomicBodyCommand & command);

private:
  void logSafetyState(HolonomicSafetyMode mode, const HolonomicBodyCommand & command,
    const HolonomicBodyCommand & measured, double dt, const char * reason);
  HolonomicSafetyMode safety_mode_{HolonomicSafetyMode::kInactive};
  HolonomicCommandLimiterConfig config_;
  HolonomicBodyCommand previous_command_;
  bool seeded_{false};
  // Initial turn direction; opposite-direction corrections keep max_alpha.
  double rotation_direction_{0.0};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__HOLONOMIC_COMMAND_LIMITER_HPP_
