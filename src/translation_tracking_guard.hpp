#ifndef PID_TRACKING_CONTROLLER__TRANSLATION_TRACKING_GUARD_HPP_
#define PID_TRACKING_CONTROLLER__TRANSLATION_TRACKING_GUARD_HPP_

#include <optional>

namespace pid_tracking_controller
{

struct TranslationTrackingGuardConfig
{
  double enter_error{0.03};
  double exit_error{0.015};
  double min_outward_speed{0.02};
  double confirm_time{0.15};
};

// Holds feedforward acceleration when a turn is producing sustained outward drift.
// Errors are robot-minus-path along the path's left normal, in metres.
class TranslationTrackingGuard
{
public:
  explicit TranslationTrackingGuard(const TranslationTrackingGuardConfig & config = {});
  bool update(
    double lateral_error, double measured_normal_speed, double local_tangent_rate,
    double dt);
  void reset();

private:
  TranslationTrackingGuardConfig config_;
  std::optional<double> previous_error_;
  double confirmation_duration_{0.0};
  bool previous_qualifying_{false};
  bool holding_{false};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__TRANSLATION_TRACKING_GUARD_HPP_
