#ifndef PID_TRACKING_CONTROLLER__TRANSLATION_TRACKING_GUARD_HPP_
#define PID_TRACKING_CONTROLLER__TRANSLATION_TRACKING_GUARD_HPP_

#include <optional>

namespace pid_tracking_controller
{

struct TranslationTrackingGuardConfig
{
  double enter_error{0.03};
  double exit_error{0.015};
  // Minimum measured outward speed or same-side position-error growth rate (m/s).
  double min_outward_speed{0.02};
  double confirm_time{0.15};
  double release_confirm_time{0.4};
};

// Holds feedforward acceleration during sustained outward drift in translation.
// Errors are robot-minus-path along the path's left normal, in metres.
class TranslationTrackingGuard
{
public:
  explicit TranslationTrackingGuard(const TranslationTrackingGuardConfig & config = {});
  bool update(double lateral_error, double measured_normal_speed, double dt);
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
