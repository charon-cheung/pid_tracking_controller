#ifndef PID_TRACKING_CONTROLLER__FIXED_HEADING_YAW_GATE_HPP_
#define PID_TRACKING_CONTROLLER__FIXED_HEADING_YAW_GATE_HPP_

#include <optional>

#include "holonomic_control_law.hpp"
#include "holonomic_tracking_reference.hpp"

namespace pid_tracking_controller
{

struct FixedHeadingYawGateConfig
{
  bool enabled{true};
  double enter_error{0.03490658503988659};  // 2 degrees, radians.
  double exit_error{0.017453292519943295};  // 1 degree, radians.
};

// Suppress small yaw corrections only while the path requests fixed-heading translation.
// The final command limiter still owns angular acceleration and deceleration.
class FixedHeadingYawGate
{
public:
  explicit FixedHeadingYawGate(const FixedHeadingYawGateConfig & config = {});
  void apply(const HolonomicTrackingReference & reference, HolonomicRawCommand & command);
  void reset();
  bool suppressing() const {return suppressing_;}

private:
  FixedHeadingYawGateConfig config_;
  std::optional<double> target_yaw_;
  bool correcting_{false};
  bool suppressing_{false};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__FIXED_HEADING_YAW_GATE_HPP_
