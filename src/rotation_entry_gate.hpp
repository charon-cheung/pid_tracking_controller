#ifndef PID_TRACKING_CONTROLLER__ROTATION_ENTRY_GATE_HPP_
#define PID_TRACKING_CONTROLLER__ROTATION_ENTRY_GATE_HPP_

#include "holonomic_command_limiter.hpp"

namespace pid_tracking_controller
{

struct RotationEntryGateConfig
{
  double xy_speed_tolerance{0.05};
  double omega_tolerance{0.05};
  double stop_confirm_time{0.20};
  double braking_timeout{8.0};
};

// Owns stop-before-rotation admission, not yaw completion or path progress.
class RotationEntryGate
{
public:
  explicit RotationEntryGate(const RotationEntryGateConfig & config);

  // Recheck the latest measurement before the first admitted rotation command.
  // Once admitted, rotation stays enabled until reset (omega is then expected).
  bool beginCycle(const HolonomicBodyCommand & measured);
  void observeBraking(
    const HolonomicBodyCommand & measured, const HolonomicBodyCommand & command, double dt);
  void reset(bool preserve_braking = false);
  double stoppedDuration() const {return stopped_duration_;}

private:
  bool stopped(const HolonomicBodyCommand & velocity) const;
  RotationEntryGateConfig config_;
  double stopped_duration_{0.0};
  double braking_duration_{0.0};
  bool observed_{false};
  bool previous_stopped_{false};
  bool ready_{false};
  bool rotating_{false};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__ROTATION_ENTRY_GATE_HPP_
