#ifndef PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_GATE_HPP_
#define PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_GATE_HPP_

#include "execution_safety_inputs.hpp"
#include "motion_observation.hpp"
#include "path_risk_trigger.hpp"

namespace pid_tracking_controller
{
struct SafetyGateDecision
{
  HolonomicBodyCommand command;
  CommandSweepResult candidate, resume, output;
  std::string reason;
  std::string motion_reason{"unavailable"};
  const char * action{"pass"};
  uint8_t path_risk{msg::PathRiskState::UNAVAILABLE};
  bool sweep_requested{false};
  bool inhibited{false};
  unsigned clear_cycles{0}, evaluations{0};
};

// Single control-thread owner. A synchronous nominal gate, never a consumer of
// background sweep results. Valid path CLEAR skips the sweep; yellow/red run it.
// Does not claim actuator/queued-command guarantees.
class ExecutionSafetyGate
{
public:
  using Wall = MotionObservation::Wall;
  explicit ExecutionSafetyGate(const NominalTransitionAssumptions & nominal,
    const ExecutionSafetyOptions & options = {});
  void reset();
  SafetyGateDecision apply(SafetyInputResult & geometry, const MotionObservation::History & history,
    int64_t pose_stamp_ns, int64_t now_ns, uint64_t revision,
    const HolonomicBodyCommand & candidate, const HolonomicBodyCommand & measured,
    double dt, std::optional<HolonomicBodyCommand> accepted_output,
    HolonomicCommandLimiter & limiter, Wall::time_point cycle_started, const PathRiskTrigger & risk);
  void report(const rclcpp::Logger & logger, const SafetyGateDecision & decision,
    const HolonomicBodyCommand & candidate, uint64_t revision, int64_t now_ns,
    int64_t pose_stamp_ns, int64_t map_stamp_ns, double elapsed_ms,
    const MotionObservation::History & history);

private:
  NominalTransitionAssumptions nominal_;
  ExecutionSafetyOptions options_;
  bool inhibited_{false};
  unsigned clear_cycles_{0};
  uint64_t revision_{0}, epoch_{0}, odom_drops_{0};
  int64_t last_now_{0};
  std::string last_report_;
  Wall::time_point reported_at_{};
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_GATE_HPP_
