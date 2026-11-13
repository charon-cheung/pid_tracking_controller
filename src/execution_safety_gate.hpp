#ifndef PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_GATE_HPP_
#define PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_GATE_HPP_

#include <deque>
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
  bool stopped{false};
  int64_t aligned_stamp_ns{0};
  double risk_distance{std::numeric_limits<double>::quiet_NaN()}, stopping_distance{0.};
  const char * mode{"tracking"};
  unsigned clear_cycles{0}, evaluations{0};
};

// Single control-thread owner. A synchronous nominal gate, never a consumer of
// background sweep results. Valid path CLEAR skips the sweep; yellow/red run it.
// Issued-output timing is nominal; no physical actuator/steering guarantees.
class ExecutionSafetyGate
{
public:
  using Wall = MotionObservation::Wall;
  explicit ExecutionSafetyGate(const NominalTransitionAssumptions & nominal,
    const ExecutionSafetyOptions & options = {}, bool simulation = false);
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
  struct IssuedOutput {int64_t stamp; HolonomicBodyCommand command;};
  void commit(const HolonomicBodyCommand & command, int64_t now, HolonomicCommandLimiter & limiter);
  std::deque<IssuedOutput> issued_;
  std::optional<HolonomicBodyCommand> last_output_;
  bool inhibited_{false};
  bool simulation_{false}, obstacle_stop_{false};
  Wall::time_point last_tick_wall_{};
  int64_t zero_since_{0}, low_since_{0}, confirmed_pose_{0}, confirmed_risk_{0};
  unsigned clear_cycles_{0};
  uint64_t revision_{0}, epoch_{0}, odom_drops_{0};
  int64_t last_now_{0};
  std::string last_report_, last_motion_report_;
  Wall::time_point reported_at_{};
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_GATE_HPP_
