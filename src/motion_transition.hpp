#ifndef PID_TRACKING_CONTROLLER__MOTION_TRANSITION_HPP_
#define PID_TRACKING_CONTROLLER__MOTION_TRANSITION_HPP_

#include <limits>
#include <optional>
#include <vector>

#include "holonomic_command_limiter.hpp"

namespace pid_tracking_controller
{
enum class SweepPhase {kInitial, kHold, kBraking, kDelay, kTransition};

// Nominal response assumptions shared by 2-A prediction and the online gate.
// Not actuator evidence: pending commands and steering dynamics remain unmodeled.
struct NominalTransitionAssumptions
{
  double delay{std::numeric_limits<double>::quiet_NaN()};
  double duration{std::numeric_limits<double>::quiet_NaN()};
  HolonomicBodyCommand acceleration, deceleration, braking_deceleration;
};

struct ExecutionWindow
{
  double earliest{std::numeric_limits<double>::quiet_NaN()};
  double latest{std::numeric_limits<double>::quiet_NaN()};
};

struct PendingMotionCommand
{
  HolonomicBodyCommand velocity;
  ExecutionWindow execution;
  double expires_after{std::numeric_limits<double>::quiet_NaN()};
  bool issued{false};
};

struct MotionTransitionInput
{
  // All times are relative to the same aligned initial pose/measured state.
  // Velocities are body-frame setpoints, not world-frame vectors frozen at t=0.
  HolonomicBodyCommand measured;
  std::optional<HolonomicBodyCommand> active_command;
  double active_valid_for{std::numeric_limits<double>::quiet_NaN()};
  std::vector<PendingMotionCommand> pending;
  HolonomicBodyCommand candidate;
  ExecutionWindow candidate_execution;
  // Time from candidate application until braking, including any unfinished ramp.
  double candidate_duration{std::numeric_limits<double>::quiet_NaN()};
  // Explicit deterministic response assumptions, not inferred physical guarantees.
  // Axes decelerate to zero before reversing; there is no steering-joint model.
  HolonomicBodyCommand acceleration;
  HolonomicBodyCommand deceleration;
  HolonomicBodyCommand braking_deceleration;
};

struct MotionSegment
{
  HolonomicBodyCommand start, end;
  double duration;
  SweepPhase phase;
};

struct MotionTransition
{
  bool available{false};
  const char * reason{"invalid_motion_input"};
  std::vector<MotionSegment> segments;
};

// Explicit model times are required: uncertain intervals are NOT
// approximated by endpoints. No asynchronous MotionObservation snapshot conversion.
// Issued entries are ordered by execution time; expired/unissued entries are ignored.
// Limits: 16 pending entries / 128 segments. Unavailable results are never a safe path.
MotionTransition buildMotionTransition(const MotionTransitionInput & input);

// Legacy constant-body-velocity hold followed by full independent-axis braking.
MotionTransition buildCommandMotion(const HolonomicBodyCommand & velocity,
  double hold_time, const HolonomicBodyCommand & deceleration);
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__MOTION_TRANSITION_HPP_
