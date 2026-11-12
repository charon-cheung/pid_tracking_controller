#ifndef PID_TRACKING_CONTROLLER__MOTION_TRANSITION_OBSERVER_HPP_
#define PID_TRACKING_CONTROLLER__MOTION_TRANSITION_OBSERVER_HPP_

#include "execution_safety_inputs.hpp"
#include "motion_observation.hpp"

namespace pid_tracking_controller
{
// Explicit executor evidence, never synthesized from publication/receipt timestamps.
// The caller must supply the complete active/pending queue and known response model,
// with all model times relative to pose_stamp_ns. Identity binds it to one observation.
struct TransitionExecutionEvidence
{
  uint64_t epoch{0}, revision{0}, candidate_count{0};
  int64_t pose_stamp_ns{0};
  bool queue_complete{false};
  MotionTransitionInput model;
};

// Observation-only hypothesis, NOT execution evidence or a physical response bound.
// Hold measured body velocity for delay, approach the candidate for duration,
// then brake fully. Outstanding commands and steering dynamics are not modeled.
struct NominalTransitionAssumptions
{
  double delay{std::numeric_limits<double>::quiet_NaN()};
  double duration{std::numeric_limits<double>::quiet_NaN()};
  HolonomicBodyCommand acceleration, deceleration, braking_deceleration;
};

struct TransitionObservation
{
  CommandSweepResult sweep;
  // Never copy this conditional prediction into the execution-evidence result.
  std::optional<CommandSweepResult> nominal_sweep;
  std::string geometry_reason, motion_reason;
  bool execution_present{false}, queue_complete{false}, evaluated{false};
  uint64_t candidate_count{0};
  size_t deferred_odometry{0};
  int64_t pose_stamp_ns{0}, map_stamp_ns{0}, candidate_stamp_ns{0};
  double evaluate_ms{0}, total_ms{0};
};

// Single observation-timer owner. Results have no control-path consumers.
class MotionTransitionObserver
{
public:
  using Wall = MotionObservation::Wall;
  explicit MotionTransitionObserver(const ExecutionSafetyOptions & options = {}) : options_(options) {}
  TransitionObservation evaluate(SafetyInputResult & geometry,
    const MotionObservation::History & history, int64_t pose_stamp_ns, int64_t ros_now,
    Wall::time_point cycle_started, const TransitionExecutionEvidence * execution = nullptr,
    const NominalTransitionAssumptions * nominal = nullptr);
  void report(const rclcpp::Logger & logger, const TransitionObservation & result,
    const MotionObservation::History & history, int64_t ros_now, double capture_ms, double history_ms);
  void stopped(const rclcpp::Logger & logger) const;
  void reset();

private:
  ExecutionSafetyOptions options_;
  std::string last_key_;
  Wall::time_point reported_at_{};
  uint64_t nominal_evaluated_{0}, nominal_clear_{0}, nominal_collision_{0}, nominal_unavailable_{0};
  uint64_t reports_{0}, clear_{0}, collision_{0}, unavailable_{0}, evaluated_{0};
  // Availability denominators exclude idle periods without a recent candidate.
  uint64_t recent_candidate_cycles_{0}, motion_ready_cycles_{0}, gap_cycles_{0};
  uint64_t future_odom_cycles_{0}, execution_missing_cycles_{0};
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__MOTION_TRANSITION_OBSERVER_HPP_
