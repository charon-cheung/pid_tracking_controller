#include "execution_safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include "rclcpp/logging.hpp"

namespace pid_tracking_controller
{
namespace
{
using M = MotionObservation;
bool moving(const HolonomicBodyCommand & c)
{return std::hypot(c.vx, c.vy) > 1e-6 || std::abs(c.omega) > 1e-6;}
const char * statusName(SweepStatus s)
{return s == SweepStatus::kClear ? "clear" : s == SweepStatus::kCollision ? "collision" : "unavailable";}

// A newer odometry header may precede /clock; require the exact older pose-time
// anchor, never interpolate or rewrite timestamps. Invalid later records still fail.
const char * alignedOdometry(const M::History & h, int64_t pose, int64_t now,
  M::Wall::time_point wall, HolonomicBodyCommand & measured)
{
  if (std::strcmp(h.state, "ready") != 0) {return h.state;}
  if (pose <= 0 || now < pose || now - pose > 300000000LL) {return "pose_time_invalid";}
  const auto & series = h.odometry;
  const M::Sample * anchor = nullptr;
  for (size_t i = 0; i < series.size; ++i) {
    const auto & s = series.at(i);
    if (s.stamp_ns == pose) {anchor = &s;}
  }
  if (!anchor) {return "odometry_anchor_missing";}
  if (wall < anchor->received || wall - anchor->received > std::chrono::milliseconds(600)) {
    return "odometry_receipt_stale";
  }
  for (size_t i = 0; i < series.size; ++i) {
    const auto & s = series.at(i);
    if (s.count >= anchor->count && (std::strcmp(s.payload, "valid") != 0 ||
      std::strcmp(s.frame, "base") != 0)) {return "odometry_invalid";}
    if (s.stamp_ns == pose &&
      (s.vx != anchor->vx || s.vy != anchor->vy || s.omega != anchor->omega))
    {return "odometry_anchor_ambiguous";}
  }
  measured = {anchor->vx, anchor->vy, anchor->omega};
  return "ready";
}
// Reach the current target before braking when testing release. Testing only one
// tiny acceleration step at standstill would repeatedly reopen the gate.
double rampTime(double from, double to, double acceleration, double deceleration)
{
  if (from * to < 0.) {return std::abs(from) / deceleration + std::abs(to) / acceleration;}
  return std::abs(to - from) / (std::abs(to) > std::abs(from) ? acceleration : deceleration);
}
}  // namespace

ExecutionSafetyGate::ExecutionSafetyGate(const NominalTransitionAssumptions & nominal,
  const ExecutionSafetyOptions & options) : nominal_(nominal), options_(options) {}

void ExecutionSafetyGate::reset()
{
  inhibited_ = false; clear_cycles_ = 0; revision_ = epoch_ = odom_drops_ = 0;
  last_now_ = 0; last_report_.clear(); reported_at_ = {};
}

SafetyGateDecision ExecutionSafetyGate::apply(SafetyInputResult & geometry, const M::History & history,
  int64_t pose_stamp_ns, int64_t now_ns, uint64_t revision,
  const HolonomicBodyCommand & candidate, const HolonomicBodyCommand & measured,
  double dt, std::optional<HolonomicBodyCommand> accepted_output,
  HolonomicCommandLimiter & limiter, Wall::time_point cycle_started, const PathRiskTrigger & risk)
{
  SafetyGateDecision out;
  out.path_risk = risk.state;
  out.sweep_requested = risk.state == msg::PathRiskState::CLEARANCE_LOW ||
    risk.state == msg::PathRiskState::COLLISION_AHEAD;
  out.resume.reason = "not_required";
  if (revision != revision_ || (history.epoch && history.epoch != epoch_)) {
    clear_cycles_ = 0;
    revision_ = revision;
    if (history.epoch) {epoch_ = history.epoch;}
  }
  HolonomicBodyCommand aligned;
  out.reason = alignedOdometry(history, pose_stamp_ns, now_ns, Wall::now(), aligned);
  if (std::strcmp(history.state, "ready") == 0) {
    if (history.dropped != odom_drops_) {out.reason = "odometry_history_gap";}
    odom_drops_ = history.dropped;
  }
  out.motion_reason = out.reason;
  if (!geometry.snapshot || !geometry.snapshot->map) {
    out.reason = geometry.reason.empty() ? "geometry_missing" : geometry.reason;
  }
  if (!revision || now_ns <= 0 || (last_now_ && now_ns <= last_now_)) {
    out.reason = "control_time_or_revision_invalid";
  }
  last_now_ = now_ns;
  if (!out.sweep_requested && risk.state != msg::PathRiskState::CLEAR) {
    out.reason = risk.reason;
    if (out.reason == "ready") {out.reason = "path_risk_unavailable";}
  }
  if (out.reason == "ready" && risk.state == msg::PathRiskState::CLEAR) {
    // Path CLEAR is only the trigger policy, not an evaluated motion-sweep result.
    inhibited_ = false; clear_cycles_ = 0;
    out.command = candidate; out.action = "path_clear_bypass"; out.reason = "path_clear";
    out.candidate.reason = out.output.reason = "not_checked_path_clear";
    limiter.commitSafetyOutput(out.command);
    return out;
  }
  const bool inputs_ready = out.reason == "ready" && out.sweep_requested;
  const auto evaluate = [&](const HolonomicBodyCommand & target, bool resume) {
      CommandSweepResult sweep;
      if (!inputs_ready) {sweep.reason = "inputs_unavailable"; return sweep;}
      auto limits = options_;
      const auto elapsed = std::chrono::duration<double>(Wall::now() - cycle_started).count();
      limits.budget_seconds -= elapsed;
      if (elapsed < 0. || !std::isfinite(limits.budget_seconds) || limits.budget_seconds <= 0.) {
        sweep.reason = "gate_budget_exceeded"; return sweep;
      }
      MotionTransitionInput model;
      model.measured = aligned; model.active_command = aligned;
      // The current candidate applies after NOW + nominal delay, not after the
      // older pose timestamp + delay. The intervening motion remains nominal.
      const double delay = (now_ns - pose_stamp_ns) * 1e-9 + nominal_.delay;
      model.active_valid_for = delay; model.candidate_execution = {delay, delay};
      model.candidate = target; model.candidate_duration = nominal_.duration;
      model.acceleration = nominal_.acceleration; model.deceleration = nominal_.deceleration;
      model.braking_deceleration = nominal_.braking_deceleration;
      if (resume) {
        const double ramp = std::max({
          rampTime(aligned.vx, target.vx, nominal_.acceleration.vx, nominal_.deceleration.vx),
          rampTime(aligned.vy, target.vy, nominal_.acceleration.vy, nominal_.deceleration.vy),
          rampTime(aligned.omega, target.omega, nominal_.acceleration.omega, nominal_.deceleration.omega)});
        // Rolling release probe: use today's PID direction/speed, not the old
        // rejected command. Include a short hold plus full braking, so a tiny
        // acceleration-only sweep is not sufficient evidence for restart.
        model.candidate_duration = std::max(nominal_.duration, .2) + ramp;
      }
      ++out.evaluations;
      return ExecutionSafety(limits).checkTransition(geometry.snapshot->pose, model,
        *geometry.snapshot->map, geometry.snapshot->footprint);
    };
  out.candidate = evaluate(candidate, false);
  if (inputs_ready) {out.reason = out.candidate.reason;}
  bool clear = out.candidate.status == SweepStatus::kClear;
  if (!clear) {
    inhibited_ = true; clear_cycles_ = 0;
  } else if (inhibited_) {
    if (moving(candidate)) {
      out.resume = evaluate(candidate, true);
      clear = out.resume.status == SweepStatus::kClear;
      if (!clear) {out.reason = out.resume.reason;}
    } else {
      clear = false; out.reason = "resume_motion_missing";
    }
    // Duplicate ROS-time cycles never accumulate confirmations (inputs_ready=false).
    clear_cycles_ = clear ? clear_cycles_ + 1 : 0;
    if (clear_cycles_ >= 3) {
      inhibited_ = false; out.action = "released";
    } else if (clear) {out.reason = "resume_confirming";}
  }
  out.command = candidate;
  if (inhibited_) {
    const auto brake = limiter.brake(measured, dt, accepted_output);
    out.command = brake.command;
    out.action = brake.mode == HolonomicSafetyMode::kEmergency ? "emergency_zero" :
      brake.mode == HolonomicSafetyMode::kHold ? "hold" : "braking";
    out.output = evaluate(out.command, false);
    if (out.output.status == SweepStatus::kCollision) {
      // Nominal braking already intersects: zero is the strongest available stop
      // request. Do not advertise it as a collision-free or physical stop proof.
      out.command = {}; out.action = "emergency_zero";
      out.reason = "braking_collision";
      out.output = evaluate(out.command, false);
    }
    limiter.commitSafetyOutput(out.command);
  } else {
    out.output = out.candidate;
    limiter.commitSafetyOutput(out.command);
  }
  out.inhibited = inhibited_; out.clear_cycles = clear_cycles_;
  return out;
}

void ExecutionSafetyGate::report(const rclcpp::Logger & logger, const SafetyGateDecision & d,
  const HolonomicBodyCommand & candidate, uint64_t revision, int64_t now_ns,
  int64_t pose_stamp_ns, int64_t map_stamp_ns, double elapsed_ms, const M::History & history)
{
  const auto wall = Wall::now();
  const auto key = std::to_string(revision) + ':' + pathRiskName(d.path_risk) + ':' + d.action +
    ':' + d.reason + ':' + d.motion_reason + ':' + std::to_string(history.epoch) +
    ':' + std::to_string(history.dropped);
  if (key == last_report_ && wall - reported_at_ < std::chrono::seconds(1)) {return;}
  last_report_ = key; reported_at_ = wall;
  const auto & series = history.odometry;
  const M::Sample missing;
  const auto & latest = series.size ? series.at(series.size - 1) : missing;
  const M::Sample * anchor = nullptr;
  for (size_t i = 0; i < series.size; ++i) {
    if (series.at(i).stamp_ns == pose_stamp_ns) {anchor = &series.at(i);}
  }
  const auto & sample = anchor ? *anchor : missing;
  RCLCPP_INFO(logger, "PID_MOTION stage=2-A revision=%lu history_revision=%lu epoch=%lu last_reset=%s "
    "history_state=%s alignment=%s samples=%zu dropped=%lu overwritten=%lu "
    "now=%.9f pose_stamp=%.9f latest_stamp=%.9f latest_payload=%s anchor_found=%d "
    "anchor_count=%lu anchor_age=%.6f measured=(%.6f,%.6f,%.6f) "
    "candidate=(%.6f,%.6f,%.6f) effective_delay=%.6f prediction_requested=%d",
    revision, history.revision, history.epoch, history.reset_reason, history.state,
    d.motion_reason.c_str(), series.size, history.dropped, series.overwritten,
    now_ns * 1e-9, pose_stamp_ns * 1e-9, latest.stamp_ns * 1e-9, latest.payload, anchor != nullptr,
    sample.count, anchor ? std::chrono::duration<double>(wall - sample.received).count() : -1.,
    sample.vx, sample.vy, sample.omega, candidate.vx, candidate.vy, candidate.omega,
    (now_ns - pose_stamp_ns) * 1e-9 + nominal_.delay, d.sweep_requested);
  const double unknown = std::numeric_limits<double>::quiet_NaN();
  RCLCPP_INFO(logger, "PID_GATE stage=2-C evaluation=2-B basis=nominal revision=%lu now=%.9f pose_age=%.6f map_age=%.6f "
    "path_risk=%s sweep_requested=%d evaluations=%u candidate=(%.6f,%.6f,%.6f) candidate_state=%s candidate_reason=%s phase=%d hit_t=%.6f "
    "resume_state=%s resume_reason=%s action=%s reason=%s clear_cycles=%u "
    "resume_policy=current_candidate resume_hold=%.6f resume_t=%.6f resume_distance=%.6f "
    "candidate_hit_pose=(%.6f,%.6f,%.6f) candidate_cell=(%.6f,%.6f) "
    "resume_hit_pose=(%.6f,%.6f,%.6f) resume_cell=(%.6f,%.6f) "
    "output=(%.6f,%.6f,%.6f) output_state=%s output_reason=%s total_ms=%.3f "
    "physical_response=unverified queued_commands=unmodeled steering=unmodeled",
    revision, now_ns * 1e-9, (now_ns - pose_stamp_ns) * 1e-9,
    map_stamp_ns ? (now_ns - map_stamp_ns) * 1e-9 : -1., pathRiskName(d.path_risk), d.sweep_requested, d.evaluations,
    candidate.vx, candidate.vy, candidate.omega,
    statusName(d.candidate.status), d.candidate.reason, static_cast<int>(d.candidate.phase), d.candidate.time,
    statusName(d.resume.status), d.resume.reason, d.action, d.reason.c_str(), d.clear_cycles,
    std::max(nominal_.duration, .2), d.resume.time, d.resume.distance,
    d.candidate.pose.x, d.candidate.pose.y, d.candidate.pose.yaw,
    d.candidate.collision_cell ? d.candidate.collision_cell->x : unknown,
    d.candidate.collision_cell ? d.candidate.collision_cell->y : unknown,
    d.resume.pose.x, d.resume.pose.y, d.resume.pose.yaw,
    d.resume.collision_cell ? d.resume.collision_cell->x : unknown,
    d.resume.collision_cell ? d.resume.collision_cell->y : unknown,
    d.command.vx, d.command.vy, d.command.omega, statusName(d.output.status), d.output.reason, elapsed_ms);
}
}  // namespace pid_tracking_controller
