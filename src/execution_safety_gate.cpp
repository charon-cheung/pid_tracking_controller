#include "execution_safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <iomanip>
#include <sstream>
#include <utility>
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

// Simulation callbacks can arrive out of order relative to /clock and TF.
// Select a nearby real sample within the tolerance; never rewrite its timestamp.
const char * alignedOdometry(const M::History & h, int64_t pose, int64_t now,
  M::Wall::time_point wall, HolonomicBodyCommand & measured, int64_t tolerance,
  int64_t & aligned_stamp)
{
  if (std::strcmp(h.state, "ready") != 0) {return h.state;}
  if (pose <= 0 || pose - now > tolerance || now - pose > 300000000LL) {return "pose_time_invalid";}
  const auto & series = h.odometry;
  const M::Sample * anchor = nullptr;
  for (size_t i = 0; i < series.size; ++i) {
    const auto & s = series.at(i);
    if (s.stamp_ns > 0 && s.stamp_ns - now <= tolerance &&
      std::abs(s.stamp_ns - pose) <= tolerance &&
      (!anchor || std::abs(s.stamp_ns - pose) <= std::abs(anchor->stamp_ns - pose)))
    {anchor = &s;}
  }
  if (!anchor) {return "odometry_anchor_missing";}
  if (wall < anchor->received || wall - anchor->received > std::chrono::milliseconds(600)) {
    return "odometry_receipt_stale";
  }
  for (size_t i = 0; i < series.size; ++i) {
    const auto & s = series.at(i);
    if (s.count >= anchor->count && (std::strcmp(s.payload, "valid") != 0 ||
      std::strcmp(s.frame, "base") != 0)) {return "odometry_invalid";}
    if (s.stamp_ns == anchor->stamp_ns &&
      (s.vx != anchor->vx || s.vy != anchor->vy || s.omega != anchor->omega))
    {return "odometry_anchor_ambiguous";}
  }
  measured = {anchor->vx, anchor->vy, anchor->omega};
  aligned_stamp = anchor->stamp_ns;
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
  const ExecutionSafetyOptions & options, bool simulation)
: nominal_(nominal), options_(options), simulation_(simulation) {}

void ExecutionSafetyGate::reset()
{
  inhibited_ = obstacle_stop_ = false; clear_cycles_ = 0; revision_ = epoch_ = odom_drops_ = 0;
  last_tick_wall_ = {};
  last_now_ = 0; last_report_.clear(); last_motion_report_.clear(); reported_at_ = {};
  issued_.clear(); last_output_.reset(); zero_since_ = low_since_ = confirmed_pose_ = confirmed_risk_ = 0;
}

void ExecutionSafetyGate::commit(const HolonomicBodyCommand & command, int64_t now,
  HolonomicCommandLimiter & limiter)
{
  limiter.commitSafetyOutput(command);
  last_output_ = command;  // Same-stamp safety overrides are still real outputs.
  if (moving(command)) {zero_since_ = 0;} else if (!zero_since_) {zero_since_ = now;}
  if (now <= 0 || (!issued_.empty() && now <= issued_.back().stamp)) {return;}
  // Coalesce identical outputs without renewing their original issue time.
  if (issued_.empty() || command.vx != issued_.back().command.vx ||
    command.vy != issued_.back().command.vy || command.omega != issued_.back().command.omega)
  {issued_.push_back({now, command});}
  // Keep the active command preceding the oldest usable pose and all later transitions.
  const auto cutoff = now - static_cast<int64_t>((nominal_.delay + .3) * 1e9);
  while (issued_.size() > 1 && issued_[1].stamp <= cutoff) {issued_.pop_front();}
  // A dropped transition is fenced in apply(), never silently treated as zero.
  while (issued_.size() > 65) {issued_.pop_front();}
}

SafetyGateDecision ExecutionSafetyGate::apply(SafetyInputResult & geometry, const M::History & history,
  int64_t pose_stamp_ns, int64_t now_ns, uint64_t revision,
  const HolonomicBodyCommand & candidate, const HolonomicBodyCommand & measured,
  double dt, std::optional<HolonomicBodyCommand> accepted_output,
  HolonomicCommandLimiter & limiter, Wall::time_point cycle_started, const PathRiskTrigger & risk)
{
  SafetyGateDecision out;
  out.path_risk = risk.state;
  out.resume.reason = "not_required";
  const bool hazard = risk.state == msg::PathRiskState::CLEARANCE_LOW ||
    risk.state == msg::PathRiskState::COLLISION_AHEAD;
  const auto wall_now = Wall::now();
  const bool repeat = simulation_ && now_ns > 0 && now_ns == last_now_ &&
    revision == revision_ && history.epoch == epoch_ && last_output_ &&
    wall_now - last_tick_wall_ <= std::chrono::milliseconds(200);
  // Repeated simulation ticks must not reseed the accepted output to zero.
  // Still evaluate new obstacle/input evidence; do not advance release counters.
  const auto target_command = repeat ? *last_output_ : candidate;
  if (repeat) {accepted_output = last_output_;}
  const bool clock_reset = last_now_ && now_ns < last_now_;
  if (revision != revision_ || clock_reset || (history.epoch && history.epoch != epoch_)) {
    clear_cycles_ = 0; confirmed_pose_ = confirmed_risk_ = low_since_ = 0;
    revision_ = revision;
    if (history.epoch) {epoch_ = history.epoch;}
  }
  if (clock_reset) {issued_.clear(); zero_since_ = 0;}
  HolonomicBodyCommand aligned;
  out.reason = alignedOdometry(history, pose_stamp_ns, now_ns, wall_now, aligned,
    simulation_ ? 100000000LL : 0LL, out.aligned_stamp_ns);
  if (std::strcmp(history.state, "ready") == 0) {
    if (!simulation_ && history.dropped != odom_drops_) {out.reason = "odometry_history_gap";}
    odom_drops_ = history.dropped;
  }
  out.motion_reason = out.reason;
  if (!geometry.snapshot || !geometry.snapshot->map) {
    out.reason = (geometry.reason.empty() || geometry.reason == "ready") ? "geometry_missing" : geometry.reason;
  }
  if (!revision || now_ns <= 0 || (last_now_ && now_ns <= last_now_ && !repeat)) {
    out.reason = "control_time_or_revision_invalid";
  }
  if (now_ns != last_now_) {last_tick_wall_ = wall_now;}
  last_now_ = now_ns;
  if (!hazard && risk.state != msg::PathRiskState::CLEAR) {
    out.reason = risk.reason;
    if (out.reason == "ready") {out.reason = "path_risk_unavailable";}
  }
  if (out.reason == "ready" && risk.state == msg::PathRiskState::COLLISION_AHEAD) {
    const auto & snapshot = *geometry.snapshot;
    if (!std::isfinite(risk.distance) || risk.distance < 0. ||
      !std::isfinite(risk.pose.x) || !std::isfinite(risk.pose.y) ||
      risk.frame != snapshot.header.frame_id || risk.frame.empty())
    {out.reason = "path_risk_geometry_invalid";} else {
      // Centre-to-risk-pose is a conservative shortcut of the remaining path.
      // The red envelope already includes the body: do not subtract robot width again.
      out.risk_distance = std::max(0., std::min(risk.distance,
        std::hypot(risk.pose.x - snapshot.pose.x, risk.pose.y - snapshot.pose.y)) - .05);
    }
  }
  out.sweep_requested = hazard || inhibited_;
  if (out.reason == "ready" && risk.state == msg::PathRiskState::CLEAR && !inhibited_) {
    out.command = target_command;
    out.action = repeat ? "repeat_output" : "path_clear_bypass";
    out.reason = repeat ? "simulation_clock_repeat" : "path_clear";
    out.candidate.reason = out.output.reason = "not_checked_path_clear";
    commit(out.command, now_ns, limiter);
    return out;
  }
  const bool inputs_ready = out.reason == "ready";
  // All observations at/after the pose anchor can veto stopped evidence.
  const auto low = [](const HolonomicBodyCommand & v) {
      return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.omega) &&
        std::hypot(v.vx, v.vy) <= .02 && std::abs(v.omega) <= .02;
    };
  bool still = out.motion_reason == "ready" && low(aligned) && low(measured);
  for (size_t i = 0; i < history.odometry.size; ++i) {
    const auto & sample = history.odometry.at(i);
    if (sample.stamp_ns >= pose_stamp_ns && !low({sample.vx, sample.vy, sample.omega})) {still = false;}
  }
  if (!still) {low_since_ = 0;} else if (!low_since_) {low_since_ = pose_stamp_ns;}
  const auto settle = static_cast<int64_t>(std::max(.2, nominal_.delay) * 1e9);
  out.stopped = still && zero_since_ > 0 && low_since_ > 0 &&
    now_ns - zero_since_ >= settle && pose_stamp_ns - low_since_ >= settle;
  const auto evaluate = [&](const HolonomicBodyCommand & target, bool resume) {
      CommandSweepResult worst;
      if (!inputs_ready) {worst.reason = "inputs_unavailable"; return worst;}
      MotionTransitionInput model;
      model.measured = aligned; model.active_command = aligned;
      const double delay = std::max(0., (now_ns - pose_stamp_ns) * 1e-9) + nominal_.delay;
      model.active_valid_for = delay; model.candidate_execution = {delay, delay};
      model.candidate = target; model.candidate_duration = nominal_.duration;
      model.acceleration = nominal_.acceleration; model.deceleration = nominal_.deceleration;
      model.braking_deceleration = nominal_.braking_deceleration;
      if (resume) {
        const double ramp = std::max({
          rampTime(aligned.vx, target.vx, nominal_.acceleration.vx, nominal_.deceleration.vx),
          rampTime(aligned.vy, target.vy, nominal_.acceleration.vy, nominal_.deceleration.vy),
          rampTime(aligned.omega, target.omega, nominal_.acceleration.omega, nominal_.deceleration.omega)});
        model.candidate_duration = std::max(nominal_.duration, .2) + ramp;
      }
      if (issued_.size() >= 65) {worst.reason = "issued_history_overflow"; return worst;}
      for (const auto & issued : issued_) {
        const double apply = (issued.stamp - pose_stamp_ns) * 1e-9 + nominal_.delay;
        if (apply <= 0.) {model.active_command = issued.command;}
        else if (apply < delay) {model.pending.push_back({issued.command, {apply, apply}, delay, true});}
      }
      const auto check = [&](const MotionTransitionInput & input) {
          CommandSweepResult sweep;
          auto limits = options_;
          const auto elapsed = std::chrono::duration<double>(Wall::now() - cycle_started).count();
          limits.budget_seconds -= elapsed;
          if (elapsed < 0. || !std::isfinite(limits.budget_seconds) || limits.budget_seconds <= 0.) {
            sweep.reason = "gate_budget_exceeded"; return sweep;
          }
          ++out.evaluations;
          sweep = ExecutionSafety(limits).checkTransition(geometry.snapshot->pose, input,
            *geometry.snapshot->map, geometry.snapshot->footprint);
          if (sweep.status == SweepStatus::kClear && !resume) {
            const auto motion = buildMotionTransition(input);
            double distance = 0.;
            for (const auto & segment : motion.segments) {
              // Convex norm: endpoint trapezoid bounds centre travel even while yaw changes.
              distance += .5 * segment.duration * (std::hypot(segment.start.vx, segment.start.vy) +
                std::hypot(segment.end.vx, segment.end.vy));
            }
            out.stopping_distance = std::max(out.stopping_distance, distance);
          }
          return sweep;
        };
      worst = check(model);
      if (worst.status != SweepStatus::kClear) {return worst;}
      // Also cover the most recent output already being active. These two
      // deterministic timing hypotheses are not actuator acknowledgement.
      if (accepted_output && moving(*accepted_output)) {
        model.active_command = accepted_output; model.pending.clear();
        const auto active = check(model);
        if (active.status != SweepStatus::kClear) {return active;}
      }
      return worst;
    };
  out.candidate = evaluate(target_command, false);
  if (inputs_ready) {out.reason = out.candidate.reason;}
  bool clear = out.candidate.status == SweepStatus::kClear;
  if (clear && risk.state == msg::PathRiskState::COLLISION_AHEAD &&
    out.stopping_distance >= out.risk_distance) {
    clear = false; out.reason = "path_braking_boundary";
  }
  if (!clear) {
    inhibited_ = true; clear_cycles_ = 0;
    obstacle_stop_ = obstacle_stop_ || out.candidate.status == SweepStatus::kCollision ||
      out.reason == "path_braking_boundary";
  } else if (inhibited_ && simulation_ && !obstacle_stop_) {
    // Input recovery is not an obstacle stop. Current geometry, motion and
    // red braking distance have all passed above; no forced full stop is needed.
    inhibited_ = false; clear_cycles_ = 0; out.action = "inputs_recovered";
  } else if (inhibited_) {
    if (!out.stopped) {clear = false; out.reason = "braking_not_stopped";}
    else if (risk.state == msg::PathRiskState::COLLISION_AHEAD) {
      clear = false; out.reason = "path_collision_persists";
    } else if (!repeat) {
      // Repeated zero output neither adds nor erases a valid restart confirmation.
      if (!moving(target_command)) {clear = false; out.reason = "resume_motion_missing";}
      else {
        out.resume = evaluate(target_command, true);
        clear = out.resume.status == SweepStatus::kClear;
        if (!clear) {out.reason = out.resume.reason;}
      }
    }
    if (!clear) {clear_cycles_ = 0;}
    else if (!repeat && pose_stamp_ns > confirmed_pose_ && risk.evidence_stamp_ns > confirmed_risk_) {
      ++clear_cycles_; confirmed_pose_ = pose_stamp_ns; confirmed_risk_ = risk.evidence_stamp_ns;
    }
    if (clear_cycles_ >= 3) {inhibited_ = obstacle_stop_ = false; out.action = "released";}
    else if (clear) {out.reason = "resume_confirming";}
  }
  out.command = target_command;
  if (inhibited_) {
    const auto brake = limiter.brake(measured, dt, accepted_output);
    out.command = brake.command;
    out.action = brake.mode == HolonomicSafetyMode::kEmergency ? "emergency_zero" :
      brake.mode == HolonomicSafetyMode::kHold ? "hold" : "braking";
    out.output = evaluate(out.command, false);
    if (out.output.status == SweepStatus::kCollision) {
      obstacle_stop_ = true;
      out.command = {}; out.action = "emergency_zero"; out.reason = "braking_collision";
      out.output = evaluate(out.command, false);
    }
  } else {
    out.output = out.candidate;
    if (repeat) {out.action = "repeat_output"; out.reason = "simulation_clock_repeat";}
  }
  commit(out.command, now_ns, limiter);
  out.inhibited = inhibited_; out.clear_cycles = clear_cycles_;
  out.mode = inhibited_ ? (out.stopped ? "stopped" : "braking") : "tracking";
  return out;
}

void ExecutionSafetyGate::report(const rclcpp::Logger & logger, const SafetyGateDecision & d,
  const HolonomicBodyCommand & candidate, uint64_t revision, int64_t now_ns,
  int64_t pose_stamp_ns, int64_t map_stamp_ns, double elapsed_ms, const M::History & history)
{
  const auto wall = Wall::now();
  const auto key = std::to_string(revision) + ':' + pathRiskName(d.path_risk) + ':' + d.action +
    ':' + d.mode + ':' + d.reason + ':' + std::to_string(d.stopped) + ':' +
    std::to_string(d.clear_cycles) + ':' + statusName(d.candidate.status) + ':' + d.candidate.reason +
    ':' + statusName(d.output.status) + ':' + d.output.reason + ':' +
    statusName(d.resume.status) + ':' + d.resume.reason;
  const auto motion_key = std::to_string(revision) + ':' + std::to_string(history.revision) +
    ':' + history.state + ':' + d.motion_reason + ':' + std::to_string(history.epoch) +
    ':' + std::to_string(history.dropped);
  const bool changed = key != last_report_;
  const bool motion_changed = motion_key != last_motion_report_;
  const bool debug = logger.get_effective_level() <= rclcpp::Logger::Level::Debug;
  if (!changed && !motion_changed &&
    (!debug || wall - reported_at_ < std::chrono::seconds(1))) {return;}
  if (motion_changed) {
    RCLCPP_INFO(logger, "PID_MOTION summary revision=%lu history_revision=%lu "
      "epoch=%lu history_state=%s alignment=%s last_reset=%s dropped=%lu",
      revision, history.revision, history.epoch, history.state,
      d.motion_reason.c_str(), history.reset_reason, history.dropped);
  }
  if (changed) {
    std::ostringstream event;
    event << std::fixed << std::setprecision(3) << "PID_GATE summary revision=" << revision <<
      " path_risk=" << pathRiskName(d.path_risk) <<
      " action=" << d.action << " reason=" << d.reason << " mode=" << d.mode << " stopped=" << d.stopped;
    if (d.inhibited || std::strcmp(d.action, "released") == 0) {
      event << " candidate=(" << candidate.vx << ',' << candidate.vy << ',' << candidate.omega <<
        ") output=(" << d.command.vx << ',' << d.command.vy << ',' << d.command.omega << ')';
    }
    if (d.path_risk == msg::PathRiskState::COLLISION_AHEAD) {
      event << " risk_remaining=" << d.risk_distance << " stop_travel=" << d.stopping_distance;
    }
    if (d.reason == "pose_invalid_or_stale" || d.motion_reason == "pose_time_invalid") {
      event << " pose_age=" << (now_ns - pose_stamp_ns) * 1e-9;
    }
    if (d.reason == "cached_map_stale" || d.reason == "map_not_current") {
      event << " map_age=" << (map_stamp_ns ? (now_ns - map_stamp_ns) * 1e-9 : -1.);
    }
    if (d.reason == "clock_invalid_or_regressed" || d.reason == "control_time_or_revision_invalid") {
      event << " now=" << now_ns * 1e-9;
    }
    // Include only checks that actually ran; empty hit geometry is DEBUG-only.
    for (const auto & check : {std::pair<const char *, const CommandSweepResult &>{"candidate", d.candidate},
      {"output", d.output}, {"resume", d.resume}})
    {
      const auto & result = check.second;
      if ((std::strcmp(check.first, "resume") == 0 && std::strcmp(result.reason, "not_required") != 0) ||
        result.status == SweepStatus::kCollision ||
        (d.sweep_requested && result.status == SweepStatus::kUnavailable &&
        std::strcmp(result.reason, "not_required") != 0))
      {
        event << ' ' << check.first << "_state=" << statusName(result.status) <<
          ' ' << check.first << "_reason=" << result.reason;
        if (result.status == SweepStatus::kCollision) {
          event << ' ' << check.first << "_phase=" << static_cast<int>(result.phase) <<
            ' ' << check.first << "_hit_t=" << result.time << ' ' << check.first <<
            "_hit_pose=(" << result.pose.x << ',' << result.pose.y << ',' << result.pose.yaw << ')';
          if (result.collision_cell) {
            event << ' ' << check.first << "_cell=(" << result.collision_cell->x << ',' <<
              result.collision_cell->y << ')';
          }
        }
      }
    }
    if (d.clear_cycles || std::strcmp(d.resume.reason, "not_required") != 0) {
      event << " clear_cycles=" << d.clear_cycles << " resume_t=" << d.resume.time <<
        " resume_distance=" << d.resume.distance;
    }
    if (d.sweep_requested && (d.candidate.status == SweepStatus::kUnavailable ||
      d.output.status == SweepStatus::kUnavailable)) {event << " total_ms=" << elapsed_ms;}
    RCLCPP_INFO(logger, "%s", event.str().c_str());
  }
  last_report_ = key; last_motion_report_ = motion_key; reported_at_ = wall;
  if (!debug) {return;}
  // Full DEBUG records remain paired by revision and integer ROS cycle identity.
  {
    const auto & series = history.odometry;
    const M::Sample missing;
    const auto & latest = series.size ? series.at(series.size - 1) : missing;
    const M::Sample * anchor = nullptr;
    for (size_t i = 0; i < series.size; ++i) {
      if (d.aligned_stamp_ns && series.at(i).stamp_ns == d.aligned_stamp_ns) {anchor = &series.at(i);}
    }
    const auto & sample = anchor ? *anchor : missing;
    RCLCPP_DEBUG(logger, "PID_MOTION stage=2-A revision=%lu history_revision=%lu epoch=%lu last_reset=%s "
      "history_state=%s alignment=%s samples=%zu dropped=%lu overwritten=%lu "
      "now=%.3f cycle_ns=%ld pose_stamp=%.3f latest_stamp=%.3f latest_payload=%s anchor_found=%d "
      "anchor_count=%lu anchor_age=%.3f anchor_skew_ms=%.3f measured=(%.3f,%.3f,%.3f) "
      "candidate=(%.3f,%.3f,%.3f) effective_delay=%.3f prediction_requested=%d",
      revision, history.revision, history.epoch, history.reset_reason, history.state,
      d.motion_reason.c_str(), series.size, history.dropped, series.overwritten,
      now_ns * 1e-9, now_ns, pose_stamp_ns * 1e-9, latest.stamp_ns * 1e-9, latest.payload, anchor != nullptr,
      sample.count, anchor ? std::chrono::duration<double>(wall - sample.received).count() : -1.,
      anchor ? (sample.stamp_ns - pose_stamp_ns) * 1e-6 : 0.,
      sample.vx, sample.vy, sample.omega, candidate.vx, candidate.vy, candidate.omega,
      std::max(0., (now_ns - pose_stamp_ns) * 1e-9) + nominal_.delay, d.sweep_requested);
  }
  const double unknown = std::numeric_limits<double>::quiet_NaN();
  RCLCPP_DEBUG(logger, "PID_GATE stage=2-C evaluation=2-B basis=nominal revision=%lu now=%.3f cycle_ns=%ld pose_age=%.3f map_age=%.3f "
    "path_risk=%s sweep_requested=%d evaluations=%u candidate=(%.3f,%.3f,%.3f) candidate_state=%s candidate_reason=%s phase=%d hit_t=%.3f "
    "resume_state=%s resume_reason=%s action=%s reason=%s clear_cycles=%u "
    "resume_policy=stopped_current_candidate resume_hold=%.3f resume_t=%.3f resume_distance=%.3f "
    "candidate_hit_pose=(%.3f,%.3f,%.3f) candidate_cell=(%.3f,%.3f) "
    "resume_hit_pose=(%.3f,%.3f,%.3f) resume_cell=(%.3f,%.3f) "
    "output=(%.3f,%.3f,%.3f) output_state=%s output_reason=%s total_ms=%.3f "
    "mode=%s stopped=%d risk_remaining=%.3f stop_travel=%.3f issued=%zu "
    "physical_response=unverified queued_commands=nominal_history steering=unmodeled",
    revision, now_ns * 1e-9, now_ns, (now_ns - pose_stamp_ns) * 1e-9,
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
    d.command.vx, d.command.vy, d.command.omega, statusName(d.output.status), d.output.reason, elapsed_ms, d.mode, d.stopped, d.risk_distance, d.stopping_distance, issued_.size());
}
}  // namespace pid_tracking_controller
