#include "motion_transition_observer.hpp"

#include <cmath>
#include <cstring>
#include "rclcpp/logging.hpp"

namespace pid_tracking_controller
{
namespace
{
using M = MotionObservation;
constexpr const char * stages[] = {"candidate", "published", "smoothed", "odometry"};
const char * statusName(SweepStatus status)
{
  return status == SweepStatus::kClear ? "clear" :
         status == SweepStatus::kCollision ? "collision" : "unavailable";
}
const char * phaseName(SweepPhase phase)
{
  switch (phase) {
    case SweepPhase::kHold: return "hold";
    case SweepPhase::kBraking: return "braking";
    case SweepPhase::kDelay: return "delay";
    case SweepPhase::kTransition: return "transition";
    default: return "initial";
  }
}
bool sameVelocity(const M::Sample & a, const M::Sample & b)
{return a.vx == b.vx && a.vy == b.vy && a.omega == b.omega;}
// Freshness is an evidence gate only; it never limits commanded velocity.
const char * sampleReason(const M::Sample & s, size_t stage, int64_t ros_now, M::Wall::time_point now)
{
  if (std::strcmp(s.payload, "valid") != 0) {return s.payload;}
  if (std::strcmp(s.frame, "base") != 0) {return "frame_unverified";}
  if (now < s.received) {return "receipt_future";}
  if (now - s.received > std::chrono::milliseconds(600)) {return "receipt_stale";}
  if (s.stamp_ns > ros_now) {return "header_future";}
  if (stage != M::Smoothed && ros_now - s.stamp_ns > 600000000LL) {return "header_stale";}
  return "ready";
}
}  // namespace

TransitionObservation MotionTransitionObserver::evaluate(SafetyInputResult & geometry,
  const M::History & history, int64_t pose_stamp_ns, int64_t ros_now,
  Wall::time_point cycle_started, const TransitionExecutionEvidence * execution,
  const NominalTransitionAssumptions * nominal)
{
  const auto started = Wall::now();
  TransitionObservation out;
  out.pose_stamp_ns = pose_stamp_ns; out.execution_present = execution != nullptr;
  out.queue_complete = execution && execution->queue_complete;
  out.geometry_reason = geometry.snapshot && geometry.snapshot->map ? "ready" :
    geometry.reason.empty() || geometry.reason == "ready" ? "geometry_missing" : geometry.reason;
  if (geometry.snapshot) {
    const auto & stamp = geometry.snapshot->header.stamp;
    out.map_stamp_ns = int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
  }
  out.motion_reason = history.state;
  const M::Sample * candidate = nullptr, * odometry = nullptr;
  if (out.motion_reason == "ready") {
    if (history.last_drop != Wall::time_point{} &&
      started - history.last_drop < std::chrono::milliseconds(600))
    {out.motion_reason = "history_gap";}
    for (size_t i = 0; i < history.stages.size() && out.motion_reason == "ready"; ++i) {
      const auto & series = history.stages[i];
      size_t n = series.size;
      const char * reason = n ? sampleReason(series.at(n - 1), i, ros_now, started) : "missing";
      // Sensor delivery may precede /clock. Defer future odometry, then require
      // the exact fresh pose-time anchor below. Never defer issued commands or
      // hide malformed/faulted sensor samples, and never rewrite timestamps.
      while (i == M::Odometry && n && std::strcmp(reason, "header_future") == 0) {
        ++out.deferred_odometry;
        --n;
        if (!n) {break;}
        reason = sampleReason(series.at(n - 1), i, ros_now, started);
      }
      if (std::strcmp(reason, "ready") != 0) {
        out.motion_reason = std::string(stages[i]) + ":" + reason;
      }
    }
  }
  if (out.motion_reason == "ready") {
    const auto & candidates = history.stages[M::Candidate];
    const auto & odom = history.stages[M::Odometry];
    for (size_t i = 0; i < candidates.size; ++i) {
      const auto & s = candidates.at(i);
      if (s.stamp_ns <= pose_stamp_ns) {candidate = &s;}
    }
    for (size_t i = 0; i < odom.size; ++i) {
      const auto & s = odom.at(i);
      if (s.stamp_ns == pose_stamp_ns) {odometry = &s;}
    }
    if (pose_stamp_ns <= 0 || ros_now < pose_stamp_ns || ros_now - pose_stamp_ns > 300000000LL) {
      out.motion_reason = "pose_time_invalid";
    } else if (!candidate) {out.motion_reason = "candidate_anchor_missing";
    } else if (!odometry) {out.motion_reason = "odometry_anchor_missing";
    } else if (!history.revision || candidate->revision != history.revision) {
      out.motion_reason = "candidate_revision_unknown";
    } else {
      // Never use an invalid/stale older sample merely because the latest one recovered.
      for (const auto & entry : {std::pair<size_t, const M::Sample *>{M::Candidate, candidate},
          {M::Odometry, odometry}})
      {
        const auto * reason = sampleReason(*entry.second, entry.first, ros_now, started);
        if (std::strcmp(reason, "ready") != 0) {
          out.motion_reason = std::string(stages[entry.first]) + ":" + reason;
        }
        const auto & series = history.stages[entry.first];
        for (size_t j = 0; j < series.size; ++j) {
          const auto & s = series.at(j);
          if (s.stamp_ns == entry.second->stamp_ns && !sameVelocity(s, *entry.second)) {
            out.motion_reason = "anchor_ambiguous";
          }
          if (s.count >= entry.second->count && std::strcmp(s.payload, "valid") != 0) {
            out.motion_reason = "history_invalid";
          }
        }
      }
    }
  }
  if (candidate) {
    out.candidate_count = candidate->count; out.candidate_stamp_ns = candidate->stamp_ns;
  }
  const auto evaluate = [&]() -> CommandSweepResult {
      CommandSweepResult result;
      if (out.geometry_reason != "ready") {result.reason = "geometry_unavailable"; return result;}
      if (out.motion_reason != "ready") {result.reason = "motion_unavailable"; return result;}
      if (!execution && !nominal) {result.reason = "execution_evidence_missing"; return result;}
      if (execution) {
        if (!execution->queue_complete) {result.reason = "execution_queue_incomplete"; return result;}
        if (execution->epoch != history.epoch || execution->revision != history.revision ||
          execution->candidate_count != candidate->count || execution->pose_stamp_ns != pose_stamp_ns)
        {result.reason = "execution_evidence_mismatch"; return result;}
        if (execution->model.pending.size() > 16) {result.reason = "motion_history_limit"; return result;}
      }
      auto limits = options_;
      limits.budget_seconds -= std::chrono::duration<double>(Wall::now() - cycle_started).count();
      if (!std::isfinite(limits.budget_seconds) || limits.budget_seconds <= 0 || cycle_started > started) {
        result.reason = "observer_budget_exceeded"; return result;
      }
      auto model = execution ? execution->model : MotionTransitionInput{};
      model.measured = {odometry->vx, odometry->vy, odometry->omega};
      model.candidate = {candidate->vx, candidate->vy, candidate->omega};
      if (!execution) {
        // Explicit counterfactual: no claim that these are active/queued commands.
        model.active_command = model.measured; model.active_valid_for = nominal->delay;
        model.candidate_execution = {nominal->delay, nominal->delay};
        model.candidate_duration = nominal->duration;
        model.acceleration = nominal->acceleration; model.deceleration = nominal->deceleration;
        model.braking_deceleration = nominal->braking_deceleration;
        out.nominal_sweep = ExecutionSafety(limits).checkTransition(geometry.snapshot->pose, model,
          *geometry.snapshot->map, geometry.snapshot->footprint);
        result.reason = "execution_evidence_missing";
        return result;
      }
      out.evaluated = true;
      return ExecutionSafety(limits).checkTransition(geometry.snapshot->pose, model,
        *geometry.snapshot->map, geometry.snapshot->footprint);
    };
  out.sweep = evaluate();
  const auto finished = Wall::now();
  out.evaluate_ms = std::chrono::duration<double, std::milli>(finished - started).count();
  out.total_ms = std::chrono::duration<double, std::milli>(finished - cycle_started).count();
  auto & prediction = out.nominal_sweep ? *out.nominal_sweep : out.sweep;
  if (prediction.status != SweepStatus::kUnavailable && out.total_ms > options_.budget_seconds * 1000.) {
    prediction.status = SweepStatus::kUnavailable; prediction.reason = "observer_budget_exceeded";
  }
  return out;
}

void MotionTransitionObserver::reset()
{
  last_key_.clear(); reported_at_ = {}; reports_ = clear_ = collision_ = unavailable_ = evaluated_ = 0;
  nominal_evaluated_ = nominal_clear_ = nominal_collision_ = nominal_unavailable_ = 0;
  recent_candidate_cycles_ = motion_ready_cycles_ = gap_cycles_ = future_odom_cycles_ = execution_missing_cycles_ = 0;
}

void MotionTransitionObserver::report(const rclcpp::Logger & logger, const TransitionObservation & r,
  const M::History & h, int64_t ros_now, double capture_ms, double history_ms)
{
  const auto now = Wall::now(); const auto & s = r.sweep;
  if (s.status == SweepStatus::kClear) {++clear_;}
  else if (s.status == SweepStatus::kCollision) {++collision_;} else {++unavailable_;}
  if (r.evaluated || r.nominal_sweep) {++evaluated_;}
  const auto & prediction = r.nominal_sweep ? *r.nominal_sweep : s;
  if (r.nominal_sweep) {
    ++nominal_evaluated_;
    nominal_clear_ += prediction.status == SweepStatus::kClear;
    nominal_collision_ += prediction.status == SweepStatus::kCollision;
    nominal_unavailable_ += prediction.status == SweepStatus::kUnavailable;
  }
  const char * basis = r.nominal_sweep ? "nominal" : r.evaluated ? "execution_evidence" : "none";
  const auto & candidates = h.stages[M::Candidate];
  if (candidates.size && now >= candidates.at(candidates.size - 1).received &&
    now - candidates.at(candidates.size - 1).received <= std::chrono::milliseconds(600))
  {
    ++recent_candidate_cycles_;
    motion_ready_cycles_ += r.motion_reason == "ready";
    gap_cycles_ += r.motion_reason == "history_gap";
    future_odom_cycles_ += r.deferred_odometry != 0;
    execution_missing_cycles_ += std::strcmp(s.reason, "execution_evidence_missing") == 0;
  }
  const std::string key = std::string(statusName(s.status)) + ":" + s.reason + ":" +
    r.geometry_reason + ":" + r.motion_reason + ":" + basis + ":" + statusName(prediction.status) + ":" +
    prediction.reason + ":" + phaseName(prediction.phase) + ":" + std::to_string(h.epoch) + ":" + std::to_string(h.revision);
  if (key == last_key_ && now - reported_at_ < std::chrono::seconds(5)) {return;}
  const bool event = key != last_key_; last_key_ = key; reported_at_ = now; ++reports_;
  RCLCPP_INFO(logger, "PID_TRANSITION status mode=observe_only state=%s reason=%s "
    "geometry_reason=%s motion_reason=%s execution_evidence=%s queue_complete=%d "
    "model_evaluated=%d model_basis=%s model_state=%s model_reason=%s "
    "action=none control_authorized=0 physical_stop=unverified report=%lu event=%d deferred_odom=%zu "
    "epoch=%lu plan_revision=%lu candidate_seq=%lu pose_stamp=%.9f candidate_stamp=%.9f map_stamp=%.9f "
    "phase=%s checked_time=%.6f distance=%.6f pose=(%.4f,%.4f,%.4f) collision_cell=%d cell=(%.4f,%.4f)",
    statusName(s.status), s.reason, r.geometry_reason.c_str(), r.motion_reason.c_str(),
    r.execution_present ? "provided" : "missing", r.queue_complete, bool(r.evaluated || r.nominal_sweep),
    basis, statusName(prediction.status), prediction.reason, reports_, event, r.deferred_odometry, h.epoch, h.revision,
    r.candidate_count, r.pose_stamp_ns * 1e-9, r.candidate_stamp_ns * 1e-9, r.map_stamp_ns * 1e-9,
    phaseName(prediction.phase), prediction.time, prediction.distance,
    prediction.pose.x, prediction.pose.y, prediction.pose.yaw, bool(prediction.collision_cell),
    prediction.collision_cell ? prediction.collision_cell->x : 0.,
    prediction.collision_cell ? prediction.collision_cell->y : 0.);
  for (size_t i = 0; i < h.stages.size(); ++i) {
    const auto & series = h.stages[i]; const auto latest = series.size ? series.at(series.size - 1) : M::Sample{};
    RCLCPP_INFO(logger, "PID_TRANSITION history report=%lu stage=%s size=%zu overwritten=%lu "
      "attempted=%lu dropped=%lu oldest_stamp=%.9f latest_stamp=%.9f anchor_delta=%.6f receipt_age=%.6f payload=%s frame=%s "
      "seq=%lu repeated=%lu regressed=%lu invalid=%lu vx=%.6f vy=%.6f omega=%.6f",
      reports_, stages[i], series.size, series.overwritten, h.attempts[i], h.drops[i],
      series.size ? series.at(0).stamp_ns * 1e-9 : 0.,
      latest.stamp_ns * 1e-9, series.size ? (latest.stamp_ns - r.pose_stamp_ns) * 1e-9 : 0.,
      series.size ? std::chrono::duration<double>(now - latest.received).count() : -1., latest.payload,
      latest.frame, latest.count, latest.repeated, latest.regressed, latest.invalid, latest.vx, latest.vy, latest.omega);
  }
  RCLCPP_INFO(logger, "PID_TRANSITION timing report=%lu ros_now=%.9f capture_ms=%.3f history_ms=%.3f "
    "evaluate_ms=%.3f total_ms=%.3f budget_ms=%.3f dropped=%lu evaluated=%lu clear=%lu collision=%lu unavailable=%lu "
    "recent_candidate_cycles=%lu motion_ready_cycles=%lu gap_cycles=%lu future_odom_cycles=%lu execution_missing_cycles=%lu "
    "nominal_evaluated=%lu nominal_clear=%lu nominal_collision=%lu nominal_unavailable=%lu",
    reports_, ros_now * 1e-9, capture_ms, history_ms, r.evaluate_ms, r.total_ms, options_.budget_seconds * 1000.,
    h.dropped, evaluated_, clear_, collision_, unavailable_, recent_candidate_cycles_, motion_ready_cycles_,
    gap_cycles_, future_odom_cycles_, execution_missing_cycles_,
    nominal_evaluated_, nominal_clear_, nominal_collision_, nominal_unavailable_);
}

void MotionTransitionObserver::stopped(const rclcpp::Logger & logger) const
{
  RCLCPP_INFO(logger, "PID_TRANSITION stopped mode=observe_only action=none evaluated=%lu "
    "clear=%lu collision=%lu unavailable=%lu nominal_evaluated=%lu nominal_clear=%lu "
    "nominal_collision=%lu nominal_unavailable=%lu", evaluated_, clear_, collision_, unavailable_,
    nominal_evaluated_, nominal_clear_, nominal_collision_, nominal_unavailable_);
}
}  // namespace pid_tracking_controller
