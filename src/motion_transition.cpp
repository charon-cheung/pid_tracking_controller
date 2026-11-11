#include "motion_transition.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace pid_tracking_controller
{
namespace
{
using Axes = std::array<double, 3>;
Axes axes(const HolonomicBodyCommand & v) {return {v.vx, v.vy, v.omega};}
HolonomicBodyCommand command(const Axes & v) {return {v[0], v[1], v[2]};}
bool finite(const HolonomicBodyCommand & v)
{return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.omega);}
bool positive(const HolonomicBodyCommand & v)
{return finite(v) && v.vx > 0 && v.vy > 0 && v.omega > 0;}
bool known(const ExecutionWindow & w)
{return std::isfinite(w.earliest) && w.earliest >= 0 && w.earliest == w.latest;}

// Append continuous linear segments, splitting at each axis zero/target event.
bool approach(MotionTransition & out, HolonomicBodyCommand & velocity,
  const HolonomicBodyCommand & target, double duration, SweepPhase phase,
  const HolonomicBodyCommand & acceleration, const HolonomicBodyCommand & deceleration)
{
  const auto goal = axes(target), up = axes(acceleration), down = axes(deceleration);
  double elapsed = 0.;
  while (elapsed < duration) {
    if (out.segments.size() >= 128) {return false;}
    auto current = axes(velocity), end = current, boundary = goal;
    Axes times, rates;
    double h = duration - elapsed;
    bool moving = false;
    for (std::size_t i = 0; i < 3; ++i) {
      if ((current[i] > 0 && goal[i] < 0) || (current[i] < 0 && goal[i] > 0)) {
        boundary[i] = 0.;
      }
      rates[i] = std::abs(boundary[i]) > std::abs(current[i]) ? up[i] : down[i];
      if (boundary[i] == current[i]) {
        times[i] = std::numeric_limits<double>::infinity();
      } else {
        moving = true;
        times[i] = std::abs(boundary[i] - current[i]) / rates[i];
        if (!std::isfinite(times[i]) || times[i] <= 0) {return false;}
        h = std::min(h, times[i]);
      }
    }
    if (!std::isfinite(h) || h <= 0 || elapsed + h == elapsed) {return false;}
    for (std::size_t i = 0; i < 3; ++i) {
      if (h >= times[i]) {end[i] = boundary[i];} else if (boundary[i] != current[i]) {
        end[i] += std::copysign(rates[i] * h, boundary[i] - current[i]);
      }
    }
    const auto next = command(end);
    if (!finite(next)) {return false;}
    out.segments.push_back({velocity, next, h,
      !moving && phase == SweepPhase::kTransition ? SweepPhase::kHold : phase});
    velocity = next;
    elapsed += h;
  }
  return true;
}

bool brake(MotionTransition & out, HolonomicBodyCommand & v,
  const HolonomicBodyCommand & deceleration)
{
  while (v.vx != 0 || v.vy != 0 || v.omega != 0) {
    const double duration = std::max({std::abs(v.vx) / deceleration.vx,
        std::abs(v.vy) / deceleration.vy, std::abs(v.omega) / deceleration.omega});
    if (!std::isfinite(duration) || duration <= 0 ||
      !approach(out, v, {}, duration, SweepPhase::kBraking, deceleration, deceleration))
    {return false;}
    // Splitting at another axis's stop can leave an ulp-sized residual. Model
    // its remaining braking rather than snapping to zero or rejecting normal
    // motion. approach's shared segment budget also bounds this completion loop.
  }
  return true;
}
}  // namespace

MotionTransition buildMotionTransition(const MotionTransitionInput & in)
{
  MotionTransition out;
  if (!finite(in.measured) || !finite(in.candidate) || !positive(in.acceleration) ||
    !positive(in.deceleration) || !positive(in.braking_deceleration) ||
    !std::isfinite(in.candidate_duration) || in.candidate_duration < 0)
  {return out;}
  out.reason = "execution_timing_unknown";
  if (!known(in.candidate_execution)) {return out;}
  const double delay = in.candidate_execution.earliest;
  if (!std::isfinite(delay + in.candidate_duration)) {return out;}
  out.reason = "motion_history_limit";
  if (in.pending.size() > 16) {return out;}
  out.reason = "active_command_unknown";
  if (delay > 0 && (!in.active_command || !finite(*in.active_command) ||
    !std::isfinite(in.active_valid_for) || in.active_valid_for < 0))
  {return out;}
  auto velocity = in.measured;
  auto active = in.active_command.value_or(HolonomicBodyCommand{});
  double expiry = delay > 0 ? in.active_valid_for : 0.;
  double now = 0., last_apply = -1.;
  const auto advance_to = [&](double until) {
      if (expiry > now && expiry < until) {
        if (!approach(out, velocity, active, expiry - now, SweepPhase::kDelay,
          in.acceleration, in.deceleration)) {return false;}
        now = expiry;
      }
      if (expiry <= now) {active = {};}
      if (!approach(out, velocity, active, until - now, SweepPhase::kDelay,
        in.acceleration, in.deceleration)) {return false;}
      now = until;
      return true;
    };
  for (const auto & pending : in.pending) {
    if (!pending.issued) {continue;}
    out.reason = "pending_command_unknown";
    if (!std::isfinite(pending.expires_after)) {return out;}
    if (pending.expires_after <= 0) {continue;}
    if (!known(pending.execution) || !finite(pending.velocity)) {return out;}
    const double apply = pending.execution.earliest;
    if (pending.expires_after <= apply) {continue;}
    out.reason = "pending_command_order";
    if (apply <= last_apply || apply >= delay) {return out;}
    out.reason = "motion_integration_limit";
    if (!advance_to(apply)) {return out;}
    active = pending.velocity;
    expiry = pending.expires_after;
    last_apply = apply;
  }
  out.reason = "motion_integration_limit";
  if (!advance_to(delay) ||
    !approach(out, velocity, in.candidate, in.candidate_duration, SweepPhase::kTransition,
    in.acceleration, in.deceleration) || !brake(out, velocity, in.braking_deceleration))
  {return out;}
  double duration = 0.;
  for (const auto & segment : out.segments) {duration += segment.duration;}
  if (!std::isfinite(duration)) {return out;}
  out.available = true;
  out.reason = "modeled_to_stop";
  return out;
}

MotionTransition buildCommandMotion(const HolonomicBodyCommand & initial,
  double hold_time, const HolonomicBodyCommand & deceleration)
{
  MotionTransition out;
  if (!finite(initial) || !positive(deceleration) ||
    !std::isfinite(hold_time) || hold_time < 0) {return out;}
  auto velocity = initial;
  out.reason = "motion_integration_limit";
  if (!approach(out, velocity, initial, hold_time, SweepPhase::kHold,
    deceleration, deceleration) || !brake(out, velocity, deceleration)) {return out;}
  double duration = 0.;
  for (const auto & segment : out.segments) {duration += segment.duration;}
  if (!std::isfinite(duration)) {return out;}
  out.available = true;
  out.reason = "modeled_to_stop";
  return out;
}
}  // namespace pid_tracking_controller
