#ifndef PID_TRACKING_CONTROLLER__PATH_RISK_TRIGGER_HPP_
#define PID_TRACKING_CONTROLLER__PATH_RISK_TRIGGER_HPP_

#include <chrono>
#include "pid_tracking_controller/msg/path_risk_state.hpp"
#include "pid_tracking_controller/msg/tracking_progress.hpp"

namespace pid_tracking_controller
{
struct PathRiskTrigger
{
  uint8_t state{msg::PathRiskState::UNAVAILABLE};
  const char * reason{"path_risk_missing"};
};

inline const char * pathRiskName(uint8_t state)
{
  switch (state) {
    case msg::PathRiskState::CLEAR: return "CLEAR";
    case msg::PathRiskState::CLEARANCE_LOW: return "CLEARANCE_LOW";
    case msg::PathRiskState::COLLISION_AHEAD: return "COLLISION_AHEAD";
    default: return "UNAVAILABLE";
  }
}

// A marker's absence is not CLEAR. A bypass requires fresh evidence for this
// exact controller instance and path load, including the observer's source times.
inline PathRiskTrigger validatePathRisk(const msg::PathRiskState * risk,
  std::chrono::steady_clock::time_point received, const msg::TrackingProgress & current,
  int64_t now, std::chrono::steady_clock::time_point wall)
{
  using Risk = msg::PathRiskState;
  if (!risk) {return {};}
  const auto unavailable = [](const char * reason) {return PathRiskTrigger{Risk::UNAVAILABLE, reason};};
  if (!current.valid || current.controller_instance.empty() || !current.plan_revision ||
    !current.path_fingerprint || current.path_header.frame_id.empty() ||
    risk->controller_instance != current.controller_instance ||
    risk->plan_revision != current.plan_revision || risk->path_fingerprint != current.path_fingerprint ||
    risk->path_header != current.path_header)
  {return unavailable("path_risk_identity_mismatch");}
  const auto stamp = [](const builtin_interfaces::msg::Time & t) {
      return int64_t(t.sec) * 1000000000LL + t.nanosec;
    };
  const auto fresh = [&](const builtin_interfaces::msg::Time & t, int64_t age) {
      const auto ns = stamp(t);
      return t.nanosec < 1000000000u && ns > 0 && now >= ns && now - ns <= age;
    };
  if (now <= 0 || wall < received || wall - received > std::chrono::milliseconds(300) ||
    !fresh(current.header.stamp, 300000000LL) || !fresh(risk->header.stamp, 300000000LL) ||
    !fresh(risk->progress_stamp, 300000000LL) || !fresh(risk->map_stamp, 600000000LL) ||
    stamp(current.loaded_at) <= 0 || stamp(risk->progress_stamp) < stamp(current.loaded_at) ||
    stamp(risk->progress_stamp) > stamp(risk->header.stamp) || risk->header.frame_id.empty())
  {return unavailable("path_risk_time_invalid_or_stale");}
  if (risk->state != Risk::CLEAR && risk->state != Risk::CLEARANCE_LOW &&
    risk->state != Risk::COLLISION_AHEAD) {return unavailable("path_risk_unavailable");}
  return {risk->state, "ready"};
}
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__PATH_RISK_TRIGGER_HPP_
