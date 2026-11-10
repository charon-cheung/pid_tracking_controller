#ifndef PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_HPP_
#define PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include "holonomic_command_limiter.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace pid_tracking_controller
{
struct SweepPose
{
  double x, y, yaw;
};

struct CommandSweepInput
{
  SweepPose pose;
  HolonomicBodyCommand velocity;
  // Seconds at this body velocity before braking: include command/reaction delay.
  // Zero checks immediate braking. These are model inputs, not physical defaults.
  double hold_time;
  // Positive, guaranteed body-axis braking rates (m/s^2, m/s^2, rad/s^2).
  HolonomicBodyCommand deceleration;
};

struct ExecutionSafetyOptions
{
  double step{0.025};  // Maximum point displacement per interval, metres.
  std::size_t max_samples{512};
  std::size_t max_cell_checks{500000};
  double budget_seconds{0.01};
};

enum class SweepStatus {kClear, kCollision, kUnavailable};
enum class SweepPhase {kInitial, kHold, kBraking};

struct CommandSweepResult
{
  SweepStatus status{SweepStatus::kUnavailable};
  const char * reason{"invalid_input"};
  SweepPhase phase{SweepPhase::kInitial};
  // Start of the first blocked/unverified interval, or final stop pose if clear.
  // Conservative envelope contact, NOT an exact physical time of impact.
  SweepPose pose{0., 0., 0.};
  double time{0.0};
  double distance{0.0};  // Accumulated midpoint centre travel, metres.
  std::optional<geometry_msgs::msg::Point> collision_cell;
};

// Geometric prediction only: constant body velocity, then independent linear
// braking to zero without reversal. A body AABB encloses the entire footprint;
// padding covers continuous motion AND numerical integration error.
// Clear does not certify observation freshness, command reachability, physical
// stopping, or a transition from some other measured/issued velocity (1D/1E).
class ExecutionSafety
{
public:
  explicit ExecutionSafety(const ExecutionSafetyOptions & options = {});

  // pose/map share a frame; footprint is in the body frame. Caller supplies a
  // stable map for the whole call. Map is read only; non-const reference is
  // required by the existing Nav2/shared footprint-checker interface.
  CommandSweepResult checkCommand(const CommandSweepInput & input,
    nav2_costmap_2d::Costmap2D & map, const nav2_costmap_2d::Footprint & footprint);

private:
  ExecutionSafetyOptions options_;
  std::optional<SweepStatus> last_status_;
  std::string last_reason_;
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_HPP_
