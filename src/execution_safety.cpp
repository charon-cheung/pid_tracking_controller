#include "execution_safety.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rbs_mppi_controller/footprint_collision.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pid_tracking_controller
{
namespace
{
bool finite(const HolonomicBodyCommand & v)
{
  return std::isfinite(v.vx) && std::isfinite(v.vy) && std::isfinite(v.omega);
}

CommandSweepResult evaluate(const CommandSweepInput & input, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, const ExecutionSafetyOptions & options)
{
  CommandSweepResult result;
  result.pose = input.pose;
  const auto started = std::chrono::steady_clock::now();
  const auto expired = [&]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >=
             options.budget_seconds;
    };
  const auto & v = input.velocity;
  const auto & a = input.deceleration;
  const double resolution = map.getResolution();
  if (!std::isfinite(input.pose.x) || !std::isfinite(input.pose.y) ||
    !std::isfinite(input.pose.yaw) || !finite(v) || !finite(a) ||
    a.vx <= 0 || a.vy <= 0 || a.omega <= 0 ||
    !std::isfinite(input.hold_time) || input.hold_time < 0 ||
    footprint.size() < 3 || footprint.size() > 64 || !map.getCharMap() ||
    !map.getSizeInCellsX() || !map.getSizeInCellsY() ||
    !std::isfinite(resolution) || resolution <= 0 ||
    !std::isfinite(map.getOriginX()) || !std::isfinite(map.getOriginY()))
  {return result;}
  const double end_x = map.getOriginX() + map.getSizeInCellsX() * resolution;
  const double end_y = map.getOriginY() + map.getSizeInCellsY() * resolution;
  const std::array<double, 3> stop_times{
    std::abs(v.vx) / a.vx, std::abs(v.vy) / a.vy, std::abs(v.omega) / a.omega};
  const double brake_time = *std::max_element(stop_times.begin(), stop_times.end());
  if (!std::isfinite(end_x) || !std::isfinite(end_y) ||
    !std::isfinite(brake_time + input.hold_time) ||
    (brake_time == 0 && (v.vx != 0 || v.vy != 0 || v.omega != 0)))
  {return result;}
  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = xmax, area = 0.;
  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const auto & p = footprint[i];
    const auto & q = footprint[(i + 1) % footprint.size()];
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return result;}
    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    area += p.x * q.y - q.x * p.y;
  }
  const double radius = std::hypot(std::max(std::abs(xmin), std::abs(xmax)),
    std::max(std::abs(ymin), std::abs(ymax)));
  if (!std::isfinite(radius) || !std::isfinite(area) || area == 0 ||
    xmin >= xmax || ymin >= ymax)
  {return result;}

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  nav2_costmap_2d::Footprint body(4), oriented;
  std::size_t samples = 0, cell_checks = 0;
  const auto inspect = [&](double padding) {
      result.status = SweepStatus::kUnavailable;
      result.reason = "evaluation_budget";
      if (samples >= options.max_samples || expired()) {return false;}
      ++samples;
      body[0].x = body[3].x = xmin - padding;
      body[1].x = body[2].x = xmax + padding;
      body[0].y = body[1].y = ymin - padding;
      body[2].y = body[3].y = ymax + padding;
      const double c = std::cos(result.pose.yaw), s = std::sin(result.pose.yaw);
      double wxmin = std::numeric_limits<double>::infinity(), wxmax = -wxmin;
      double wymin = wxmin, wymax = wxmax;
      for (const auto & p : body) {
        const double x = result.pose.x + c * p.x - s * p.y;
        const double y = result.pose.y + s * p.x + c * p.y;
        if (!std::isfinite(x) || !std::isfinite(y) || x < map.getOriginX() ||
          y < map.getOriginY() || x >= end_x || y >= end_y)
        {result.reason = "outside_map"; return false;}
        wxmin = std::min(wxmin, x); wxmax = std::max(wxmax, x);
        wymin = std::min(wymin, y); wymax = std::max(wymax, y);
      }
      // Bound raster scan plus boundary traversal BEFORE entering the shared
      // helper, including its extra touching-cell border. No unbounded map scan.
      const double cols = std::ceil((wxmax - wxmin) / resolution) + 4;
      const double rows = std::ceil((wymax - wymin) / resolution) + 4;
      const double work = cols * rows + 4 * (cols + rows);
      if (!std::isfinite(work) || work > options.max_cell_checks - cell_checks) {return false;}
      cell_checks += static_cast<std::size_t>(work);
      std::optional<std::array<unsigned int, 2>> cell;
      const double cost = rbs_mppi_controller::fullFootprintCostAtPose(checker,
        result.pose.x, result.pose.y, result.pose.yaw, body, oriented, false, &cell, true);
      if (expired()) {return false;}
      if (!std::isfinite(cost) || cost < 0) {result.reason = "invalid_geometry"; return false;}
      if (cost >= nav2_costmap_2d::NO_INFORMATION) {
        result.reason = "unknown_space"; return false;
      }
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        result.status = SweepStatus::kCollision;
        result.reason = "body_sweep_occupied";
        if (cell) {
          geometry_msgs::msg::Point point;
          map.mapToWorld((*cell)[0], (*cell)[1], point.x, point.y);
          result.collision_cell = point;
        }
        return false;
      }
      return true;
    };
  if (!inspect(0.)) {return result;}
  double position_error = 0.;
  const auto velocity_at = [&](double t) {
      const auto axis = [t](double initial, double rate) {
          return std::copysign(std::max(0., std::abs(initial) - rate * t), initial);
        };
      return HolonomicBodyCommand{axis(v.vx, a.vx), axis(v.vy, a.vy), axis(v.omega, a.omega)};
    };
  const auto advance = [&](double duration, bool braking) {
      result.phase = braking ? SweepPhase::kBraking : SweepPhase::kHold;
      double t = 0.;
      while (t < duration) {
        const auto velocity = braking ? velocity_at(t) : v;
        const double speed = std::hypot(velocity.vx, velocity.vy);
        const double point_speed = speed + radius * std::abs(velocity.omega);
        double h = duration - t;
        if (point_speed > 0) {h = std::min(h, options.step / point_speed);}
        // Split where an axis reaches zero; each remaining interval has linear
        // body velocities, with no reversal or angular-acceleration discontinuity.
        if (braking) {
          for (const double stop : stop_times) {
            if (stop > t) {h = std::min(h, stop - t);}
          }
        }
        if (!std::isfinite(point_speed) || !std::isfinite(h) || h <= 0 || t + h == t) {
          result.status = SweepStatus::kUnavailable;
          result.reason = "integration_limit"; return false;
        }
        // From this interval's start, every point moves at most
        // integral(|v| + radius*|omega|) <= h*point_speed. An expanded body AABB
        // therefore covers ALL intermediate poses, not merely sampled endpoints.
        if (!inspect(position_error + h * point_speed)) {return false;}
        auto middle = velocity;
        double middle_yaw = result.pose.yaw + velocity.omega * h / 2;
        double yaw_step = velocity.omega * h;
        double scale = h;
        if (braking) {
          middle = velocity_at(t + h / 2);
          const auto end = velocity_at(t + h);
          middle_yaw = result.pose.yaw + (velocity.omega + middle.omega) * h / 4;
          yaw_step = (velocity.omega + end.omega) * h / 2;
          // Midpoint translation error <= L*h^2/4 for
          // g(t)=R(yaw(t))*v(t), |g'| <= |a_xy| + |v|*|omega|.
          const double acceleration = std::hypot(velocity.vx != 0 ? a.vx : 0.,
            velocity.vy != 0 ? a.vy : 0.);
          position_error += h * h / 4 * (acceleration + speed * std::abs(velocity.omega));
        } else if (yaw_step != 0) {
          // Exact constant-twist SE(2) integration (including lateral velocity).
          scale *= std::sin(yaw_step / 2) / (yaw_step / 2);
        }
        const double c = std::cos(middle_yaw), s = std::sin(middle_yaw);
        result.pose.x += scale * (c * middle.vx - s * middle.vy);
        result.pose.y += scale * (s * middle.vx + c * middle.vy);
        result.pose.yaw += yaw_step;
        position_error += 32 * std::numeric_limits<double>::epsilon() *
          (1 + std::abs(result.pose.x) + std::abs(result.pose.y) + speed * h +
          radius * (std::abs(result.pose.yaw) + std::abs(yaw_step)));
        t += h;
        result.time = (braking ? input.hold_time : 0.) + t;
        result.distance += std::hypot(middle.vx, middle.vy) * h;
        if (!std::isfinite(position_error) || !std::isfinite(result.distance) ||
          !std::isfinite(result.pose.yaw))
        {result.status = SweepStatus::kUnavailable; result.reason = "integration_limit"; return false;}
      }
      return true;
    };
  if (!advance(input.hold_time, false) || !advance(brake_time, true) ||
    !inspect(position_error))
  {return result;}
  result.status = SweepStatus::kClear;
  result.reason = "checked_to_stop";
  return result;
}
}  // namespace

ExecutionSafety::ExecutionSafety(const ExecutionSafetyOptions & options)
: options_(options)
{
  if (!std::isfinite(options.step) || options.step <= 0 || !options.max_samples ||
    !options.max_cell_checks || !std::isfinite(options.budget_seconds) || options.budget_seconds <= 0)
  {throw std::invalid_argument("invalid execution safety sampling/budget options");}
}

CommandSweepResult ExecutionSafety::checkCommand(const CommandSweepInput & input,
  nav2_costmap_2d::Costmap2D & map, const nav2_costmap_2d::Footprint & footprint)
{
  auto result = evaluate(input, map, footprint, options_);
  if (!last_status_ || *last_status_ != result.status || last_reason_ != result.reason) {
    const char * states[] = {"clear", "collision", "unavailable"};
    const char * phases[] = {"initial", "hold", "braking"};
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger("PidTrackingController"),
      "PID_SAFETY sweep state=%s reason=%s phase=%s velocity=(%.3f,%.3f,%.3f) "
      "hold=%.3fs decel=(%.3f,%.3f,%.3f) checked_time=%.3fs distance=%.3fm "
      "pose=(%.3f,%.3f,%.3f) cell=(%.3f,%.3f) action=geometry_only physical_stop=unverified",
      states[static_cast<unsigned>(result.status)], result.reason,
      phases[static_cast<unsigned>(result.phase)], input.velocity.vx, input.velocity.vy,
      input.velocity.omega, input.hold_time, input.deceleration.vx, input.deceleration.vy,
      input.deceleration.omega, result.time, result.distance, result.pose.x, result.pose.y,
      result.pose.yaw, result.collision_cell ? result.collision_cell->x : unknown,
      result.collision_cell ? result.collision_cell->y : unknown);
    last_status_ = result.status;
    last_reason_ = result.reason;
  }
  return result;
}
}  // namespace pid_tracking_controller
