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
CommandSweepResult evaluate(const SweepPose & pose, const MotionTransition & motion,
  nav2_costmap_2d::Costmap2D & map, const nav2_costmap_2d::Footprint & footprint,
  const ExecutionSafetyOptions & options, std::chrono::steady_clock::time_point started)
{
  CommandSweepResult result;
  result.pose = pose;
  if (!motion.available) {result.reason = motion.reason; return result;}
  const auto expired = [&]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >=
             options.budget_seconds;
    };
  const double resolution = map.getResolution();
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
    footprint.size() < 3 || footprint.size() > 64 || !map.getCharMap() ||
    !map.getSizeInCellsX() || !map.getSizeInCellsY() ||
    !std::isfinite(resolution) || resolution <= 0 ||
    !std::isfinite(map.getOriginX()) || !std::isfinite(map.getOriginY()))
  {return result;}
  const double end_x = map.getOriginX() + map.getSizeInCellsX() * resolution;
  const double end_y = map.getOriginY() + map.getSizeInCellsY() * resolution;
  if (!std::isfinite(end_x) || !std::isfinite(end_y)) {return result;}
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
  double position_error = 0., elapsed = 0.;
  for (const auto & segment : motion.segments) {
    result.phase = segment.phase;
    const auto & a = segment.start;
    const auto & b = segment.end;
    const bool constant = a.vx == b.vx && a.vy == b.vy && a.omega == b.omega;
    const auto velocity_at = [&](double t) {
        const double u = t / segment.duration;
        return HolonomicBodyCommand{a.vx + (b.vx - a.vx) * u,
          a.vy + (b.vy - a.vy) * u, a.omega + (b.omega - a.omega) * u};
      };
    const double acceleration = std::hypot(b.vx - a.vx, b.vy - a.vy) / segment.duration;
    double t = 0.;
    while (t < segment.duration) {
      const auto start = velocity_at(t);
      // Affine velocity norms are bounded by the REMAINING interval endpoints.
      // Includes acceleration from rest without oversampling a decelerating tail.
      const double speed = std::max(std::hypot(start.vx, start.vy), std::hypot(b.vx, b.vy));
      const double omega = std::max(std::abs(start.omega), std::abs(b.omega));
      const double point_speed = speed + radius * omega;
      double h = segment.duration - t;
      if (point_speed > 0) {h = std::min(h, options.step / point_speed);}
      if (!std::isfinite(point_speed) || !std::isfinite(acceleration) ||
        !std::isfinite(h) || h <= 0 || t + h == t)
      {
        result.status = SweepStatus::kUnavailable;
        result.reason = "integration_limit"; return result;
      }
      // One expanded footprint encloses the complete interval, not just samples.
      if (!inspect(position_error + h * point_speed)) {return result;}
      const auto middle = velocity_at(t + h / 2);
      const auto end = velocity_at(t + h);
      const double middle_yaw = result.pose.yaw + (start.omega + middle.omega) * h / 4;
      const double yaw_step = (start.omega + end.omega) * h / 2;
      double scale = h;
      if (constant && yaw_step != 0) {
        scale *= std::sin(yaw_step / 2) / (yaw_step / 2);
      } else if (!constant) {
        // Midpoint integration error <= L*h^2/4, L bounds |a_xy| + |v|*|omega|.
        position_error += h * h / 4 * (acceleration + speed * omega);
      }
      const double c = std::cos(middle_yaw), sn = std::sin(middle_yaw);
      result.pose.x += scale * (c * middle.vx - sn * middle.vy);
      result.pose.y += scale * (sn * middle.vx + c * middle.vy);
      result.pose.yaw += yaw_step;
      position_error += 32 * std::numeric_limits<double>::epsilon() *
        (1 + std::abs(result.pose.x) + std::abs(result.pose.y) + speed * h +
        radius * (std::abs(result.pose.yaw) + std::abs(yaw_step)));
      t += h;
      result.time = elapsed + t;
      result.distance += std::hypot(middle.vx, middle.vy) * h;
      if (!std::isfinite(position_error) || !std::isfinite(result.distance) ||
        !std::isfinite(result.pose.yaw) || !std::isfinite(result.time))
      {
        result.status = SweepStatus::kUnavailable;
        result.reason = "integration_limit"; return result;
      }
    }
    elapsed += segment.duration;
  }
  if (!inspect(position_error)) {return result;}
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
  const auto started = std::chrono::steady_clock::now();
  const auto motion = buildCommandMotion(input.velocity, input.hold_time, input.deceleration);
  auto result = evaluate(input.pose, motion, map, footprint, options_, started);
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
CommandSweepResult ExecutionSafety::checkTransition(const SweepPose & pose,
  const MotionTransitionInput & input, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint)
{
  const auto started = std::chrono::steady_clock::now();
  return evaluate(pose, buildMotionTransition(input), map, footprint, options_, started);
}
}  // namespace pid_tracking_controller
