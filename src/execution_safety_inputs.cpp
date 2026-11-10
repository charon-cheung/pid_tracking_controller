#include "execution_safety_inputs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pid_tracking_controller
{
namespace
{
int64_t stamp(const builtin_interfaces::msg::Time & t)
{
  return t.sec < 0 || t.nanosec >= 1000000000u ? 0 : int64_t(t.sec) * 1000000000LL + t.nanosec;
}

bool positive(double x) {return std::isfinite(x) && x > 0.;}

bool fresh(const std_msgs::msg::Header & h, int64_t now, double age)
{
  const auto t = stamp(h.stamp);
  return !h.frame_id.empty() && t > 0 && t <= now && (now - t) * 1e-9 <= age;
}

bool planar(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(norm) && std::abs(norm - 1.) < 1e-3 &&
         std::abs(q.x) < 1e-3 && std::abs(q.y) < 1e-3;
}
}  // namespace

ExecutionSafetyInputs::ExecutionSafetyInputs(const SafetyInputOptions & options)
: options_(options)
{
  if (options.map_frame.empty() || options.base_frame.empty() ||
    !positive(options.max_age) || !positive(options.pose_max_age) ||
    !positive(options.map_update_period) || !positive(options.budget_seconds) ||
    !options.max_cells || options.max_cells > std::numeric_limits<unsigned int>::max() ||
    options.sources.empty() || options.sources.size() > 16)
  {throw std::invalid_argument("invalid execution safety input options");}
  std::set<std::string> names;
  for (const auto & source : options.sources) {
    if (source.name.empty() || !positive(source.max_age) || !names.insert(source.name).second) {
      throw std::invalid_argument("invalid execution safety observation source");
    }
    observations_.push_back({source, {}, 0, {}});
  }
}

void ExecutionSafetyInputs::recordMap(
  nav2_msgs::msg::Costmap::ConstSharedPtr map,
  Wall::time_point received)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto t = map ? stamp(map->header.stamp) : 0;
  if (t <= map_stamp_) {map_.reset(); return;}
  map_stamp_ = t; map_received_ = received; map_ = std::move(map);
}

void ExecutionSafetyInputs::recordFootprint(
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint, Wall::time_point received)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto t = footprint ? stamp(footprint->header.stamp) : 0;
  if (t <= footprint_stamp_) {footprint_.reset(); return;}
  footprint_stamp_ = t; footprint_received_ = received; footprint_ = std::move(footprint);
}

void ExecutionSafetyInputs::recordObservation(
  const std::string & source,
  const std_msgs::msg::Header & header, bool valid, Wall::time_point received)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & observation : observations_) {
    if (observation.source.name != source) {continue;}
    const auto t = stamp(header.stamp);
    if (!valid || header.frame_id.empty() || t <= observation.last_stamp) {
      observation.last_stamp = std::max(t, observation.last_stamp);
      observation.samples.clear(); return;
    }
    if (!observation.samples.empty() &&
      ((t - observation.last_stamp) * 1e-9 > observation.source.max_age ||
      received < observation.received ||
      std::chrono::duration<double>(received - observation.received).count() >
      observation.source.max_age || header.frame_id != observation.samples.back().first.frame_id))
    {observation.samples.clear();}
    observation.last_stamp = t;
    observation.received = received;
    observation.samples.emplace_back(header, received);
    // Bounded history; if the map lags beyond this window, reject instead of guessing.
    while (observation.samples.size() > 32) {observation.samples.pop_front();}
    return;
  }
}

void ExecutionSafetyInputs::clear()
{
  map_.reset(); footprint_.reset(); map_stamp_ = footprint_stamp_ = last_now_ = 0;
  for (auto & observation : observations_) {
    observation.samples.clear(); observation.last_stamp = 0;
  }
}

void ExecutionSafetyInputs::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  clear();
}

SafetyInputResult ExecutionSafetyInputs::capture(
  const geometry_msgs::msg::PoseStamped & pose,
  const rclcpp::Time & now, tf2_ros::Buffer & tf, Wall::time_point pose_received,
  Wall::time_point wall_now)
{
  const auto started = Wall::now();
  const auto now_ns = now.nanoseconds();
  SafetyInputResult result;
  const auto finish = [&](const char * reason) {
      result.reason = reason;
      const auto key = result.reason + ":" + result.source;
      if (key != last_log_key_) {
        RCLCPP_INFO(rclcpp::get_logger("PidTrackingController"),
          "PID_SAFETY inputs ready=%s reason=%s source=%s",
          result.snapshot ? "true" : "false", reason, result.source.c_str());
        last_log_key_ = key;
      }
      return std::move(result);
    };
  const auto expired = [&]() {
      return std::chrono::duration<double>(Wall::now() - started).count() >=
             options_.budget_seconds;
    };
  const auto wall_fresh = [&](Wall::time_point received, double age) {
      return received <= wall_now &&
             std::chrono::duration<double>(wall_now - received).count() <= age;
    };
  nav2_msgs::msg::Costmap::ConstSharedPtr map;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
  Wall::time_point map_received, footprint_received;
  std::vector<Observation> observations;
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {return finish("inputs_busy");}
    if (now_ns <= 0 || now.get_clock_type() != RCL_ROS_TIME ||
      (last_now_ && now_ns < last_now_))
    {
      clear(); last_now_ = now_ns;
      return finish("clock_invalid_or_regressed");
    }
    last_now_ = now_ns;
    map = map_; footprint = footprint_; observations = observations_;
    map_received = map_received_; footprint_received = footprint_received_;
  }
  if (!map || !fresh(map->header, now_ns, options_.max_age) ||
    !wall_fresh(map_received, options_.max_age)) {return finish("map_missing_or_stale");}
  if (map->header.frame_id != options_.map_frame) {return finish("map_frame_mismatch");}
  if (!footprint || !fresh(footprint->header, now_ns, options_.max_age) ||
    !wall_fresh(footprint_received, options_.max_age))
  {
    return finish("footprint_missing_or_stale");
  }
  if (!fresh(pose.header, now_ns, options_.pose_max_age) ||
    !wall_fresh(pose_received, options_.pose_max_age) || !planar(pose.pose))
  {
    return finish("pose_invalid_or_stale");
  }
  const auto & m = map->metadata;
  const uint64_t cells = uint64_t(m.size_x) * m.size_y;
  if (!m.size_x || !m.size_y || cells > options_.max_cells || map->data.size() != cells ||
    !positive(m.resolution) || !planar(m.origin) ||
    std::abs(m.origin.orientation.x) > 1e-6 || std::abs(m.origin.orientation.y) > 1e-6 ||
    std::abs(m.origin.orientation.z) > 1e-6 ||
    !std::isfinite(m.origin.position.x + m.size_x * double(m.resolution)) ||
    !std::isfinite(m.origin.position.y + m.size_y * double(m.resolution)))
  {return finish("invalid_map_geometry");}

  for (const auto & observation : observations) {
    result.source = observation.source.name;
    if (observation.samples.empty() ||
      !wall_fresh(observation.received, observation.source.max_age) ||
      !fresh(observation.samples.back().first, now_ns, observation.source.max_age))
    {return finish("observation_missing_or_stale");}
    const auto & latest = observation.samples.back().first;
    if (!tf.canTransform(options_.map_frame, latest.frame_id, rclcpp::Time(latest.stamp))) {
      return finish("observation_tf_unavailable");
    }
    bool anchored = false;
    const auto map_time = stamp(map->header.stamp);
    for (const auto & sample : observation.samples) {
      const auto & h = sample.first;
      if (expired()) {return finish("input_budget");}
      if (wall_fresh(sample.second, observation.source.max_age) &&
        fresh(h, map_time, observation.source.max_age) &&
        (map_time - stamp(h.stamp)) * 1e-9 >= options_.map_update_period &&
        tf.canTransform(options_.map_frame, h.frame_id, rclcpp::Time(h.stamp)))
      {anchored = true; break;}
    }
    if (!anchored) {return finish("observation_not_anchored_to_map");}
  }
  result.source.clear();
  SafetyInputSnapshot snapshot;
  snapshot.header = map->header;
  try {
    auto map_pose = pose;
    if (pose.header.frame_id != options_.map_frame) {
      tf.transform(pose, map_pose, options_.map_frame, tf2::durationFromSec(0.));
    }
    if (!planar(map_pose.pose)) {return finish("invalid_transformed_pose");}
    snapshot.pose = {map_pose.pose.position.x, map_pose.pose.position.y,
      tf2::getYaw(map_pose.pose.orientation)};
    const auto & points = footprint->polygon.points;
    if (points.size() < 3 || points.size() > 64) {return finish("invalid_footprint");}
    geometry_msgs::msg::TransformStamped body_transform;
    if (footprint->header.frame_id != options_.base_frame) {
      body_transform = tf.lookupTransform(options_.base_frame, footprint->header.frame_id,
          rclcpp::Time(footprint->header.stamp));
      geometry_msgs::msg::Pose transform_pose;
      transform_pose.position.x = body_transform.transform.translation.x;
      transform_pose.position.y = body_transform.transform.translation.y;
      transform_pose.position.z = body_transform.transform.translation.z;
      transform_pose.orientation = body_transform.transform.rotation;
      if (!planar(transform_pose)) {return finish("nonplanar_footprint_transform");}
    }
    for (const auto & p : points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return finish("invalid_footprint");
      }
      geometry_msgs::msg::PointStamped point;
      point.header = footprint->header;
      point.point.x = p.x; point.point.y = p.y; point.point.z = p.z;
      if (footprint->header.frame_id != options_.base_frame) {
        tf2::doTransform(point, point, body_transform);
      }
      if (!std::isfinite(point.point.x) || !std::isfinite(point.point.y) ||
        !std::isfinite(point.point.z)) {return finish("invalid_footprint");}
      snapshot.footprint.push_back(point.point);
    }
  } catch (const tf2::TransformException &) {
    return finish("pose_or_footprint_tf_unavailable");
  }
  double area = 0.;
  for (std::size_t i = 0; i < snapshot.footprint.size(); ++i) {
    const auto & a = snapshot.footprint[i];
    const auto & b = snapshot.footprint[(i + 1) % snapshot.footprint.size()];
    area += a.x * b.y - b.x * a.y;
  }
  if (!std::isfinite(area) || area == 0.) {return finish("invalid_footprint");}
  if (expired()) {return finish("input_budget");}
  snapshot.map = std::make_unique<nav2_costmap_2d::Costmap2D>(m.size_x, m.size_y, m.resolution,
      m.origin.position.x, m.origin.position.y, 0);
  std::copy(map->data.begin(), map->data.end(), snapshot.map->getCharMap());
  if (expired()) {return finish("input_budget");}
  result.snapshot = std::move(snapshot);
  return finish("ready");
}
}  // namespace pid_tracking_controller
