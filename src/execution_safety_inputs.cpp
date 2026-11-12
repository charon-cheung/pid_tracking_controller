#include "execution_safety_inputs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
  ++reset_epoch_;
  footprint_.reset(); footprint_stamp_ = last_now_ = 0;
  path_risk_.reset(); risk_received_ = {};
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
  const rclcpp::Time & now, tf2_ros::Buffer & tf, nav2_costmap_2d::LayeredCostmap & costmap,
  Wall::time_point pose_received,
  Wall::time_point wall_now)
{
  const auto started = Wall::now();
  const auto now_ns = now.nanoseconds();
  SafetyInputResult result;
  bool reused_map = false;
  const auto finish = [&](const char * reason) {
      if (configuration_changed_) {
        result.snapshot.reset(); reason = "configuration_changed_reactivate";
      }
      if (!result.snapshot) {cached_map_ = {};}
      result.reason = reason;
      const auto grid = result.snapshot ? (reused_map ? "cached" : "direct") : "none";
      const auto key = result.reason + ":" + result.source + ":" + grid;
      if (key != last_log_key_) {
        RCLCPP_INFO(rclcpp::get_logger("PidTrackingController"),
          "PID_SAFETY inputs ready=%s reason=%s source=%s grid=%s map_age=%.3f",
          result.snapshot ? "true" : "false", reason, result.source.c_str(), grid,
          result.snapshot ? (now_ns - stamp(result.snapshot->header.stamp)) * 1e-9 : -1.);
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
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
  Wall::time_point footprint_received;
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
    if (cache_epoch_ != reset_epoch_) {cached_map_ = {}; cache_epoch_ = reset_epoch_;}
    footprint = footprint_; observations = observations_;
    footprint_received = footprint_received_;
  }
  if (configuration_changed_) {return finish("configuration_changed_reactivate");}
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
  SafetyInputSnapshot snapshot;
  snapshot.pose_stamp_ns = stamp(pose.header.stamp);
  {
    auto & map = *costmap.getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex(), std::try_to_lock);
    if (!lock.owns_lock()) {
      if (!cached_map_.map || cached_map_.owner != &costmap) {return finish("map_busy");}
      if (!fresh(cached_map_.footprint->header, now_ns, options_.max_age) ||
        !wall_fresh(cached_map_.received, options_.max_age))
      {return finish("cached_map_stale");}
      // Keep the grid's original update witness, never the latest callback stamp.
      footprint = cached_map_.footprint; footprint_received = cached_map_.received;
      snapshot.map = std::make_unique<nav2_costmap_2d::Costmap2D>(*cached_map_.map);
      reused_map = true;
    } else {
      cached_map_ = {};  // Known-invalid live data must fence previously good data.
      // LayeredCostmap::isCurrent() writes a non-atomic aggregate also used by
      // ControllerServer. Read the layer flags under the update mutex instead.
      const auto current = [](const auto & layers) {
          return std::all_of(layers.begin(), layers.end(), [](const auto & layer) {
              return !layer->isEnabled() || layer->isCurrent();
            });
        };
      if (!costmap.isInitialized() || !current(*costmap.getPlugins()) || !current(*costmap.getFilters())) {
        return finish("map_not_current");
      }
      const auto x = map.getSizeInCellsX(), y = map.getSizeInCellsY();
      const uint64_t cells = uint64_t(x) * y;
      if (!x || !y || cells > options_.max_cells || !map.getCharMap() ||
        !positive(map.getResolution()) || !std::isfinite(map.getOriginX()) ||
        !std::isfinite(map.getOriginY()) ||
        !std::isfinite(map.getOriginX() + x * map.getResolution()) ||
        !std::isfinite(map.getOriginY() + y * map.getResolution()))
      {return finish("invalid_map_geometry");}
      if (expired()) {return finish("input_budget");}
      snapshot.map = std::make_unique<nav2_costmap_2d::Costmap2D>(x, y, map.getResolution(),
          map.getOriginX(), map.getOriginY(), 0);
      std::copy_n(map.getCharMap(), cells, snapshot.map->getCharMap());
    }
  }  // Release the live map before TF checks and all trajectory evaluation.
  snapshot.header = footprint->header;
  snapshot.header.frame_id = options_.map_frame;

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
    const auto map_time = stamp(footprint->header.stamp);
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
  if (!reused_map) {
    // Separate ownership preserves the existing mutable snapshot API. Neither
    // downstream users nor later live-map updates can mutate the retained grid.
    cached_map_.map = std::make_unique<nav2_costmap_2d::Costmap2D>(*snapshot.map);
    cached_map_.owner = &costmap; cached_map_.footprint = footprint;
    cached_map_.received = footprint_received;
  }
  if (expired()) {return finish("input_budget");}
  result.snapshot = std::move(snapshot);
  return finish("ready");
}

SafetyInputOptions safetyInputOptions(nav2_costmap_2d::Costmap2DROS & costmap)
{
  SafetyInputOptions options;
  options.map_frame = costmap.getGlobalFrameID();
  options.base_frame = costmap.getBaseFrameID();
  const auto rate = costmap.get_parameter("update_frequency").as_double();
  if (!positive(rate)) {throw std::invalid_argument("positive_update_frequency_required");}
  options.map_update_period = 1. / rate;
  const auto layers = costmap.get_parameter("plugins").as_string_array();
  for (const auto & layer : layers) {
    if (!costmap.has_parameter(layer + ".observation_sources") ||
      !costmap.get_parameter(layer + ".enabled").as_bool()) {continue;}
    std::istringstream names(costmap.get_parameter(layer + ".observation_sources").as_string());
    for (std::string name; names >> name; ) {
      const auto key = layer + "." + name;
      const auto get = [&](const char * suffix) {return costmap.get_parameter(key + "." + suffix);};
      if (!get("marking").as_bool() && !get("clearing").as_bool()) {continue;}
      const auto topic = get("topic").as_string(), type = get("data_type").as_string();
      const auto age = get("expected_update_rate").as_double();
      if (topic.empty() || !std::isfinite(age) || age < 0 ||
        (type != "LaserScan" && type != "PointCloud2"))
      {throw std::invalid_argument("unsupported_or_invalid_observation_source:" + key);}
      options.sources.push_back({key, age > 0 ? std::min(age, options.max_age) : options.max_age,
        costmap.get_node_topics_interface()->resolve_topic_name(topic), type});
    }
  }
  return options;
}

void ExecutionSafetyInputs::subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap)
{
  parameter_callback_ = costmap->add_post_set_parameters_callback(
    [weak = weak_from_this()](const std::vector<rclcpp::Parameter> &) {
      if (const auto self = weak.lock()) {self->configuration_changed_ = true;}
    });
  // Fence a parameter change between reading options and installing the callback.
  // Changes after registration invalidate capture through configuration_changed_.
  const auto live = safetyInputOptions(*costmap);
  if (live.map_frame != options_.map_frame || live.base_frame != options_.base_frame ||
    live.map_update_period != options_.map_update_period || live.sources.size() != options_.sources.size() ||
    !std::equal(live.sources.begin(), live.sources.end(), options_.sources.begin(),
      [](const auto & a, const auto & b) {
        return a.name == b.name && a.max_age == b.max_age && a.topic == b.topic && a.type == b.type;
      })) {configuration_changed_ = true;}
  for (const auto & source : options_.sources) {
    RCLCPP_INFO(node->get_logger(), "PID_SAFETY source=%s topic=%s type=%s max_age=%.3f",
      source.name.c_str(), source.topic.c_str(), source.type.c_str(), source.max_age);
  }
  for (size_t i = 0; i < options_.sources.size(); ++i) {
    bool duplicate = false;
    for (size_t j = 0; j < i; ++j) {
      if (options_.sources[j].topic != options_.sources[i].topic) {continue;}
      if (options_.sources[j].type != options_.sources[i].type) {throw std::runtime_error("conflicting_observation_types");}
      duplicate = true;
    }
    if (duplicate) {continue;}
    const auto record = [weak = weak_from_this(), wire = options_.sources[i].topic](
      const std_msgs::msg::Header & h, bool valid) {
        if (const auto self = weak.lock()) {
          for (const auto & source : self->options_.sources) {
            if (source.topic == wire) {self->recordObservation(source.name, h, valid);}
          }
        }
      };
    if (options_.sources[i].type == "LaserScan") {
      subscriptions_.push_back(node->create_subscription<sensor_msgs::msg::LaserScan>(
        options_.sources[i].topic, rclcpp::SensorDataQoS().keep_last(1),
        [record](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
          record(m->header, !m->ranges.empty() && std::isfinite(m->angle_min) &&
            std::isfinite(m->angle_max) && std::isfinite(m->angle_increment) && m->angle_increment > 0 &&
            std::isfinite(m->time_increment) && m->time_increment >= 0 &&
            std::isfinite(m->scan_time) && m->scan_time >= 0 &&
            std::isfinite(m->range_min) && m->range_min >= 0 &&
            std::isfinite(m->range_max) && m->range_max > m->range_min &&
            std::any_of(m->ranges.begin(), m->ranges.end(), [&](float range) {
              // Positive infinity is a valid no-return scan; individual NaNs may be dropped.
              return (std::isfinite(range) && range >= m->range_min && range <= m->range_max) ||
                     (std::isinf(range) && range > 0);
            }));
        }));
    } else {
      subscriptions_.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(
        options_.sources[i].topic, rclcpp::SensorDataQoS().keep_last(1),
        [record](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          bool valid = m->point_step > 0 && m->height > 0 &&
            m->row_step >= uint64_t(m->width) * m->point_step &&
            m->data.size() == uint64_t(m->height) * m->row_step;
          for (const auto * axis : {"x", "y", "z"}) {
            valid = valid && std::any_of(m->fields.begin(), m->fields.end(), [&](const auto & f) {
                return f.name == axis && f.count == 1 &&
                  f.datatype == sensor_msgs::msg::PointField::FLOAT32 && uint64_t(f.offset) + 4 <= m->point_step;
              });
          }
          record(m->header, valid);
        }));
    }
  }
  subscriptions_.push_back(node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      costmap->get_node_topics_interface()->resolve_topic_name("published_footprint"), rclcpp::QoS(1),
      [weak = weak_from_this()](geometry_msgs::msg::PolygonStamped::ConstSharedPtr message) {
        if (const auto self = weak.lock()) {self->recordFootprint(std::move(message));}
      }));
  RCLCPP_INFO(node->get_logger(), "PID_SAFETY inputs grid=local_costmap_direct "
    "update_witness=published_footprint busy_map=reuse_unexpired source_count=%zu executor=controller "
    "map_frame=%s base_frame=%s map_period=%.3f max_age=%.3f budget_ms=%.1f",
    options_.sources.size(), options_.map_frame.c_str(), options_.base_frame.c_str(),
    options_.map_update_period, options_.max_age, options_.budget_seconds * 1000.);
}

void ExecutionSafetyInputs::recordPathRisk(msg::PathRiskState::ConstSharedPtr risk)
{
  std::lock_guard<std::mutex> lock(mutex_);
  path_risk_ = std::move(risk); risk_received_ = Wall::now();
}

PathRiskTrigger ExecutionSafetyInputs::pathRisk(
  const msg::TrackingProgress & current, const rclcpp::Time & now)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {return {msg::PathRiskState::UNAVAILABLE, "inputs_busy"};}
  return validatePathRisk(path_risk_.get(), risk_received_, current, now.nanoseconds(), Wall::now());
}

}  // namespace pid_tracking_controller
