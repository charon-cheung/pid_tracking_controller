#include "execution_safety_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pid_tracking_controller
{
ExecutionSafetyAdapter::ExecutionSafetyAdapter(
  rclcpp_lifecycle::LifecycleNode::WeakPtr node,
  rclcpp_lifecycle::LifecycleNode::SharedPtr costmap, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<MotionObservation> motion, std::string smoothed_topic,
  std::optional<NominalTransitionAssumptions> nominal)
: node_(node), costmap_(std::move(costmap)), tf_(std::move(tf)),
  motion_(std::move(motion)), smoothed_topic_(std::move(smoothed_topic)), nominal_(std::move(nominal)),
  logger_(rclcpp::get_logger("PidTrackingController")) {}

ExecutionSafetyAdapter::~ExecutionSafetyAdapter() {stop();}

void ExecutionSafetyAdapter::start()
{
  stop();
  try {
    const auto node = node_.lock();
    if (!node || !costmap_ || !tf_) {throw std::runtime_error("missing_node_costmap_or_tf");}
    logger_ = node->get_logger(); clock_ = node->get_clock();
    config_changed_ = std::make_shared<std::atomic<bool>>(false);
    // A small atomic callback, never subscription rebuilding in the costmap thread.
    parameter_callback_ = costmap_->add_post_set_parameters_callback(
      [changed = config_changed_](const std::vector<rclcpp::Parameter> &) {changed->store(true);});
    group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    connect(node);
    if (motion_) {motion_->activate();}
    transition_observer_.reset();
    RCLCPP_INFO(logger_, "PID_TRANSITION started mode=observe_only action=none history_capacity=%zu "
      "pose_odom_alignment=exact execution_evidence=missing physical_response=unverified "
      "poll_ms=100 heartbeat_s=5", MotionObservation::kHistoryCapacity);
    if (nominal_) {
      const auto & n = *nominal_;
      RCLCPP_INFO(logger_, "PID_TRANSITION model basis=nominal source=controller_parameters "
        "delay=%.6f duration=%.6f acceleration=(%.6f,%.6f,%.6f) deceleration=(%.6f,%.6f,%.6f) "
        "braking=(%.6f,%.6f,%.6f) delay_motion=hold_measured queued_commands=unmodeled "
        "steering=unmodeled physical_response=unverified action=none control_authorized=0",
        n.delay, n.duration, n.acceleration.vx, n.acceleration.vy, n.acceleration.omega,
        n.deceleration.vx, n.deceleration.vy, n.deceleration.omega,
        n.braking_deceleration.vx, n.braking_deceleration.vy, n.braking_deceleration.omega);
    }
    last_status_.clear(); last_report_ = {}; ready_count_ = unavailable_count_ = 0;
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() {observe();}, group_);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = node->get_node_base_interface()->get_context();
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_callback_group(group_, node->get_node_base_interface());
    running_ = true;
    worker_ = std::thread([this, context = executor_options.context]() {
        try {
          // Bounded wait also makes stop safe if cancel precedes the first spin.
          while (running_ && rclcpp::ok(context)) {executor_->spin_once(std::chrono::milliseconds(50));}
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger_, "PID_OBSERVE worker_failed reason=%s mode=observe_only action=none", e.what());
        }
      });
  } catch (const std::exception & e) {
    stop();
    RCLCPP_WARN(logger_, "PID_OBSERVE unavailable reason=%s mode=observe_only action=none", e.what());
  }
}

void ExecutionSafetyAdapter::stop()
{
  running_ = false;
  if (executor_) {executor_->cancel();}
  if (worker_.joinable()) {
    worker_.join();
    transition_observer_.stopped(logger_);
    RCLCPP_INFO(logger_, "PID_OBSERVE stopped ready_checks=%lu unavailable_checks=%lu action=none",
      ready_count_, unavailable_count_);
  }
  if (motion_) {motion_->deactivate();}
  timer_.reset(); subscriptions_.clear(); executor_.reset(); group_.reset();
  parameter_callback_.reset(); inputs_.reset(); sources_.clear(); footprint_.reset();
  map_info_ = {"map", "", "Costmap"}; footprint_info_ = {"footprint", "", "PolygonStamped"};
}

void ExecutionSafetyAdapter::connect(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  options_ = SafetyInputOptions{};
  options_.map_frame = costmap_->get_parameter("global_frame").as_string();
  options_.base_frame = costmap_->get_parameter("robot_base_frame").as_string();
  const double rate = costmap_->get_parameter("update_frequency").as_double();
  if (!std::isfinite(rate) || rate <= 0 ||
    !costmap_->get_parameter("always_send_full_costmap").as_bool())
  {throw std::runtime_error("full_costmap_and_positive_update_frequency_required");}
  options_.map_update_period = 1. / rate;
  const auto topic = [&](const std::string & name) {
      return costmap_->get_node_topics_interface()->resolve_topic_name(name);
    };
  const auto layers = costmap_->get_parameter("plugins").as_string_array();
  for (const auto & layer : layers) {
    if (!costmap_->has_parameter(layer + ".observation_sources") ||
      !costmap_->get_parameter(layer + ".enabled").as_bool()) {continue;}
    std::istringstream names(costmap_->get_parameter(layer + ".observation_sources").as_string());
    for (std::string name; names >> name; ) {
      const auto key = layer + "." + name;
      const auto get = [&](const char * suffix) {return costmap_->get_parameter(key + "." + suffix);};
      const bool marking = get("marking").as_bool(), clearing = get("clearing").as_bool();
      if (!marking && !clearing) {continue;}
      const auto raw_topic = get("topic").as_string(), type = get("data_type").as_string();
      const auto age = get("expected_update_rate").as_double();
      if (raw_topic.empty() || !std::isfinite(age) || age < 0 ||
        (type != "LaserScan" && type != "PointCloud2"))
      {throw std::runtime_error("unsupported_or_invalid_observation_source:" + key);}
      const double max_age = age > 0 ? std::min(age, options_.max_age) : options_.max_age;
      options_.sources.push_back({key, max_age});
      sources_.push_back({key, topic(raw_topic), type});
      RCLCPP_INFO(logger_, "PID_OBSERVE source=%s topic=%s type=%s marking=%d clearing=%d "
        "expected_update_rate=%.3f max_age=%.3f mode=observe_only",
        key.c_str(), sources_.back().topic.c_str(), type.c_str(), marking, clearing, age, max_age);
    }
  }
  if (sources_.empty()) {throw std::runtime_error("no_active_observation_sources");}
  inputs_ = std::make_unique<ExecutionSafetyInputs>(options_);
  rclcpp::SubscriptionOptions sub_options; sub_options.callback_group = group_;
  if (motion_) {
    for (const auto & entry : {std::pair<MotionObservation::Stage, std::string>{
          MotionObservation::Published, "cmd_vel"}, {MotionObservation::Smoothed, smoothed_topic_}})
    {
      subscriptions_.push_back(node->create_subscription<geometry_msgs::msg::TwistStamped>(
        entry.second, rclcpp::SensorDataQoS().keep_last(1),
        [motion = motion_, stage = entry.first](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
          motion->record(stage, m->header, m->twist, m->header.frame_id);
        }, sub_options));
      RCLCPP_INFO(logger_, "PID_MOTION source=%s topic=%s type=TwistStamped "
        "header_semantics=%s publisher_identity=topic_only mode=observe_only action=none",
        entry.first == MotionObservation::Published ? "published" : "smoothed",
        subscriptions_.back()->get_topic_name(),
        entry.first == MotionObservation::Published ? "command_stamp" : "upstream_or_producer_stamp");
    }
  }
  for (size_t i = 0; i < sources_.size(); ++i) {
    bool duplicate = false;
    for (size_t j = 0; j < i; ++j) {
      if (sources_[j].topic != sources_[i].topic) {continue;}
      if (sources_[j].type != sources_[i].type) {throw std::runtime_error("conflicting_observation_types");}
      duplicate = true;
    }
    if (duplicate) {continue;}
    const auto record = [this, wire = sources_[i].topic](const std_msgs::msg::Header & h, bool valid) {
        for (auto & source : sources_) {
          if (source.topic != wire) {continue;}
          received(source, h, valid);
          inputs_->recordObservation(source.name, h, valid, source.received);
        }
      };
    if (sources_[i].type == "LaserScan") {
      subscriptions_.push_back(node->create_subscription<sensor_msgs::msg::LaserScan>(
        sources_[i].topic, rclcpp::SensorDataQoS().keep_last(1),
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
        }, sub_options));
    } else {
      subscriptions_.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(
        sources_[i].topic, rclcpp::SensorDataQoS().keep_last(1),
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
        }, sub_options));
    }
  }
  map_info_.topic = topic("costmap_raw"); footprint_info_.topic = topic("published_footprint");
  subscriptions_.push_back(node->create_subscription<nav2_msgs::msg::Costmap>(map_info_.topic,
      rclcpp::QoS(1).reliable().transient_local(), [this](nav2_msgs::msg::Costmap::ConstSharedPtr m) {
        received(map_info_, m->header, true); inputs_->recordMap(m, map_info_.received);
      }, sub_options));
  subscriptions_.push_back(node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_info_.topic, rclcpp::QoS(1), [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr m) {
        received(footprint_info_, m->header, true); footprint_ = m;
        inputs_->recordFootprint(m, footprint_info_.received);
      }, sub_options));
  RCLCPP_INFO(logger_, "PID_OBSERVE started mode=observe_only action=none map=%s footprint=%s "
    "map_frame=%s base_frame=%s sources=%zu subscriptions=%zu max_age=%.3f pose_max_age=%.3f "
    "map_period=%.3f budget_ms=%.1f poll_ms=100 heartbeat_s=5 pose_anchor=footprint_stamp",
    map_info_.topic.c_str(), footprint_info_.topic.c_str(), options_.map_frame.c_str(),
    options_.base_frame.c_str(), sources_.size(), subscriptions_.size(), options_.max_age,
    options_.pose_max_age, options_.map_update_period, 1000. * options_.budget_seconds);
}

void ExecutionSafetyAdapter::received(Input & input, const std_msgs::msg::Header & h, bool valid)
{
  input.header = h; input.received = Wall::now(); ++input.count; input.valid = valid;
}

void ExecutionSafetyAdapter::observe()
{
  if (motion_) {motion_->report(logger_, clock_->now().nanoseconds());}
  const auto started = Wall::now();
  SafetyInputResult result;
  int64_t pose_stamp_ns = 0;
  if (config_changed_->load()) {
    result.reason = "configuration_changed_reactivate";
  } else {
    geometry_msgs::msg::PoseStamped pose;
    // This pose is derived from the received footprint event, not a repeatedly
    // polled latest TF. Exact-time lookup cannot refresh an old dynamic transform.
    if (footprint_) {
      try {
        const auto transform = tf_->lookupTransform(options_.map_frame, options_.base_frame,
            rclcpp::Time(footprint_->header.stamp));
        pose_stamp_ns = rclcpp::Time(footprint_->header.stamp).nanoseconds();
        pose.header = footprint_->header; pose.header.frame_id = options_.map_frame;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
      } catch (const tf2::TransformException &) {result.reason = "robot_tf_unavailable";}
    }
    if (result.reason.empty()) {
      result = inputs_->capture(pose, clock_->now(), *tf_, footprint_info_.received);
    }
  }
  if (config_changed_->load()) {
    result.snapshot.reset(); result.source.clear(); result.reason = "configuration_changed_reactivate";
  }
  const auto captured = Wall::now();
  const auto history = motion_ ? motion_->history() : MotionObservation::History{};
  const auto copied = Wall::now();
  const auto ros_now = clock_->now().nanoseconds();
  // Execution evidence remains absent. Nominal prediction is a separately labeled
  // hypothesis and cannot replace the unavailable execution-evidence result.
  const auto transition = transition_observer_.evaluate(result, history, pose_stamp_ns, ros_now, started,
      nullptr, nominal_ ? &*nominal_ : nullptr);
  const double capture_ms = std::chrono::duration<double, std::milli>(captured - started).count();
  transition_observer_.report(logger_, transition, history, ros_now, capture_ms,
    std::chrono::duration<double, std::milli>(copied - captured).count());
  report(result, capture_ms);
}

void ExecutionSafetyAdapter::report(const SafetyInputResult & result, double elapsed_ms)
{
  if (result.snapshot) {++ready_count_;} else {++unavailable_count_;}
  const auto now = Wall::now();
  const auto key = result.reason + ":" + result.source;
  if (key == last_status_ && now - last_report_ < std::chrono::seconds(5)) {return;}
  last_status_ = key; last_report_ = now;
  RCLCPP_INFO(logger_, "PID_OBSERVE status mode=observe_only ready=%s reason=%s source=%s "
    "action=none capture_ms=%.3f ready_checks=%lu unavailable_checks=%lu",
    result.snapshot ? "true" : "false", result.reason.c_str(), result.source.c_str(),
    elapsed_ms, ready_count_, unavailable_count_);
  const auto ros_now = clock_->now();
  const auto detail = [&](const Input & input) {
      const double age = input.count ? (ros_now.nanoseconds() -
        (int64_t(input.header.stamp.sec) * 1000000000LL + input.header.stamp.nanosec)) * 1e-9 : -1.;
      const double wall_age = input.count ? std::chrono::duration<double>(now - input.received).count() : -1.;
      RCLCPP_INFO(logger_, "PID_OBSERVE input=%s topic=%s count=%lu validation=%s "
        "frame=%s stamp=%d.%09u ros_age=%.3f receipt_age=%.3f",
        input.name.c_str(), input.topic.c_str(), input.count,
        !input.count ? "not_received" : (input.type == "LaserScan" || input.type == "PointCloud2") ?
        (input.valid ? "payload_valid" : "payload_invalid") : "deferred_to_capture",
        input.header.frame_id.c_str(),
        input.header.stamp.sec, input.header.stamp.nanosec, age, wall_age);
    };
  detail(map_info_); detail(footprint_info_);
  for (const auto & source : sources_) {detail(source);}
}
}  // namespace pid_tracking_controller
