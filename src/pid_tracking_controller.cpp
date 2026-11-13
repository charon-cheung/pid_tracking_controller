/*
 *	Copyright (C), suzhou, muhai
 *	@author:	ZZP
 *	@date:		2025/11/03
 *	@note:
 */
#include "pid_tracking_controller/pid_tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "angular_velocity_feedback_filter.hpp"
#include "fixed_heading_yaw_gate.hpp"
#include "execution_safety_inputs.hpp"
#include "execution_safety_gate.hpp"
#include "holonomic_command_limiter.hpp"
#include "holonomic_control_law.hpp"
#include "holonomic_speed_planner.hpp"
#include "holonomic_tracking_reference.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "path_progress_tracker.hpp"
#include "tracking_progress_publisher.hpp"
#include "prepared_path.hpp"
#include "rotation_completion_tracker.hpp"
#include "rotation_entry_gate.hpp"
#include "rotation_velocity_planner.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace pid_tracking_controller
{
namespace
{

constexpr double kGeometryEpsilon = 1e-10;

bool scalarIsFinite(double value)
{
    return std::isfinite(value);
}

bool quaternionIsFinite(const geometry_msgs::msg::Quaternion& quaternion)
{
    return scalarIsFinite(quaternion.x) && scalarIsFinite(quaternion.y) &&
           scalarIsFinite(quaternion.z) && scalarIsFinite(quaternion.w);
}

bool pointIsFinite(const geometry_msgs::msg::Point& point)
{
    return scalarIsFinite(point.x) && scalarIsFinite(point.y) && scalarIsFinite(point.z);
}

bool poseIsFinite(const geometry_msgs::msg::PoseStamped& pose)
{
    return pointIsFinite(pose.pose.position) && quaternionIsFinite(pose.pose.orientation);
}

bool pathIsFinite(const nav_msgs::msg::Path& path)
{
    return std::all_of(path.poses.begin(), path.poses.end(), poseIsFinite);
}

bool twistIsFinite(const geometry_msgs::msg::Twist& twist)
{
    return scalarIsFinite(twist.linear.x) && scalarIsFinite(twist.linear.y) &&
           scalarIsFinite(twist.linear.z) && scalarIsFinite(twist.angular.x) &&
           scalarIsFinite(twist.angular.y) && scalarIsFinite(twist.angular.z);
}

const char * holonomicReferenceModeName(HolonomicReferenceMode mode)
{
    switch (mode)
    {
        case HolonomicReferenceMode::kTranslation:
            return "translation";
        case HolonomicReferenceMode::kRotationPending:
            return "rotation_pending";
        case HolonomicReferenceMode::kTerminal:
            return "terminal";
    }
    return "unknown";
}

const char * holonomicLimitReasonName(HolonomicSpeedLimitReason reason)
{
    switch (reason) {
        case HolonomicSpeedLimitReason::kTrackingResponse: return "tracking_response";
        case HolonomicSpeedLimitReason::kNominal: return "nominal";
        case HolonomicSpeedLimitReason::kFeedbackBudget: return "feedback_budget";
        case HolonomicSpeedLimitReason::kAccelerationPreview: return "acceleration_preview";
        case HolonomicSpeedLimitReason::kAccelerationBudget: return "acceleration_budget";
        case HolonomicSpeedLimitReason::kExternal: return "external";
        case HolonomicSpeedLimitReason::kBodyAxis: return "body_axis";
        case HolonomicSpeedLimitReason::kCombinedSpeed: return "xy_speed";
        case HolonomicSpeedLimitReason::kYawRate: return "yaw_rate";
        case HolonomicSpeedLimitReason::kTangentRate: return "tangent_rate";
        case HolonomicSpeedLimitReason::kStoppingDistance: return "stopping_distance";
        case HolonomicSpeedLimitReason::kObstacleStop: return "obstacle_stop";
        case HolonomicSpeedLimitReason::kRotation: return "rotation";
        case HolonomicSpeedLimitReason::kTerminal: return "terminal";
    }
    return "unknown";
}


}  // namespace

PidTrackingController::PidTrackingController()
: fixed_heading_yaw_gate_(std::make_unique<FixedHeadingYawGate>()),
  path_progress_tracker_(std::make_unique<PathProgressTracker>())
{
}

PidTrackingController::~PidTrackingController() = default;

void PidTrackingController::resetRotationState(bool preserve_braking)
{
    if (rotation_entry_gate_) {
        rotation_entry_gate_->reset(preserve_braking);
    }
    if (rotation_completion_tracker_) {
        rotation_completion_tracker_->reset();
    }
    rotation_pending_ = false;
    pending_rotation_segment_index_ = 0u;
    rotation_ack_candidate_ = false;
    rotation_last_all_conditions_ = false;
}

void PidTrackingController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    const auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("Unable to lock lifecycle node in PidTrackingController");
    }
    if (!costmap_ros) {
        throw std::runtime_error("PidTrackingController requires a valid local costmap");
    }

    execution_safety_gate_.reset();
    stopExecutionSafety();
    motion_observation_.reset();
    controller_plugin_name = std::move(name);
    control_time_valid_ = false;
    prepared_path_.reset();
    progress_handoff_pending_ = false;
    holonomic_speed_planner_.reset();
    holonomic_command_limiter_.reset();
    rotation_velocity_planner_.reset();
    omega_feedback_sub_.reset();
    omega_feedback_filter_.reset();
    resetRotationState();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    tf_ = std::move(tf);
    declare_parameter_if_not_declared(node, controller_plugin_name + ".base_frame", rclcpp::ParameterValue("base_link") );

    declare_parameter_if_not_declared(node, controller_plugin_name + ".lookahead_time", rclcpp::ParameterValue(0.5) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".min_lookahead_dist", rclcpp::ParameterValue(0.3) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".max_lookahead_dist", rclcpp::ParameterValue(0.9) );

    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".kp_yaw", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".rotation_yaw_tolerance",
      rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".rotation_omega_tolerance",
      rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".rotation_xy_speed_tolerance",
      rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".rotation_settle_time",
      rclcpp::ParameterValue(0.30));

    node->get_parameter(controller_plugin_name + ".base_frame", base_frame_);

    node->get_parameter(controller_plugin_name + ".lookahead_time", lookahead_time_);
    node->get_parameter(controller_plugin_name + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(controller_plugin_name + ".max_lookahead_dist", max_lookahead_dist_);

    double configured_kp_yaw = 1.0;
    RotationCompletionTrackerConfig rotation_completion_config;
    node->get_parameter(controller_plugin_name + ".kp_yaw", configured_kp_yaw);
    node->get_parameter(
      controller_plugin_name + ".rotation_yaw_tolerance",
      rotation_completion_config.rotation_yaw_tolerance);
    node->get_parameter(
      controller_plugin_name + ".rotation_omega_tolerance",
      rotation_completion_config.rotation_omega_tolerance);
    node->get_parameter(
      controller_plugin_name + ".rotation_xy_speed_tolerance",
      rotation_completion_config.rotation_xy_speed_tolerance);
    node->get_parameter(
      controller_plugin_name + ".rotation_settle_time",
      rotation_completion_config.rotation_settle_time);
    if (!std::isfinite(configured_kp_yaw) || configured_kp_yaw < 0.0) {
        throw std::invalid_argument("kp_yaw must be finite and nonnegative");
    }
    auto rotation_completion_tracker =
            std::make_unique<RotationCompletionTracker>(rotation_completion_config);

    kp_yaw_ = configured_kp_yaw;
    rotation_yaw_tolerance_ = rotation_completion_config.rotation_yaw_tolerance;
    rotation_omega_tolerance_ = rotation_completion_config.rotation_omega_tolerance;
    rotation_xy_speed_tolerance_ =
            rotation_completion_config.rotation_xy_speed_tolerance;
    rotation_settle_time_ = rotation_completion_config.rotation_settle_time;
    rotation_completion_tracker_ = std::move(rotation_completion_tracker);
    resetRotationState();

    const auto parameter = [&node, this](const std::string & suffix, double initial) {
        const auto key = controller_plugin_name + "." + suffix;
        declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(initial));
        return node->get_parameter(key).as_double();
    };
    FixedHeadingYawGateConfig yaw_gate_config;
    declare_parameter_if_not_declared(node, controller_plugin_name + ".fixed_heading_yaw_hold",
            rclcpp::ParameterValue(yaw_gate_config.enabled));
    node->get_parameter(controller_plugin_name + ".fixed_heading_yaw_hold", yaw_gate_config.enabled);
    yaw_gate_config.enter_error = parameter("fixed_heading_yaw_enter_error", yaw_gate_config.enter_error);
    yaw_gate_config.exit_error = parameter("fixed_heading_yaw_exit_error", yaw_gate_config.exit_error);
    fixed_heading_yaw_gate_ = std::make_unique<FixedHeadingYawGate>(yaw_gate_config);
    const double omega_filter_tau = parameter("omega_feedback_filter_tau", 0.05);
    if (!std::isfinite(omega_filter_tau) || omega_filter_tau < 0.0) {
        throw std::invalid_argument("omega_feedback_filter_tau must be finite and nonnegative");
    }
    RotationEntryGateConfig entry_config;
    entry_config.xy_speed_tolerance = rotation_xy_speed_tolerance_;
    entry_config.omega_tolerance = rotation_omega_tolerance_;
    entry_config.stop_confirm_time = parameter("rotation_stop_confirm_time", 0.20);
    entry_config.braking_timeout = parameter("rotation_braking_timeout", 8.0);
    rotation_entry_gate_ = std::make_unique<RotationEntryGate>(entry_config);
    HolonomicSpeedPlannerConfig speed_config;
    speed_config.nominal_tracking_speed = parameter("nominal_tracking_speed", 0.20);
    speed_config.max_vx = parameter("max_vx", 0.20);
    speed_config.max_vy = parameter("max_vy", 0.20);
    speed_config.max_xy_speed = parameter("max_xy_speed", 0.20);
    speed_config.max_omega = parameter("max_omega", 0.30);
    speed_config.max_ax = parameter("max_ax", 0.20);
    speed_config.max_ay = parameter("max_ay", 0.20);
    speed_config.a_ref_up = parameter("a_ref_up", 0.20);
    speed_config.a_ref_down = parameter("a_ref_down", 0.20);
    speed_config.stopping_deceleration = parameter("stopping_deceleration", 0.12);
    speed_config.stopping_delay = parameter("stopping_delay", 0.50);
    speed_config.stopping_min_speed = parameter("stopping_min_speed", 0.04);
    speed_config.yaw_feedback_reserve = parameter("yaw_feedback_reserve", 0.05);
    speed_config.configured_min_rate_window = parameter("configured_min_rate_window", 0.30);
    speed_config.max_translation_curve_accel = parameter(
      "max_translation_curve_accel", speed_config.max_translation_curve_accel);
    speed_config.tracking_guard.enter_error = parameter("tracking_guard_enter_error", 0.03);
    speed_config.tracking_guard.exit_error = parameter("tracking_guard_exit_error", 0.015);
    speed_config.tracking_guard.min_outward_speed = parameter("tracking_guard_min_outward_speed", 0.02);
    speed_config.tracking_guard.confirm_time = parameter("tracking_guard_confirm_time", 0.15);
    speed_config.tracking_guard.release_confirm_time = parameter(
      "tracking_guard_release_confirm_time", 0.40);
    HolonomicCommandLimiterConfig limiter_config;
    limiter_config.max_vx = speed_config.max_vx;
    limiter_config.max_vy = speed_config.max_vy;
    limiter_config.max_xy_speed = speed_config.max_xy_speed;
    limiter_config.max_omega = speed_config.max_omega;
    limiter_config.max_ax = speed_config.max_ax;
    limiter_config.max_ay = speed_config.max_ay;
    limiter_config.max_decel_x = parameter("max_decel_x", speed_config.max_ax);
    limiter_config.max_decel_y = parameter("max_decel_y", speed_config.max_ay);
    if (speed_config.stopping_deceleration >
            std::min(limiter_config.max_decel_x, limiter_config.max_decel_y)) {
        throw std::invalid_argument("stopping_deceleration exceeds command braking limits");
    }
    limiter_config.max_alpha = parameter("max_alpha", 0.50);
    limiter_config.rotation_max_alpha = parameter("rotation_max_alpha", limiter_config.max_alpha);
    limiter_config.control_gap_threshold = parameter("control_gap_threshold", 0.50);
    const double kp_x = parameter("kp_x", 1.0);
    const double kp_y = parameter("kp_y", 1.0);
    if (!std::isfinite(kp_x) || kp_x < 0.0 || !std::isfinite(kp_y) || kp_y < 0.0) {
        throw std::invalid_argument("kp_x and kp_y must be finite and nonnegative");
    }
    RotationVelocityPlannerConfig rotation_velocity_config;
    rotation_velocity_config.kp_yaw = parameter("rotation_kp_yaw", 2.5);
    rotation_velocity_config.response_delay = parameter("rotation_response_delay", 0.20);
    rotation_velocity_config.max_omega = limiter_config.max_omega;
    rotation_velocity_config.max_deceleration = limiter_config.max_alpha;
    rotation_velocity_config.yaw_tolerance = rotation_yaw_tolerance_;
    auto rotation_velocity_planner =
            std::make_unique<RotationVelocityPlanner>(rotation_velocity_config);
    auto speed_planner = std::make_unique<HolonomicSpeedPlanner>(speed_config);
    auto command_limiter = std::make_unique<HolonomicCommandLimiter>(limiter_config);
    kp_x_ = kp_x;
    kp_y_ = kp_y;
    max_xy_speed_ = speed_config.max_xy_speed;
    control_gap_threshold_ = limiter_config.control_gap_threshold;
    holonomic_speed_planner_ = std::move(speed_planner);
    holonomic_command_limiter_ = std::move(command_limiter);
    rotation_velocity_planner_ = std::move(rotation_velocity_planner);
    external_speed_limit_.store(std::numeric_limits<double>::infinity(), std::memory_order_relaxed);
    control_time_valid_ = false;
    last_holonomic_mode_ = -1;
    RCLCPP_INFO(logger_, "PID_LOG holonomic_parameters controller=%s", controller_plugin_name.c_str());
    RCLCPP_DEBUG(logger_, "PID_LOG holonomic_parameters nominal=%.3f max_xy_speed=%.3f "
            "max_vx=%.3f max_vy=%.3f max_omega=%.3f max_ax=%.3f max_ay=%.3f "
            "max_alpha=%.3f rotation_max_alpha=%.3f kp_x=%.3f kp_y=%.3f kp_yaw=%.3f gap=%.3f "
            "rotation_stop_confirm=%.3f rotation_braking_timeout=%.3f "
            "stopping_deceleration=%.3f stopping_delay=%.3f stopping_min_speed=%.3f "
            "max_decel_x=%.3f max_decel_y=%.3f "
            "max_translation_curve_accel=%.3f a_ref_up=%.3f a_ref_down=%.3f "
            "tracking_guard_enter=%.3f tracking_guard_exit=%.3f "
            "tracking_guard_outward=%.3f tracking_guard_confirm=%.3f "
            "tracking_guard_release_confirm=%.3f "
            "configured_min_rate_window=%.3f yaw_feedback_reserve=%.3f "
            "fixed_heading_yaw_hold=%s yaw_enter_error=%.3f yaw_exit_error=%.3f",
            speed_config.nominal_tracking_speed, max_xy_speed_, speed_config.max_vx,
            speed_config.max_vy, speed_config.max_omega, speed_config.max_ax,
            speed_config.max_ay, limiter_config.max_alpha, *limiter_config.rotation_max_alpha,
            kp_x_, kp_y_, kp_yaw_,
            control_gap_threshold_, entry_config.stop_confirm_time, entry_config.braking_timeout,
            speed_config.stopping_deceleration, speed_config.stopping_delay,
            speed_config.stopping_min_speed,
            limiter_config.max_decel_x, limiter_config.max_decel_y,
            speed_config.max_translation_curve_accel, speed_config.a_ref_up, speed_config.a_ref_down,
            speed_config.tracking_guard.enter_error, speed_config.tracking_guard.exit_error,
            speed_config.tracking_guard.min_outward_speed, speed_config.tracking_guard.confirm_time,
            speed_config.tracking_guard.release_confirm_time,
            speed_config.configured_min_rate_window, speed_config.yaw_feedback_reserve,
            yaw_gate_config.enabled ? "true" : "false",
            yaw_gate_config.enter_error, yaw_gate_config.exit_error);
    
    node_ = node;
    costmap_ros_ = std::move(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();

    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "pid_tracking_controller/carrot", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    raw_cmd_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "tracking_cmd_vel_raw", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    declare_parameter_if_not_declared(node, controller_plugin_name + ".execution_safety.enabled",
            rclcpp::ParameterValue(false));
    const bool safety_enabled = node->get_parameter(
            controller_plugin_name + ".execution_safety.enabled").as_bool();
    if (safety_enabled) {
        // Controller limits define a nominal hypothesis, not actuator guarantees.
        double controller_frequency = 0.;
        node->get_parameter("controller_frequency", controller_frequency);
        NominalTransitionAssumptions nominal;
        nominal.delay = std::max(speed_config.stopping_delay, rotation_velocity_config.response_delay);
        if (std::isfinite(controller_frequency) && controller_frequency > 0.) {
            nominal.duration = 1. / controller_frequency;
        }
        nominal.acceleration = {speed_config.max_ax, speed_config.max_ay, limiter_config.max_alpha};
        nominal.deceleration = {limiter_config.max_decel_x, limiter_config.max_decel_y, limiter_config.max_alpha};
        nominal.braking_deceleration = {speed_config.stopping_deceleration,
                speed_config.stopping_deceleration, limiter_config.max_alpha};
        motion_observation_ = std::make_shared<MotionObservation>(base_frame_);
        execution_safety_gate_ = std::make_unique<ExecutionSafetyGate>(nominal,
                ExecutionSafetyOptions{}, node->get_parameter("use_sim_time").as_bool());
        RCLCPP_INFO(logger_, "PID_MOTION stage=2-A model=nominal delay=%.3f duration=%.3f "
                "accel=(%.3f,%.3f,%.3f) decel=(%.3f,%.3f,%.3f) brake=(%.3f,%.3f,%.3f) "
                "queued_commands=nominal_history steering=unmodeled physical_response=unverified",
                nominal.delay, nominal.duration, nominal.acceleration.vx, nominal.acceleration.vy,
                nominal.acceleration.omega, nominal.deceleration.vx, nominal.deceleration.vy,
                nominal.deceleration.omega, nominal.braking_deceleration.vx,
                nominal.braking_deceleration.vy, nominal.braking_deceleration.omega);
    }
    if (omega_filter_tau > 0.0 || motion_observation_) {
        declare_parameter_if_not_declared(node, "odom_topic", rclcpp::ParameterValue("odom"));
        const auto topic = node->get_parameter("odom_topic").as_string();
        if (omega_filter_tau > 0.0) {
            omega_feedback_filter_ = std::make_shared<AngularVelocityFeedbackFilter>(omega_filter_tau);
        }
        omega_feedback_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
                topic, rclcpp::SensorDataQoS(),
                [filter = omega_feedback_filter_, frame = base_frame_, motion = motion_observation_](
                    const nav_msgs::msg::Odometry::ConstSharedPtr message) {
                    if (motion) {motion->record(message->header, message->twist.twist, message->child_frame_id);}
                    if (!filter) {return;}
                    if (message->child_frame_id != frame) {
                        filter->reset();
                        return;
                    }
                    const int64_t stamp_ns = static_cast<int64_t>(message->header.stamp.sec) *
                            1000000000LL + message->header.stamp.nanosec;
                    filter->update(message->twist.twist.angular.z, stamp_ns);
                });
        if (motion_observation_) {
            RCLCPP_INFO(logger_, "PID_MOTION stage=2-A source=odometry topic=%s type=Odometry "
                    "velocity_frame=child_frame_id expected_frame=%s subscription=shared_with_omega_filter "
                    "candidate=direct history_capacity=64 pose_max_age=0.300 receipt_max_age=0.600 "
                    "alignment=exact_pose_stamp published_subscription=off smoothed_subscription=off",
                    omega_feedback_sub_->get_topic_name(), base_frame_.c_str());
        }
    }
    RCLCPP_INFO(logger_, "PID_LOG omega_feedback_filter tau=%.3f max_age=0.100 "
            "stop_source=raw reseed_source=%s", omega_filter_tau,
            omega_feedback_filter_ ? "filtered_if_fresh" : "raw");

    declare_parameter_if_not_declared(node, controller_plugin_name + ".publish_tracking_progress",
            rclcpp::ParameterValue(false));
    if (safety_enabled || node->get_parameter(controller_plugin_name + ".publish_tracking_progress").as_bool()) {
        tracking_progress_pub_ = std::make_unique<TrackingProgressPublisher>(node);
    }

    // Snapshot live values rather than reading a possibly different YAML on disk.
    // Keep one parameter per line so downstream tools retain names and value types.
    auto parameter_names = node->list_parameters({controller_plugin_name}, 0).names;
    for (const auto * name : {"controller_frequency", "failure_tolerance", "publish_zero_velocity"}) {
        parameter_names.emplace_back(name);
    }
    std::sort(parameter_names.begin(), parameter_names.end());
    RCLCPP_INFO(logger_, "PID_LOG parameter_snapshot_begin controller=%s count=%zu",
            controller_plugin_name.c_str(), parameter_names.size());
    for (const auto & name : parameter_names) {
        rclcpp::Parameter entry;
        const bool available = node->get_parameter(name, entry);
        std::ostringstream value;
        value.imbue(std::locale::classic());
        if (!available) {
            value << "null";
        } else if (entry.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value << std::fixed << std::setprecision(3) << entry.as_double();
        } else if (entry.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            value << std::quoted(entry.as_string());
        } else {
            value << entry.value_to_string();
        }
        RCLCPP_INFO(logger_, "PID_LOG parameter name=%s type=%s value=%s",
                name.c_str(), available ? entry.get_type_name().c_str() : "not_set",
                value.str().c_str());
    }
    RCLCPP_INFO(logger_, "PID_LOG parameter_snapshot_end controller=%s count=%zu",
            controller_plugin_name.c_str(), parameter_names.size());

    plan_revision_ = 0;
    force_diagnostics_log_ = true;

    RCLCPP_DEBUG(
            logger_,
            "PID_LOG tracking_parameters lookahead_time=%.3f "
            "min_lookahead_dist=%.3f max_lookahead_dist=%.3f "
            "rotation_yaw_tolerance=%.3f rotation_omega_tolerance=%.3f "
            "rotation_xy_speed_tolerance=%.3f rotation_settle_time=%.3f",
            lookahead_time_, min_lookahead_dist_, max_lookahead_dist_,
            rotation_yaw_tolerance_, rotation_omega_tolerance_,
            rotation_xy_speed_tolerance_, rotation_settle_time_);
}

void PidTrackingController::activate()
{
    if (execution_safety_gate_) {execution_safety_gate_->reset();}
    startExecutionSafety();
    if (tracking_progress_pub_) {tracking_progress_pub_->activate();}
    control_time_valid_ = false;
    if (omega_feedback_filter_) {
        omega_feedback_filter_->reset();
    }
    resetRotationState();
    if (carrot_pub_) {
        carrot_pub_->on_activate();
    }
    if (raw_cmd_pub_) {
        raw_cmd_pub_->on_activate();
    }
    force_diagnostics_log_ = true;
}

void PidTrackingController::deactivate()
{
    if (execution_safety_gate_) {execution_safety_gate_->reset();}
    stopExecutionSafety();
    if (tracking_progress_pub_) {tracking_progress_pub_->deactivate();}
    control_time_valid_ = false;
    if (omega_feedback_filter_) {
        omega_feedback_filter_->reset();
    }
    resetRotationState();
    if (carrot_pub_) {
        carrot_pub_->on_deactivate();
    }
    if (raw_cmd_pub_) {
        raw_cmd_pub_->on_deactivate();
    }
}

void PidTrackingController::cleanup()
{
    execution_safety_gate_.reset();
    stopExecutionSafety();
    motion_observation_.reset();
    tracking_progress_pub_.reset();
    control_time_valid_ = false;
    omega_feedback_sub_.reset();
    omega_feedback_filter_.reset();
    holonomic_speed_planner_.reset();
    holonomic_command_limiter_.reset();
    rotation_velocity_planner_.reset();
    resetRotationState();
    carrot_pub_.reset();
    raw_cmd_pub_.reset();
    prepared_path_.reset();
    progress_handoff_pending_ = false;
    costmap_ = nullptr;
    costmap_ros_.reset();
    tf_.reset();
    clock_.reset();
}

void PidTrackingController::reset()
{
    if (execution_safety_gate_) {execution_safety_gate_->reset();}
    if (tracking_progress_pub_) {tracking_progress_pub_->invalidate(true);}
    // The next task seeds the planner and limiter from measured velocity.
    control_time_valid_ = false;
    last_commanded_planar_speed_ = 0.0;
    resetRotationState();
    last_rotation_braking_ = false;
    last_holonomic_mode_ = -1;
    prepared_path_.reset();
    progress_handoff_pending_ = false;
    force_diagnostics_log_ = true;
}

// Every accepted path replaces tracker/planner geometry while retaining command continuity.
void PidTrackingController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
    if (tracking_progress_pub_) {tracking_progress_pub_->invalidate();}
    RCLCPP_DEBUG(logger_, "PidTrackingController received a path");
    resetRotationState(true);
    fixed_heading_yaw_gate_->reset();
    force_diagnostics_log_ = true;
    prepared_path_.reset();
    progress_handoff_pending_ = false;
    progress_handoff_failure_logged_ = false;
    if (orig_global_plan.poses.empty() || !pathIsFinite(orig_global_plan))
    {
        control_time_valid_ = false;
        throw nav2_core::InvalidPath(
                controller_plugin_name + " received an empty or non-finite path");
    }
    try
    {
        auto prepared_path = preparePath(orig_global_plan);
        path_progress_tracker_->setPath(prepared_path);
        if (holonomic_speed_planner_) {
            holonomic_speed_planner_->setPath(prepared_path);
        }
        prepared_path_ = std::move(prepared_path);
    }
    catch (const std::exception & exception)
    {
        control_time_valid_ = false;
        throw nav2_core::InvalidPath(
                controller_plugin_name + " received an invalid path: " + exception.what());
    }
    ++plan_revision_;
    if (tracking_progress_pub_) {
        tracking_progress_pub_->loaded(orig_global_plan, plan_revision_, prepared_path_);
    }
    progress_handoff_pending_ = true;
}

/**
 * @brief Calculate the look-ahead distance with current speed dynamically
 * @param vt the current speed
 * @return L the look-ahead distance
 */
double PidTrackingController::getLookAheadDistance(double vt)
{
    double lookahead_dist = fabs(vt) * lookahead_time_;

    return std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

void PidTrackingController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    try {
        const double cap = convertExternalSpeedLimitToAbsolute(speed_limit, percentage, max_xy_speed_);
        const double previous = external_speed_limit_.exchange(cap, std::memory_order_relaxed);
        if (previous != cap) {
            RCLCPP_INFO(logger_, "PID_LOG speed_limit external_speed_limit=%.3f", cap);
        }
    } catch (const std::invalid_argument & exception) {
        RCLCPP_WARN(logger_, "PID_LOG rejected_speed_limit reason=%s", exception.what());
    }
}

geometry_msgs::msg::TwistStamped PidTrackingController::computeHolonomicCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  const PathProgressReference & progress,
  double lookahead_distance,
  const rclcpp::Time & now)
{
    if (!prepared_path_ || !holonomic_speed_planner_ ||
        !holonomic_command_limiter_ || !rotation_completion_tracker_ || !rotation_entry_gate_ ||
        !rotation_velocity_planner_) {
        throw std::logic_error("holonomic components are unavailable");
    }
    const auto reference = buildHolonomicTrackingReference(
            *prepared_path_, progress, lookahead_distance);
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double cap = external_speed_limit_.load(std::memory_order_relaxed);
    const double dt = control_time_valid_ ? (now - last_control_time_).seconds() : 0.0;
    const bool gap = control_time_valid_ && dt > control_gap_threshold_;
    const bool reseed = !control_time_valid_ || !std::isfinite(dt) || dt <= 0.0 || gap;
    const char * reseed_reason = !control_time_valid_ ? "initial_or_recovery" :
            (gap ? "control_gap" : (reseed ? "nonpositive_dt" : "none"));
    const bool rotating = reference.mode == HolonomicReferenceMode::kRotationPending;
    if (reseed || !rotating || !rotation_pending_ ||
        reference.pending_rotation_segment_index != pending_rotation_segment_index_) {
        resetRotationState(!reseed && rotating);
    }
    if (rotating) {
        if (!reference.pending_rotation_segment_index) {
            throw std::logic_error("rotation reference has no pending index");
        }
        rotation_pending_ = true;
        pending_rotation_segment_index_ = *reference.pending_rotation_segment_index;
    }
    const HolonomicBodyCommand measured{velocity.linear.x, velocity.linear.y, velocity.angular.z};
    const auto omega_sample = omega_feedback_filter_ ?
            omega_feedback_filter_->sample(now.nanoseconds()) : std::nullopt;
    auto limiter_measured = measured;
    if (omega_sample) {
        limiter_measured.omega = omega_sample->filtered;
    }
    const bool braking = rotating && !rotation_entry_gate_->beginCycle(measured);
    if (braking != last_rotation_braking_) {
        force_diagnostics_log_ = true;
    }
    last_rotation_braking_ = braking;

    HolonomicControlInput input;
    input.robot_x = pose.pose.position.x;
    input.robot_y = pose.pose.position.y;
    input.robot_yaw = yaw;
    input.reference_x = rotating ? input.robot_x : reference.current.position.x;
    input.reference_y = rotating ? input.robot_y : reference.current.position.y;
    input.path_tangent = reference.current.path_tangent;
    input.target_yaw = rotating ? reference.rotation_target_yaw : reference.current.target_yaw;
    input.target_yaw_rate_per_meter =
            reference.mode == HolonomicReferenceMode::kTranslation ?
            reference.current.target_yaw_rate_per_meter : 0.0;
    // Use the same pose, reference and gains for the budget and final control law.
    const auto feedback = computeHolonomicRawCommand(input, {kp_x_, kp_y_, kp_yaw_});
    const double normal_x = -std::sin(input.path_tangent - yaw);
    const double normal_y = std::cos(input.path_tangent - yaw);
    const double lateral = -(normal_x * feedback.ex_body + normal_y * feedback.ey_body);
    HolonomicSpeedPlan speed_plan;
    if (reseed) {
        const double relative_tangent = reference.current.path_tangent - yaw;
        const double measured_progress_speed = velocity.linear.x * std::cos(relative_tangent) +
                velocity.linear.y * std::sin(relative_tangent);
        holonomic_speed_planner_->seed(reference.mode == HolonomicReferenceMode::kTranslation ?
                std::max(0.0, measured_progress_speed) : 0.0);
    } else {
        speed_plan = holonomic_speed_planner_->plan(reference, yaw, dt, cap,
                std::max(std::hypot(measured.vx, measured.vy), last_commanded_planar_speed_),
                {feedback.vx_fb, feedback.vy_fb}, holonomic_command_limiter_->previousCommand(),
                HolonomicTrackingObservation{lateral, measured.vx, measured.vy});
    }
    input.reference_velocity = speed_plan.reference_velocity;
    auto raw = computeHolonomicRawCommand(input, {kp_x_, kp_y_, kp_yaw_});
    const bool was_yaw_suppressed = fixed_heading_yaw_gate_->suppressing();
    if (reseed) {
        fixed_heading_yaw_gate_->reset();
    }
    fixed_heading_yaw_gate_->apply(reference, raw);
    const bool yaw_suppressed = fixed_heading_yaw_gate_->suppressing();
    if (yaw_suppressed != was_yaw_suppressed) {
        force_diagnostics_log_ = true;
    }
    RotationVelocityPlan rotation_plan;
    if (rotating && !braking) {
        const auto previous = holonomic_command_limiter_->previousCommand();
        rotation_plan = rotation_velocity_planner_->plan(raw.yaw_error, measured.omega,
                previous ? previous->omega : measured.omega, reseed ? 0.0 : dt);
        raw.omega = raw.omega_fb = rotation_plan.omega;
    }
    if (braking) {
        raw.vx = raw.vy = raw.omega = 0.0;
    }
    const auto accepted_output = reseed ? std::nullopt : holonomic_command_limiter_->previousCommand();
    auto limited = holonomic_command_limiter_->limit(
            {raw.vx, raw.vy, raw.omega},
            limiter_measured,
            reseed ? 0.0 : dt, cap, speed_plan.stopping_speed_limit, rotating && !braking);
    geometry_msgs::msg::TwistStamped command;
    command.header.frame_id = base_frame_;
    command.header.stamp = now;
    command.twist.linear.x = limited.command.vx;
    command.twist.linear.y = limited.command.vy;
    command.twist.angular.z = limited.command.omega;
    bool safety_inhibited = false;
    if (execution_safety_gate_) {
        const auto started = ExecutionSafetyGate::Wall::now();
        const auto pose_stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
        if (pose_stamp != safety_pose_stamp_) {
            safety_pose_stamp_ = pose_stamp; safety_pose_received_ = started;
        }
        auto geometry = execution_safety_inputs_ ? execution_safety_inputs_->capture(
                pose, now, *tf_, *costmap_ros_->getLayeredCostmap(), safety_pose_received_) :
                SafetyInputResult{std::nullopt, "inputs_inactive", ""};
        const auto map_stamp = geometry.snapshot ?
                rclcpp::Time(geometry.snapshot->header.stamp).nanoseconds() : 0;
        const auto candidate = limited.command;
        const auto risk = execution_safety_inputs_ ? execution_safety_inputs_->pathRisk(
                tracking_progress_pub_->state(), now) : PathRiskTrigger{};
        const auto history = motion_observation_->history(now.nanoseconds(), plan_revision_);
        const auto decision = execution_safety_gate_->apply(geometry, history,
                pose_stamp, now.nanoseconds(), plan_revision_, candidate, measured,
                reseed ? 0.0 : dt, accepted_output, *holonomic_command_limiter_, started, risk);
        limited.command = decision.command;
        safety_inhibited = decision.inhibited;
        tracking_progress_pub_->safetyInhibited(safety_inhibited);
        if (safety_inhibited) {
            const double tangent = reference.current.path_tangent - yaw;
            holonomic_speed_planner_->seed(std::max(0.0,
                    decision.command.vx * std::cos(tangent) + decision.command.vy * std::sin(tangent)));
        }
        execution_safety_gate_->report(logger_, decision, candidate, plan_revision_, now.nanoseconds(),
                pose_stamp, map_stamp, std::chrono::duration<double, std::milli>(
                ExecutionSafetyGate::Wall::now() - started).count(), history);
        command.twist.linear.x = decision.command.vx;
        command.twist.linear.y = decision.command.vy;
        command.twist.angular.z = decision.command.omega;
    }
    last_commanded_planar_speed_ = std::hypot(limited.command.vx, limited.command.vy);
    if (!twistIsFinite(command.twist)) {
        throw std::domain_error("holonomic command is non-finite");
    }

    RotationCompletionStatus completion;
    const bool previous_candidate = rotation_ack_candidate_;
    // Stop/rotation evidence uses the actual gated output. It must continue
    // while inhibited, otherwise a zero-command rotation entry can never finish.
    if (braking) {
        rotation_entry_gate_->observeBraking(measured, limited.command, reseed ? 0.0 : dt);
    } else if (rotating) {
        // Only intervals bounded by two qualifying measurements count as settled.
        completion = rotation_completion_tracker_->update(
                raw.yaw_error, velocity.linear.x, velocity.linear.y, velocity.angular.z,
                !reseed && rotation_last_all_conditions_ ? dt : 0.0);
        rotation_last_all_conditions_ = completion.all_conditions;
        rotation_ack_candidate_ = !reseed && completion.acknowledge_candidate;
    }
    last_control_time_ = now;
    control_time_valid_ = true;
    const double now_seconds = now.seconds();
    const bool log_event = force_diagnostics_log_ || reseed ||
            last_holonomic_mode_ != static_cast<int>(reference.mode);
    if (log_event) {
        if (gap || dt < 0.0) {
            RCLCPP_INFO(logger_, "PID_LOG control_time_anomaly control_now=%.3f dt=%.3f",
                    now_seconds, dt);
        }
        RCLCPP_INFO(logger_,
                "PID_LOG holonomic plan_revision=%zu mode=%s "
                "reseed=%s control_gap=%s yaw_hold=%s output=(%.3f,%.3f,%.3f)",
                plan_revision_, holonomicReferenceModeName(reference.mode),
                reseed_reason, gap ? "true" : "false", yaw_suppressed ? "true" : "false",
                command.twist.linear.x, command.twist.linear.y, command.twist.angular.z);
        if (rotating) {
            RCLCPP_INFO(logger_,
                    "PID_LOG holonomic_rotation plan_revision=%zu "
                    "rotation_index=%zu rotation_phase=%s yaw_error=%.3f measured_omega=%.3f",
                    plan_revision_, pending_rotation_segment_index_,
                    braking ? "braking" : "rotating", raw.yaw_error, measured.omega);
        }
    }
    if (log_event || previous_candidate != rotation_ack_candidate_ ||
        now_seconds - last_diagnostics_log_time_seconds_ >= 1.0) {
        RCLCPP_DEBUG(logger_,
                "PID_LOG holonomic plan_revision=%zu mode=%s control_now=%.3f planner_step=%s "
                "reference_effective=%.3f external_speed_limit=%.3f limit_reason=%s "
                "raw_vx=%.3f raw_vy=%.3f limited_vx=%.3f limited_vy=%.3f "
                "translation_lambda=%.3f reseed=%s control_gap=%s "
                "stop_speed_limit=%.3f stop_limit_unreachable=%s "
                "feedback_speed_limit=%.3f feedback_unreachable=%s "
                "acceleration_speed_limit=%.3f acceleration_unreachable=%s "
                "vx_saturated=%s vy_saturated=%s xy_saturated=%s ax_limited=%s ay_limited=%s "
                "yaw_hold=%s yaw_error=%.3f raw_omega=%.3f limited_omega=%.3f "
                "measured_omega=%.3f filtered_omega=%.3f omega_filter_valid=%s",
                plan_revision_, holonomicReferenceModeName(reference.mode), now_seconds,
                reseed ? "false" : "true",
                speed_plan.reference_velocity, cap,
                reseed ? "not_planned" : holonomicLimitReasonName(speed_plan.speed_limit_reason),
                raw.vx, raw.vy, limited.command.vx, limited.command.vy,
                limited.translation_lambda, reseed_reason, gap ? "true" : "false",
                speed_plan.stopping_speed_limit, limited.stopping_limit_unreachable ? "true" : "false",
                speed_plan.feedback_speed_limit, speed_plan.feedback_unreachable ? "true" : "false",
                speed_plan.acceleration_speed_limit, speed_plan.acceleration_unreachable ? "true" : "false",
                limited.vx_saturated ? "true" : "false", limited.vy_saturated ? "true" : "false",
                limited.xy_saturated ? "true" : "false", limited.ax_limited ? "true" : "false",
                limited.ay_limited ? "true" : "false", yaw_suppressed ? "true" : "false",
                raw.yaw_error, raw.omega, limited.command.omega,
                measured.omega, limiter_measured.omega, omega_sample ? "true" : "false");
        if (rotating) {
            RCLCPP_DEBUG(logger_,
                    "PID_LOG holonomic_rotation plan_revision=%zu control_now=%.3f "
                    "rotation_index=%zu rotation_phase=%s raw_omega=%.3f limited_omega=%.3f "
                    "stop_confirmed=%.3f rotation_settled=%.3f ack_candidate=%s "
                    "rotation_speed_cap=%.3f predicted_stop_angle=%.3f",
                    plan_revision_, now_seconds, pending_rotation_segment_index_,
                    braking ? "braking" : "rotating", raw.omega, limited.command.omega,
                    rotation_entry_gate_->stoppedDuration(), completion.settled_duration,
                    rotation_ack_candidate_ ? "true" : "false",
                    rotation_plan.stopping_speed_limit, rotation_plan.predicted_stopping_angle);
        }
        // Detailed tracking evidence is opt-in through DEBUG logging.
        if (logger_.get_effective_level() <= rclcpp::Logger::Level::Debug) {
            // Keep progress and control-law evidence in the same diagnostic cycle.
            const auto & goal = prepared_path_->poses.back().pose;
            const auto segment_index = progress.cursor.segment_index;
            const auto * segment = segment_index < prepared_path_->segments.size() ?
                    &prepared_path_->segments[segment_index] : nullptr;
            const double absent = std::numeric_limits<double>::quiet_NaN();
            std::size_t next_rotation = progress.prepared_segment_lower_bound;
            while (next_rotation < prepared_path_->segments.size() &&
                    prepared_path_->segments[next_rotation].type != PathSegmentType::kRotation) {
                ++next_rotation;
            }
            const bool has_rotation = next_rotation < prepared_path_->segments.size();
            RCLCPP_DEBUG(logger_,
                    "PID_LOG tracking_state plan_revision=%zu mode=%s control_now=%.3f "
                    "robot_x=%.3f robot_y=%.3f robot_yaw=%.3f "
                    "goal_x=%.3f goal_y=%.3f goal_distance=%.3f goal_yaw_error=%.3f "
                    "segment=%zu source_index=%zu fraction=%.3f lower_bound=%zu segment_count=%zu "
                    "segment_type=%s segment_s_begin=%.3f segment_s_end=%.3f "
                    "progress_s=%.3f total_s=%.3f remaining_s=%.3f budget_remaining=%.3f "
                    "next_rotation_index=%s next_rotation_s=%.3f "
                    "projection_x=%.3f projection_y=%.3f reference_x=%.3f reference_y=%.3f "
                    "carrot_x=%.3f carrot_y=%.3f carrot_s=%.3f "
                    "path_tangent=%.3f target_yaw=%.3f ex_body=%.3f ey_body=%.3f "
                    "vx_ff=%.3f vy_ff=%.3f vx_fb=%.3f vy_fb=%.3f "
                    "measured_vx=%.3f measured_vy=%.3f measured_omega=%.3f",
                    plan_revision_, holonomicReferenceModeName(reference.mode), now_seconds,
                    input.robot_x, input.robot_y, yaw, goal.position.x, goal.position.y,
                    std::hypot(goal.position.x - input.robot_x, goal.position.y - input.robot_y),
                    angles::normalize_angle(tf2::getYaw(goal.orientation) - yaw),
                    segment_index, progress.segment_index, progress.cursor.segment_fraction,
                    progress.prepared_segment_lower_bound, prepared_path_->segments.size(),
                    segment ? (segment->type == PathSegmentType::kRotation ? "rotation" : "translation") :
                    "terminal",
                    segment ? segment->translation_s_begin : absent,
                    segment ? segment->translation_s_end : absent,
                    progress.progress_arc_length, prepared_path_->total_translation_length,
                    progress.remaining_path_length, progress.progress_budget,
                    has_rotation ? std::to_string(next_rotation).c_str() : "none",
                    has_rotation ? prepared_path_->segments[next_rotation].translation_s_begin : absent,
                    progress.projection.x, progress.projection.y, input.reference_x, input.reference_y,
                    progress.carrot.x, progress.carrot.y, progress.carrot_arc_length,
                    input.path_tangent, input.target_yaw, raw.ex_body, raw.ey_body,
                    raw.vx_ff, raw.vy_ff, raw.vx_fb, raw.vy_fb,
                    measured.vx, measured.vy, measured.omega);
        }
        last_diagnostics_log_time_seconds_ = now_seconds;
        force_diagnostics_log_ = false;
    }
    // DEBUG response capture: only sample translation anomalies, at most 10 Hz.
    // These are same-cycle differences, not estimates of physical response delay.
    if (reference.mode == HolonomicReferenceMode::kTranslation &&
        logger_.get_effective_level() <= rclcpp::Logger::Level::Debug) {
        const double response_delta = std::hypot(
                limited.command.vx - measured.vx, limited.command.vy - measured.vy);
        const double limiter_delta = std::hypot(
                raw.vx - limited.command.vx, raw.vy - limited.command.vy);
        if (std::abs(lateral) >= 0.02 || response_delta >= 0.06 || limiter_delta >= 0.005 ||
            speed_plan.tracking_acceleration_held) {
            // Pose header age is not sensor/odometry latency; velocity has no stamp here.
            const double pose_stamp = static_cast<double>(pose.header.stamp.sec) +
                    1e-9 * pose.header.stamp.nanosec;
            RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 100,
                    "PID_LOG translation_response plan_revision=%zu control_now=%.3f dt=%.3f "
                    "planner_step=%s pose_stamp=%.3f pose_header_age=%.3f "
                    "progress_s=%.3f segment=%zu robot_yaw=%.3f path_tangent=%.3f lateral=%.3f "
                    "reference_target=%.3f reference_effective=%.3f limit_reason=%s "
                    "tangent_rate=%.3f rate_window=%.3f acceleration_cap=%.3f feedback_cap=%.3f "
                    "local_tangent_rate=%.3f tracking_hold=%s tracking_cap=%.3f "
                    "vx_ff=%.3f vy_ff=%.3f vx_fb=%.3f vy_fb=%.3f "
                    "raw_vx=%.3f raw_vy=%.3f limited_vx=%.3f limited_vy=%.3f "
                    "measured_vx=%.3f measured_vy=%.3f normal_command=%.3f normal_measured=%.3f "
                    "translation_lambda=%.3f ax_limited=%s ay_limited=%s "
                    "yaw_hold=%s yaw_error=%.3f limited_omega=%.3f measured_omega=%.3f",
                    plan_revision_, now_seconds, dt, reseed ? "false" : "true", pose_stamp,
                    pose_stamp == 0.0 ? std::numeric_limits<double>::quiet_NaN() : now_seconds - pose_stamp,
                    progress.progress_arc_length, progress.cursor.segment_index, yaw, input.path_tangent,
                    lateral, speed_plan.reference_velocity_target, speed_plan.reference_velocity,
                    reseed ? "not_planned" : holonomicLimitReasonName(speed_plan.speed_limit_reason),
                    speed_plan.max_abs_path_tangent_rate_per_meter, speed_plan.rate_window_length,
                    speed_plan.acceleration_speed_limit, speed_plan.feedback_speed_limit,
                    speed_plan.local_path_tangent_rate_per_meter,
                    speed_plan.tracking_acceleration_held ? "true" : "false", speed_plan.tracking_speed_limit,
                    raw.vx_ff, raw.vy_ff, raw.vx_fb, raw.vy_fb, raw.vx, raw.vy,
                    limited.command.vx, limited.command.vy, measured.vx, measured.vy,
                    normal_x * limited.command.vx + normal_y * limited.command.vy,
                    normal_x * measured.vx + normal_y * measured.vy,
                    limited.translation_lambda, limited.ax_limited ? "true" : "false",
                    limited.ay_limited ? "true" : "false", yaw_suppressed ? "true" : "false",
                    raw.yaw_error, limited.command.omega, measured.omega);
        }
    }
    last_holonomic_mode_ = static_cast<int>(reference.mode);
    if (raw_cmd_pub_ && raw_cmd_pub_->is_activated()) {
        // Exact limiter input, including yaw suppression and rotation-entry braking.
        geometry_msgs::msg::TwistStamped raw_command;
        raw_command.header = command.header;
        raw_command.twist.linear.x = raw.vx;
        raw_command.twist.linear.y = raw.vy;
        raw_command.twist.angular.z = raw.omega;
        raw_cmd_pub_->publish(raw_command);
    }
    return command;
}

geometry_msgs::msg::TwistStamped PidTrackingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
    try {
        if (!prepared_path_) {
            throw nav2_core::InvalidPath(controller_plugin_name + " has no path to track");
        }
        const auto & q = pose.pose.orientation;
        const double quaternion_norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (!poseIsFinite(pose) || !twistIsFinite(velocity) ||
            !std::isfinite(quaternion_norm) || quaternion_norm < kGeometryEpsilon) {
            throw std::domain_error("received an invalid pose or velocity");
        }
        if (!clock_ || !costmap_) {
            throw std::logic_error("controller is not configured for control");
        }
        if (pose.header.frame_id.empty()) {
            throw nav2_core::ControllerTFError("received pose with empty frame_id");
        }
        auto tracking_pose = pose;
        if (pose.header.frame_id != prepared_path_->frame_id) {
            if (!tf_) {
                throw nav2_core::ControllerTFError("controller has no TF buffer");
            }
            try {
                // Use the pose timestamp without waiting inside the control loop.
                tf_->transform(pose, tracking_pose, prepared_path_->frame_id,
                        tf2::durationFromSec(0.0));
            } catch (const tf2::TransformException & exception) {
                throw nav2_core::ControllerTFError(
                        "failed to transform robot pose from " + pose.header.frame_id +
                        " to " + prepared_path_->frame_id + ": " + exception.what());
            }
        }
        const auto now = clock_->now();
        const double lookahead = getLookAheadDistance(
                std::hypot(velocity.linear.x, velocity.linear.y));
        if (progress_handoff_pending_) {
            PathProgressReference handoff;
            try {
                handoff = path_progress_tracker_->initializePathHandoff(
                        tracking_pose.pose.position, lookahead);
            } catch (const std::exception & exception) {
                if (!progress_handoff_failure_logged_) {
                    RCLCPP_WARN(logger_,
                            "PID_LOG path_handoff_rejected plan_revision=%zu frame=%s "
                            "robot_x=%.3f robot_y=%.3f reason=%s",
                            plan_revision_, prepared_path_->frame_id.c_str(),
                            tracking_pose.pose.position.x, tracking_pose.pose.position.y,
                            exception.what());
                    progress_handoff_failure_logged_ = true;
                }
                throw;
            }
            progress_handoff_pending_ = false;
            const auto skipped_rotations = std::count_if(prepared_path_->segments.begin(),
                    prepared_path_->segments.begin() + handoff.prepared_segment_lower_bound,
                    [](const auto & segment) { return segment.type == PathSegmentType::kRotation; });
            RCLCPP_INFO(logger_,
                    "PID_LOG path_handoff plan_revision=%zu frame=%s accepted_s=%.3f "
                    "projection_distance=%.3f skipped_rotations=%zu",
                    plan_revision_, handoff.frame_id.c_str(), handoff.progress_arc_length,
                    std::hypot(tracking_pose.pose.position.x - handoff.projection.x,
                    tracking_pose.pose.position.y - handoff.projection.y),
                    static_cast<std::size_t>(skipped_rotations));
        }
        const auto progress = path_progress_tracker_->update(tracking_pose.pose.position, lookahead);
        auto command = computeHolonomicCommand(tracking_pose, velocity, progress, lookahead, now);
        geometry_msgs::msg::PointStamped carrot;
        carrot.header.frame_id = progress.frame_id;
        carrot.header.stamp = now;
        carrot.point = progress.carrot;
        if (carrot_pub_ && carrot_pub_->is_activated()) {
            carrot_pub_->publish(carrot);
        }
        if (tracking_progress_pub_) {
            tracking_progress_pub_->update(progress, tracking_pose, rotation_pending_);
        }
        // Commit progress only after the entire active command has been validated.
        if (rotation_ack_candidate_) {
            path_progress_tracker_->acknowledgeRotationComplete(pending_rotation_segment_index_);
            RCLCPP_INFO(logger_, "PID_LOG rotation_ack plan_revision=%zu rotation_index=%zu",
                    plan_revision_, pending_rotation_segment_index_);
            resetRotationState();
            force_diagnostics_log_ = true;
        }
        return command;
    } catch (const nav2_core::InvalidPath &) {
        if (tracking_progress_pub_) {tracking_progress_pub_->invalidate();}
        control_time_valid_ = false;
        resetRotationState();
        throw;
    } catch (const nav2_core::ControllerTFError &) {
        if (tracking_progress_pub_) {tracking_progress_pub_->invalidate();}
        control_time_valid_ = false;
        resetRotationState();
        throw;
    } catch (const std::exception & exception) {
        if (tracking_progress_pub_) {tracking_progress_pub_->invalidate();}
        control_time_valid_ = false;
        resetRotationState();
        throw nav2_core::NoValidControl(
                controller_plugin_name + " holonomic control failed: " + exception.what());
    }
}

}  // namespace pid_tracking_controller

PLUGINLIB_EXPORT_CLASS(
  pid_tracking_controller::PidTrackingController,
  nav2_core::Controller)
