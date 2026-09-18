/*
 *	Copyright (C), suzhou, muhai
 *	@author:	ZZP
 *	@date:		2025/11/03
 *	@note:
 */
#include "pid_tracking_controller/pid_tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "holonomic_command_limiter.hpp"
#include "holonomic_control_law.hpp"
#include "holonomic_speed_planner.hpp"
#include "holonomic_tracking_reference.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "path_progress_tracker.hpp"
#include "prepared_path.hpp"
#include "rotation_completion_tracker.hpp"
#include "rotation_entry_gate.hpp"
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
        case HolonomicSpeedLimitReason::kNominal: return "nominal";
        case HolonomicSpeedLimitReason::kExternal: return "external";
        case HolonomicSpeedLimitReason::kBodyAxis: return "body_axis";
        case HolonomicSpeedLimitReason::kCombinedSpeed: return "xy_speed";
        case HolonomicSpeedLimitReason::kYawRate: return "yaw_rate";
        case HolonomicSpeedLimitReason::kTangentRate: return "tangent_rate";
        case HolonomicSpeedLimitReason::kStoppingDistance: return "stopping_distance";
        case HolonomicSpeedLimitReason::kRotation: return "rotation";
        case HolonomicSpeedLimitReason::kTerminal: return "terminal";
    }
    return "unknown";
}

}  // namespace

PidTrackingController::PidTrackingController()
: path_progress_tracker_(std::make_unique<PathProgressTracker>())
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

    controller_plugin_name = std::move(name);
    control_time_valid_ = false;
    prepared_path_.reset();
    holonomic_speed_planner_.reset();
    holonomic_command_limiter_.reset();
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
    speed_config.stopping_min_speed = parameter("stopping_min_speed", 0.02);
    speed_config.yaw_feedback_reserve = parameter("yaw_feedback_reserve", 0.05);
    speed_config.configured_min_rate_window = parameter("configured_min_rate_window", 0.30);
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
    limiter_config.control_gap_threshold = parameter("control_gap_threshold", 0.50);
    const double kp_x = parameter("kp_x", 1.0);
    const double kp_y = parameter("kp_y", 1.0);
    if (!std::isfinite(kp_x) || kp_x < 0.0 || !std::isfinite(kp_y) || kp_y < 0.0) {
        throw std::invalid_argument("kp_x and kp_y must be finite and nonnegative");
    }
    auto speed_planner = std::make_unique<HolonomicSpeedPlanner>(speed_config);
    auto command_limiter = std::make_unique<HolonomicCommandLimiter>(limiter_config);
    kp_x_ = kp_x;
    kp_y_ = kp_y;
    max_xy_speed_ = speed_config.max_xy_speed;
    control_gap_threshold_ = limiter_config.control_gap_threshold;
    holonomic_speed_planner_ = std::move(speed_planner);
    holonomic_command_limiter_ = std::move(command_limiter);
    external_speed_limit_.store(std::numeric_limits<double>::infinity(), std::memory_order_relaxed);
    control_time_valid_ = false;
    last_holonomic_mode_ = -1;
    RCLCPP_INFO(logger_, "PID_LOG holonomic_parameters nominal=%.3f max_xy_speed=%.3f "
            "max_vx=%.3f max_vy=%.3f max_omega=%.3f max_ax=%.3f max_ay=%.3f "
            "max_alpha=%.3f kp_x=%.3f kp_y=%.3f kp_yaw=%.3f gap=%.3f "
            "rotation_stop_confirm=%.3f rotation_braking_timeout=%.3f "
            "stopping_deceleration=%.3f stopping_delay=%.3f stopping_min_speed=%.3f "
            "max_decel_x=%.3f max_decel_y=%.3f",
            speed_config.nominal_tracking_speed, max_xy_speed_, speed_config.max_vx,
            speed_config.max_vy, speed_config.max_omega, speed_config.max_ax,
            speed_config.max_ay, limiter_config.max_alpha, kp_x_, kp_y_, kp_yaw_,
            control_gap_threshold_, entry_config.stop_confirm_time, entry_config.braking_timeout,
            speed_config.stopping_deceleration, speed_config.stopping_delay,
            speed_config.stopping_min_speed,
            limiter_config.max_decel_x, limiter_config.max_decel_y);
    
    costmap_ros_ = std::move(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();

    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "pid_tracking_controller/carrot", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    plan_revision_ = 0;
    force_diagnostics_log_ = true;

    RCLCPP_INFO(
            logger_,
            "PID_LOG tracking_parameters lookahead_time=%.6f "
            "min_lookahead_dist=%.6f max_lookahead_dist=%.6f "
            "rotation_yaw_tolerance=%.6f rotation_omega_tolerance=%.6f "
            "rotation_xy_speed_tolerance=%.6f rotation_settle_time=%.6f",
            lookahead_time_, min_lookahead_dist_, max_lookahead_dist_,
            rotation_yaw_tolerance_, rotation_omega_tolerance_,
            rotation_xy_speed_tolerance_, rotation_settle_time_);
}

void PidTrackingController::activate()
{
    control_time_valid_ = false;
    resetRotationState();
    if (carrot_pub_) {
        carrot_pub_->on_activate();
    }
    force_diagnostics_log_ = true;
}

void PidTrackingController::deactivate()
{
    control_time_valid_ = false;
    resetRotationState();
    if (carrot_pub_) {
        carrot_pub_->on_deactivate();
    }
}

void PidTrackingController::cleanup()
{
    control_time_valid_ = false;
    holonomic_speed_planner_.reset();
    holonomic_command_limiter_.reset();
    resetRotationState();
    carrot_pub_.reset();
    prepared_path_.reset();
    costmap_ = nullptr;
    costmap_ros_.reset();
    tf_.reset();
    clock_.reset();
}

// Every accepted path replaces tracker/planner geometry while retaining command continuity.
void PidTrackingController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
    RCLCPP_INFO(logger_, "PidTrackingController received a path");
    resetRotationState(true);
    force_diagnostics_log_ = true;
    prepared_path_.reset();
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
        external_speed_limit_.store(cap, std::memory_order_relaxed);
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
        !holonomic_command_limiter_ || !rotation_completion_tracker_ || !rotation_entry_gate_) {
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
    const bool braking = rotating && !rotation_entry_gate_->beginCycle(measured);
    if (braking != last_rotation_braking_) {
        force_diagnostics_log_ = true;
    }
    last_rotation_braking_ = braking;

    HolonomicSpeedPlan speed_plan;
    if (reseed) {
        const double relative_tangent = reference.current.path_tangent - yaw;
        const double measured_progress_speed = velocity.linear.x * std::cos(relative_tangent) +
                velocity.linear.y * std::sin(relative_tangent);
        holonomic_speed_planner_->seed(reference.mode == HolonomicReferenceMode::kTranslation ?
                std::max(0.0, measured_progress_speed) : 0.0);
    } else {
        speed_plan = holonomic_speed_planner_->plan(reference, yaw, dt, cap,
                std::max(std::hypot(measured.vx, measured.vy), last_commanded_planar_speed_));
    }
    HolonomicControlInput input;
    input.robot_x = pose.pose.position.x;
    input.robot_y = pose.pose.position.y;
    input.robot_yaw = yaw;
    input.reference_x = rotating ? input.robot_x : reference.current.position.x;
    input.reference_y = rotating ? input.robot_y : reference.current.position.y;
    input.path_tangent = reference.current.path_tangent;
    input.target_yaw = rotating ? reference.rotation_target_yaw : reference.current.target_yaw;
    input.reference_velocity = speed_plan.reference_velocity;
    input.target_yaw_rate_per_meter =
            reference.mode == HolonomicReferenceMode::kTranslation ?
            reference.current.target_yaw_rate_per_meter : 0.0;
    auto raw = computeHolonomicRawCommand(input, {kp_x_, kp_y_, kp_yaw_});
    if (braking) {
        raw.vx = raw.vy = raw.omega = 0.0;
    }
    const auto limited = holonomic_command_limiter_->limit(
            {raw.vx, raw.vy, raw.omega},
            measured,
            reseed ? 0.0 : dt, cap, speed_plan.stopping_speed_limit);
    last_commanded_planar_speed_ = std::hypot(limited.command.vx, limited.command.vy);
    geometry_msgs::msg::TwistStamped command;
    command.header.frame_id = base_frame_;
    command.header.stamp = now;
    command.twist.linear.x = limited.command.vx;
    command.twist.linear.y = limited.command.vy;
    command.twist.angular.z = limited.command.omega;
    if (!twistIsFinite(command.twist)) {
        throw std::domain_error("holonomic command is non-finite");
    }

    RotationCompletionStatus completion;
    const bool previous_candidate = rotation_ack_candidate_;
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
    if (force_diagnostics_log_ || reseed ||
        last_holonomic_mode_ != static_cast<int>(reference.mode) ||
        previous_candidate != rotation_ack_candidate_ ||
        now_seconds - last_diagnostics_log_time_seconds_ >= 1.0) {
        RCLCPP_INFO(logger_,
                "PID_LOG holonomic plan_revision=%zu mode=%s planner_step=%s "
                "reference_target=%.6f reference_effective=%.6f limit_reason=%s external_limit=%.6f "
                "raw_vx=%.6f raw_vy=%.6f raw_omega=%.6f limited_vx=%.6f limited_vy=%.6f "
                "limited_omega=%.6f translation_lambda=%.6f reseed=%s control_gap=%s "
                "rotation_index=%s rotation_phase=%s stop_confirmed=%.6f "
                "rotation_settled=%.6f ack_candidate=%s "
                "stop_distance=%.6f stop_delay_distance=%.6f stop_speed_limit=%.6f "
                "stop_limit_unreachable=%s",
                plan_revision_, holonomicReferenceModeName(reference.mode), reseed ? "false" : "true",
                reseed ? std::numeric_limits<double>::quiet_NaN() : speed_plan.reference_velocity_target,
                speed_plan.reference_velocity,
                reseed ? "not_planned" : holonomicLimitReasonName(speed_plan.speed_limit_reason), cap,
                raw.vx, raw.vy, raw.omega, limited.command.vx, limited.command.vy, limited.command.omega,
                limited.translation_lambda, reseed_reason, gap ? "true" : "false",
                rotating ? std::to_string(pending_rotation_segment_index_).c_str() : "none",
                rotating ? (braking ? "braking" : "rotating") : "none",
                rotation_entry_gate_->stoppedDuration(),
                completion.settled_duration, rotation_ack_candidate_ ? "true" : "false",
                speed_plan.stopping_distance, speed_plan.stopping_delay_distance,
                speed_plan.stopping_speed_limit, limited.stopping_limit_unreachable ? "true" : "false");
        last_diagnostics_log_time_seconds_ = now_seconds;
        force_diagnostics_log_ = false;
    }
    last_holonomic_mode_ = static_cast<int>(reference.mode);
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
        const auto now = clock_->now();
        // Temporary TRA-3 observation probe: remove after validating live pose stamps.
        RCLCPP_INFO(logger_,
                "PID_LOG pose_observation plan_revision=%zu control_now=%.9f "
                "pose_stamp_sec=%d pose_stamp_nanosec=%u frame=%s "
                "x=%.9f y=%.9f yaw=%.9f vx=%.6f vy=%.6f omega=%.6f",
                plan_revision_, now.seconds(), pose.header.stamp.sec,
                pose.header.stamp.nanosec, pose.header.frame_id.c_str(),
                pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation),
                velocity.linear.x, velocity.linear.y, velocity.angular.z);
        const double lookahead = getLookAheadDistance(
                std::hypot(velocity.linear.x, velocity.linear.y));
        const auto progress = path_progress_tracker_->update(pose.pose.position, lookahead);
        auto command = computeHolonomicCommand(pose, velocity, progress, lookahead, now);
        geometry_msgs::msg::PointStamped carrot;
        carrot.header.frame_id = progress.frame_id;
        carrot.header.stamp = now;
        carrot.point = progress.carrot;
        if (carrot_pub_ && carrot_pub_->is_activated()) {
            carrot_pub_->publish(carrot);
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
        control_time_valid_ = false;
        resetRotationState();
        throw;
    } catch (const std::exception & exception) {
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
