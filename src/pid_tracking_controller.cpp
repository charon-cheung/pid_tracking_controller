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
#include <optional>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "holonomic_command_limiter.hpp"
#include "holonomic_control_law.hpp"
#include "holonomic_speed_planner.hpp"
#include "holonomic_tracking_reference.hpp"
#include "legacy_carrot_velocity_planner.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "path_progress_tracker.hpp"
#include "prepared_path.hpp"
#include "rotation_completion_tracker.hpp"
#include "rotation_entry_gate.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <sstream>

using namespace nav2_util::geometry_utils;
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

bool isNearAbsoluteLimit(double value, double limit)
{
    const double absolute_limit = std::abs(limit);
    const double tolerance = std::max(1e-4, absolute_limit * 1e-3);
    return std::abs(value) >= absolute_limit - tolerance;
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

LegacyCarrotVelocityPlannerConfig makeLegacyCarrotVelocityPlannerConfig(
  double model_k_v, double model_k_w, double characteristic_length,
  double cruise_speed, double max_angular_speed, double angular_speed_reserve)
{
    LegacyCarrotVelocityPlannerConfig config;
    config.model_k_v = model_k_v;
    config.model_k_w = model_k_w;
    config.characteristic_length = characteristic_length;
    config.cruise_speed = cruise_speed;
    config.max_angular_speed = max_angular_speed;
    config.angular_speed_reserve = angular_speed_reserve;
    return config;
}

LegacyCarrotVelocityPlan planModelVelocity(
  const Eigen::Vector3d & state,
  const Eigen::Vector3d & desired_state,
  const LegacyCarrotVelocityPlannerConfig & config)
{
    const Eigen::Vector3d error(desired_state - state);
    const double ex_body =
            cos(state[2]) * error[0] + sin(state[2]) * error[1];
    const double ey_body =
            -sin(state[2]) * error[0] + cos(state[2]) * error[1];
    try
    {
        return planLegacyCarrotVelocity(ex_body, ey_body, config);
    }
    catch (const std::exception & exception)
    {
        throw nav2_core::NoValidControl(
                std::string("could not produce a finite model control plan: ") +
                exception.what());
    }
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
    global_plan_.clear();
    prepared_path_.reset();
    holonomic_speed_planner_.reset();
    holonomic_command_limiter_.reset();
    resetRotationState();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    tf_ = std::move(tf);
    declare_parameter_if_not_declared(node, controller_plugin_name + ".goal_dist_tolerance", rclcpp::ParameterValue(0.05) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".rotate_tolerance", rclcpp::ParameterValue(0.5) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".base_frame", rclcpp::ParameterValue("base_link") );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".map_frame", rclcpp::ParameterValue("map") );

    declare_parameter_if_not_declared(node, controller_plugin_name + ".lookahead_time", rclcpp::ParameterValue(0.5) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".min_lookahead_dist", rclcpp::ParameterValue(0.3) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".max_lookahead_dist", rclcpp::ParameterValue(0.9) );

    declare_parameter_if_not_declared(node, controller_plugin_name + ".max_v", rclcpp::ParameterValue(0.7));   // 0.46
    declare_parameter_if_not_declared(node, controller_plugin_name + ".min_v", rclcpp::ParameterValue(0.05) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".max_v_inc", rclcpp::ParameterValue(0.5) );

    declare_parameter_if_not_declared(node, controller_plugin_name + ".max_w", rclcpp::ParameterValue(0.2));   // 1.57
    declare_parameter_if_not_declared(node, controller_plugin_name + ".min_w", rclcpp::ParameterValue(0.0) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".max_w_inc", rclcpp::ParameterValue(0.2) );  // 1.57

    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_v_p", rclcpp::ParameterValue(2.00) );   // 1
    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_v_i", rclcpp::ParameterValue(0.01) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_v_d", rclcpp::ParameterValue(0.10) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_w_p", rclcpp::ParameterValue(1.00) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_w_i", rclcpp::ParameterValue(0.01) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_w_d", rclcpp::ParameterValue(0.10) );

    declare_parameter_if_not_declared(node, controller_plugin_name + ".k_theta", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".model_k_v", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".model_k_w", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, controller_plugin_name + ".l", rclcpp::ParameterValue(0.2) );
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".cruise_speed", rclcpp::ParameterValue(0.51));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".angular_speed_reserve", rclcpp::ParameterValue(0.10));

    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".enable_start_heading_alignment",
      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".start_heading_enter_tolerance",
      rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".start_heading_exit_tolerance",
      rclcpp::ParameterValue(0.15));
    declare_parameter_if_not_declared(
      node, controller_plugin_name + ".start_heading_kp", rclcpp::ParameterValue(1.0));

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

    declare_parameter_if_not_declared(node, controller_plugin_name + ".pid_frequency", rclcpp::ParameterValue(15.0) );
    declare_parameter_if_not_declared(node, controller_plugin_name + ".use_model", rclcpp::ParameterValue(true) );

    node->get_parameter(controller_plugin_name + ".goal_dist_tolerance", goal_dist_tol_);
    node->get_parameter(controller_plugin_name + ".rotate_tolerance", rotate_tol_);
    node->get_parameter(controller_plugin_name + ".base_frame", base_frame_);
    node->get_parameter(controller_plugin_name + ".map_frame", map_frame_);


    node->get_parameter(controller_plugin_name + ".lookahead_time", lookahead_time_);
    node->get_parameter(controller_plugin_name + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(controller_plugin_name + ".max_lookahead_dist", max_lookahead_dist_);
    node->get_parameter(controller_plugin_name + ".max_v", max_v_);
    node->get_parameter(controller_plugin_name + ".min_v", min_v_);
    node->get_parameter(controller_plugin_name + ".max_v_inc", max_v_inc_);

    node->get_parameter(controller_plugin_name + ".max_w", max_w_);
    node->get_parameter(controller_plugin_name + ".min_w", min_w_);
    node->get_parameter(controller_plugin_name + ".max_w_inc", max_w_inc_);

    node->get_parameter(controller_plugin_name + ".k_v_p", k_v_p_);
    node->get_parameter(controller_plugin_name + ".k_v_i", k_v_i_);
    node->get_parameter(controller_plugin_name + ".k_v_d", k_v_d_);
    node->get_parameter(controller_plugin_name + ".k_w_p", k_w_p_);
    node->get_parameter(controller_plugin_name + ".k_w_i", k_w_i_);
    node->get_parameter(controller_plugin_name + ".k_w_d", k_w_d_);

    node->get_parameter(controller_plugin_name + ".k_theta", k_theta_);
    node->get_parameter(controller_plugin_name + ".model_k_v", model_k_v_);
    node->get_parameter(controller_plugin_name + ".model_k_w", model_k_w_);
    node->get_parameter(controller_plugin_name + ".l", l_);
    node->get_parameter(controller_plugin_name + ".cruise_speed", cruise_speed_);
    node->get_parameter(
      controller_plugin_name + ".angular_speed_reserve", angular_speed_reserve_);
    node->get_parameter(controller_plugin_name + ".use_model", use_model_);

    node->get_parameter(
      controller_plugin_name + ".enable_start_heading_alignment",
      enable_start_heading_alignment_);
    node->get_parameter(
      controller_plugin_name + ".start_heading_enter_tolerance",
      start_heading_enter_tolerance_);
    node->get_parameter(
      controller_plugin_name + ".start_heading_exit_tolerance",
      start_heading_exit_tolerance_);
    node->get_parameter(controller_plugin_name + ".start_heading_kp", start_heading_kp_);

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

    double pid_frequency;
    node->get_parameter(controller_plugin_name + ".pid_frequency", pid_frequency);
    d_t_ = 1 / pid_frequency;

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
    speed_config.yaw_feedback_reserve = parameter("yaw_feedback_reserve", 0.05);
    speed_config.configured_min_rate_window = parameter("configured_min_rate_window", 0.30);
    HolonomicCommandLimiterConfig limiter_config;
    limiter_config.max_vx = speed_config.max_vx;
    limiter_config.max_vy = speed_config.max_vy;
    limiter_config.max_xy_speed = speed_config.max_xy_speed;
    limiter_config.max_omega = speed_config.max_omega;
    limiter_config.max_ax = speed_config.max_ax;
    limiter_config.max_ay = speed_config.max_ay;
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
            "rotation_stop_confirm=%.3f rotation_braking_timeout=%.3f",
            speed_config.nominal_tracking_speed, max_xy_speed_, speed_config.max_vx,
            speed_config.max_vy, speed_config.max_omega, speed_config.max_ax,
            speed_config.max_ay, limiter_config.max_alpha, kp_x_, kp_y_, kp_yaw_,
            control_gap_threshold_, entry_config.stop_confirm_time, entry_config.braking_timeout);
    
    costmap_ros_ = std::move(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();
    
    goal_reached_ = false;
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
    goal_x_ = goal_y_ = std::numeric_limits<double>::max();

    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "pid_tracking_controller/carrot", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    tracking_statistics_ = TrackingStatistics();
    plan_revision_ = 0;
    tracking_start_time_valid_ = false;
    force_diagnostics_log_ = true;
    start_heading_check_pending_ = false;
    start_heading_alignment_active_ = false;

    RCLCPP_INFO(
            logger_,
            "PID_LOG common_and_legacy_parameters goal_dist_tolerance=%.6f "
            "rotate_tolerance=%.6f lookahead_time=%.6f "
            "min_lookahead_dist=%.6f max_lookahead_dist=%.6f "
            "max_v=%.6f min_v=%.6f max_v_inc=%.6f "
            "max_w=%.6f min_w=%.6f max_w_inc=%.6f "
            "k_v_p=%.6f k_v_i=%.6f k_v_d=%.6f "
            "k_w_p=%.6f k_w_i=%.6f k_w_d=%.6f "
            "k_theta=%.6f model_k_v=%.6f model_k_w=%.6f l=%.6f "
            "cruise_speed=%.6f angular_speed_reserve=%.6f "
            "enable_start_heading_alignment=%s "
            "start_heading_enter_tolerance=%.6f "
            "start_heading_exit_tolerance=%.6f start_heading_kp=%.6f "
            "kp_yaw=%.6f rotation_yaw_tolerance=%.6f "
            "rotation_omega_tolerance=%.6f rotation_xy_speed_tolerance=%.6f "
            "rotation_settle_time=%.6f "
            "pid_frequency=%.6f use_model=%s",
            goal_dist_tol_, rotate_tol_, lookahead_time_, min_lookahead_dist_,
            max_lookahead_dist_, max_v_, min_v_, max_v_inc_, max_w_, min_w_,
            max_w_inc_, k_v_p_, k_v_i_, k_v_d_, k_w_p_, k_w_i_, k_w_d_,
            k_theta_, model_k_v_, model_k_w_, l_, cruise_speed_,
            angular_speed_reserve_, enable_start_heading_alignment_ ? "true" : "false",
            start_heading_enter_tolerance_, start_heading_exit_tolerance_,
            start_heading_kp_, kp_yaw_, rotation_yaw_tolerance_,
            rotation_omega_tolerance_, rotation_xy_speed_tolerance_,
            rotation_settle_time_, pid_frequency, use_model_ ? "true" : "false");
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
    global_plan_.clear();
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
    global_plan_.clear();
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
    global_plan_ = orig_global_plan.poses;
    ++plan_revision_;
    // receive a plan for a new goal
    if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
    {
        goal_x_ = global_plan_.back().pose.position.x;
        goal_y_ = global_plan_.back().pose.position.y;
        goal_theta_ = getYawAngle(global_plan_.back());
        goal_reached_ = false;
        
        e_v_ = i_v_ = 0.0;
        e_w_ = i_w_ = 0.0;
        start_heading_check_pending_ = enable_start_heading_alignment_;
        start_heading_alignment_active_ = false;
        resetTrackingStatistics();
    }
    std::ostringstream oss_goal;
    oss_goal << std::fixed << std::setprecision(2) << goal_x_ << ", " << goal_y_ << ",  " << goal_theta_;
    // LOG_INFO("PID goal: ({})", oss_goal.str() );
}

void PidTrackingController::resetTrackingStatistics()
{
    tracking_statistics_ = TrackingStatistics();
    tracking_start_time_valid_ = false;
    if (clock_)
    {
        tracking_start_time_seconds_ = clock_->now().seconds();
        tracking_start_time_valid_ = true;
    }
    force_diagnostics_log_ = true;
}

void PidTrackingController::updateAndLogTrackingDiagnostics(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  const geometry_msgs::msg::TwistStamped& command,
  const Eigen::Vector2d& raw_command,
  const LegacyCarrotVelocityPlan& velocity_plan,
  bool alignment_phase,
  double lookahead_distance,
  const PathProgressReference& path_reference,
  const HolonomicTrackingReference * holonomic_reference,
  const HolonomicRawCommand * shadow_rotation_command,
  const RotationCompletionStatus * shadow_rotation_status)
{
    const double lateral_error = path_reference.signed_lateral_error;
    const double absolute_lateral_error = std::abs(lateral_error);
    const double actual_planar_speed = std::hypot(velocity.linear.x, velocity.linear.y);
    const double linear_speed_tracking_error =
            command.twist.linear.x - actual_planar_speed;
    ++tracking_statistics_.sample_count;
    tracking_statistics_.sum_squared_lateral_error += lateral_error * lateral_error;
    tracking_statistics_.maximum_absolute_lateral_error = std::max(
            tracking_statistics_.maximum_absolute_lateral_error, absolute_lateral_error);
    tracking_statistics_.absolute_lateral_error_samples.push_back(absolute_lateral_error);
    tracking_statistics_.sum_actual_planar_speed += actual_planar_speed;
    if (tracking_statistics_.previous_position_valid)
    {
        tracking_statistics_.traveled_distance += std::hypot(
                pose.pose.position.x - tracking_statistics_.previous_position.x,
                pose.pose.position.y - tracking_statistics_.previous_position.y);
    }
    tracking_statistics_.previous_position = pose.pose.position;
    tracking_statistics_.previous_position_valid = true;
    if (isNearAbsoluteLimit(command.twist.linear.x, max_v_))
    {
        ++tracking_statistics_.linear_saturation_count;
    }
    if (isNearAbsoluteLimit(command.twist.angular.z, max_w_))
    {
        ++tracking_statistics_.angular_saturation_count;
    }
    if (alignment_phase)
    {
        ++tracking_statistics_.alignment_sample_count;
        if (isNearAbsoluteLimit(command.twist.angular.z, max_w_))
        {
            ++tracking_statistics_.alignment_angular_saturation_count;
        }
    }
    else
    {
        ++tracking_statistics_.tracking_sample_count;
        tracking_statistics_.tracking_sum_squared_lateral_error +=
                lateral_error * lateral_error;
        tracking_statistics_.tracking_maximum_absolute_lateral_error = std::max(
                tracking_statistics_.tracking_maximum_absolute_lateral_error,
                absolute_lateral_error);
        tracking_statistics_.tracking_absolute_lateral_error_samples.push_back(
                absolute_lateral_error);
        tracking_statistics_.tracking_sum_squared_linear_speed_error +=
                linear_speed_tracking_error * linear_speed_tracking_error;
        if (isNearAbsoluteLimit(command.twist.linear.x, max_v_))
        {
            ++tracking_statistics_.tracking_linear_saturation_count;
        }
        if (isNearAbsoluteLimit(command.twist.angular.z, max_w_))
        {
            ++tracking_statistics_.tracking_angular_saturation_count;
        }
        if (std::abs(raw_command.y()) > max_w_)
        {
            ++tracking_statistics_.tracking_raw_angular_saturation_count;
        }
        if (velocity_plan.speed_limit_reason == SpeedLimitReason::kAngular)
        {
            ++tracking_statistics_.tracking_planner_limit_active_count;
        }
    }

    const double now_seconds = clock_->now().seconds();
    if (!tracking_start_time_valid_)
    {
        tracking_start_time_seconds_ = now_seconds;
        tracking_start_time_valid_ = true;
    }
    if (!force_diagnostics_log_ && now_seconds >= last_diagnostics_log_time_seconds_ &&
        now_seconds - last_diagnostics_log_time_seconds_ < 1.0)
    {
        return;
    }

    auto samples = tracking_statistics_.absolute_lateral_error_samples;
    const std::size_t p95_index =
            static_cast<std::size_t>(std::ceil(0.95 * samples.size())) - 1u;
    std::nth_element(samples.begin(), samples.begin() + p95_index, samples.end());
    const double p95 = samples[p95_index];
    const double sample_count = static_cast<double>(tracking_statistics_.sample_count);
    const double lateral_error_rms = std::sqrt(
            tracking_statistics_.sum_squared_lateral_error / sample_count);
    const double average_actual_planar_speed =
            tracking_statistics_.sum_actual_planar_speed / sample_count;
    const double linear_saturation_ratio =
            static_cast<double>(tracking_statistics_.linear_saturation_count) / sample_count;
    const double angular_saturation_ratio =
            static_cast<double>(tracking_statistics_.angular_saturation_count) / sample_count;
    double tracking_lateral_error_rms = 0.0;
    double tracking_lateral_error_p95 = 0.0;
    double tracking_linear_speed_error_rms = 0.0;
    if (tracking_statistics_.tracking_sample_count > 0u)
    {
        tracking_lateral_error_rms = std::sqrt(
                tracking_statistics_.tracking_sum_squared_lateral_error /
                static_cast<double>(tracking_statistics_.tracking_sample_count));
        tracking_linear_speed_error_rms = std::sqrt(
                tracking_statistics_.tracking_sum_squared_linear_speed_error /
                static_cast<double>(tracking_statistics_.tracking_sample_count));
        auto tracking_samples = tracking_statistics_.tracking_absolute_lateral_error_samples;
        const std::size_t tracking_p95_index =
                static_cast<std::size_t>(std::ceil(0.95 * tracking_samples.size())) - 1u;
        std::nth_element(
                tracking_samples.begin(), tracking_samples.begin() + tracking_p95_index,
                tracking_samples.end());
        tracking_lateral_error_p95 = tracking_samples[tracking_p95_index];
    }
    const auto ratio = [](std::size_t count, std::size_t total) {
        return total == 0u ? 0.0 : static_cast<double>(count) / static_cast<double>(total);
    };
    const double tracking_linear_saturation_ratio = ratio(
            tracking_statistics_.tracking_linear_saturation_count,
            tracking_statistics_.tracking_sample_count);
    const double tracking_angular_saturation_ratio = ratio(
            tracking_statistics_.tracking_angular_saturation_count,
            tracking_statistics_.tracking_sample_count);
    const double alignment_angular_saturation_ratio = ratio(
            tracking_statistics_.alignment_angular_saturation_count,
            tracking_statistics_.alignment_sample_count);
    const double tracking_raw_angular_saturation_ratio = ratio(
            tracking_statistics_.tracking_raw_angular_saturation_count,
            tracking_statistics_.tracking_sample_count);
    const double planner_limit_active_ratio = ratio(
            tracking_statistics_.tracking_planner_limit_active_count,
            tracking_statistics_.tracking_sample_count);
    const double elapsed_tracking_time = std::max(
            0.0, now_seconds - tracking_start_time_seconds_);
    const HolonomicTrackingReference unavailable_reference;
    const auto & shadow_reference = holonomic_reference ?
            *holonomic_reference : unavailable_reference;
    const char * reference_mode = holonomic_reference ?
            holonomicReferenceModeName(shadow_reference.mode) : "unavailable";
    const HolonomicRawCommand unavailable_shadow_command;
    const auto & shadow_command = shadow_rotation_command ?
            *shadow_rotation_command : unavailable_shadow_command;
    const RotationCompletionStatus unavailable_rotation_status;
    const auto & rotation_status = shadow_rotation_status ?
            *shadow_rotation_status : unavailable_rotation_status;
    const std::string pending_rotation_segment_index =
            shadow_reference.pending_rotation_segment_index ?
            std::to_string(*shadow_reference.pending_rotation_segment_index) : "none";

    RCLCPP_INFO(
            logger_,
            "PID_LOG plan_revision=%zu phase=%s sample_count=%zu "
            "tracking_sample_count=%zu alignment_sample_count=%zu lateral_error=%.6f "
            "lateral_error_rms=%.6f lateral_error_p95_abs=%.6f "
            "lateral_error_max_abs=%.6f tracking_lateral_error_rms=%.6f "
            "tracking_lateral_error_p95_abs=%.6f "
            "tracking_lateral_error_max_abs=%.6f traveled_distance=%.6f "
            "average_actual_planar_speed=%.6f commanded_linear_speed=%.6f "
            "actual_linear_speed=%.6f linear_speed_tracking_error=%.6f "
            "tracking_linear_speed_error_rms=%.6f commanded_angular_speed=%.6f "
            "actual_angular_speed=%.6f linear_saturation_ratio=%.6f "
            "angular_saturation_ratio=%.6f raw_linear_command=%.6f "
            "raw_angular_command=%.6f tracking_linear_saturation_ratio=%.6f "
            "tracking_angular_saturation_ratio=%.6f "
            "alignment_angular_saturation_ratio=%.6f "
            "tracking_raw_angular_saturation_ratio=%.6f "
            "planner_limit_active_ratio=%.6f carrot_alpha=%.6f "
            "carrot_alpha_deg=%.6f carrot_sin_alpha=%.6f carrot_cos_alpha=%.6f "
            "steering_gain_ratio=%.6f cruise_speed=%.6f "
            "angular_planning_limit=%.6f angular_limited_speed=%.6f "
            "reference_speed=%.6f speed_limit_reason=%s "
            "required_angular_at_cruise=%.6f lookahead_distance=%.6f "
            "theta_trj=%.6f kappa=%.6f carrot_x=%.6f carrot_y=%.6f "
            "progress_arc_length=%.6f carrot_arc_length=%.6f "
            "segment_index=%zu segment_fraction=%.6f "
            "remaining_path_length=%.6f progress_budget=%.6f "
            "reference_mode=%s current_segment_index=%zu "
            "current_segment_fraction=%.6f current_ref_x=%.6f current_ref_y=%.6f "
            "current_path_tangent=%.6f current_target_yaw=%.6f "
            "current_target_yaw_rate_per_meter=%.6f preview_segment_index=%zu "
            "preview_segment_fraction=%.6f preview_ref_x=%.6f preview_ref_y=%.6f "
            "preview_path_tangent=%.6f preview_target_yaw=%.6f "
            "preview_clamped_by_rotation=%s rotation_target_yaw=%.6f "
            "shadow_rotation_available=%s pending_rotation_segment_index=%s "
            "shadow_yaw_error=%.6f shadow_raw_vx=%.6f shadow_raw_vy=%.6f "
            "shadow_raw_omega=%.6f shadow_measured_vx=%.6f "
            "shadow_measured_vy=%.6f shadow_measured_omega=%.6f "
            "shadow_settled_duration=%.6f shadow_rotation_all_conditions=%s "
            "rotation_ack_candidate=%s "
            "elapsed_tracking_time=%.6f",
            plan_revision_, alignment_phase ? "START_ALIGNMENT" : "TRACKING",
            tracking_statistics_.sample_count, tracking_statistics_.tracking_sample_count,
            tracking_statistics_.alignment_sample_count, lateral_error,
            lateral_error_rms, p95, tracking_statistics_.maximum_absolute_lateral_error,
            tracking_lateral_error_rms, tracking_lateral_error_p95,
            tracking_statistics_.tracking_maximum_absolute_lateral_error,
            tracking_statistics_.traveled_distance, average_actual_planar_speed,
            command.twist.linear.x, actual_planar_speed, linear_speed_tracking_error,
            tracking_linear_speed_error_rms, command.twist.angular.z,
            velocity.angular.z, linear_saturation_ratio, angular_saturation_ratio,
            raw_command.x(), raw_command.y(), tracking_linear_saturation_ratio,
            tracking_angular_saturation_ratio, alignment_angular_saturation_ratio,
            tracking_raw_angular_saturation_ratio, planner_limit_active_ratio,
            velocity_plan.alpha, velocity_plan.alpha * 180.0 / std::acos(-1.0),
            velocity_plan.sin_alpha, velocity_plan.cos_alpha,
            velocity_plan.steering_gain_ratio, velocity_plan.cruise_speed,
            velocity_plan.angular_planning_limit,
            velocity_plan.angular_limited_speed, velocity_plan.reference_speed,
            speedLimitReasonName(velocity_plan.speed_limit_reason),
            velocity_plan.required_angular_at_cruise,
            lookahead_distance, path_reference.tangent, path_reference.curvature,
            path_reference.carrot.x, path_reference.carrot.y,
            path_reference.progress_arc_length, path_reference.carrot_arc_length,
            path_reference.segment_index, path_reference.segment_fraction,
            path_reference.remaining_path_length, path_reference.progress_budget,
            reference_mode, shadow_reference.current.cursor.segment_index,
            shadow_reference.current.cursor.segment_fraction,
            shadow_reference.current.position.x, shadow_reference.current.position.y,
            shadow_reference.current.path_tangent, shadow_reference.current.target_yaw,
            shadow_reference.current.target_yaw_rate_per_meter,
            shadow_reference.preview.cursor.segment_index,
            shadow_reference.preview.cursor.segment_fraction,
            shadow_reference.preview.position.x, shadow_reference.preview.position.y,
            shadow_reference.preview.path_tangent, shadow_reference.preview.target_yaw,
            shadow_reference.preview_clamped_by_rotation ? "true" : "false",
            shadow_reference.rotation_target_yaw,
            shadow_rotation_command && shadow_rotation_status ? "true" : "false",
            pending_rotation_segment_index.c_str(), shadow_command.yaw_error,
            shadow_command.vx, shadow_command.vy, shadow_command.omega,
            velocity.linear.x, velocity.linear.y, velocity.angular.z,
            rotation_status.settled_duration,
            rotation_status.all_conditions ? "true" : "false",
            rotation_status.acknowledge_candidate ? "true" : "false",
            elapsed_tracking_time);
    last_diagnostics_log_time_seconds_ = now_seconds;
    force_diagnostics_log_ = false;
}

bool PidTrackingController::isGoalReached()
{
    if (goal_reached_)
    {
        RCLCPP_INFO(logger_, "PidTrackingController goal reached");
        return true;
    }
    return false;
}

double PidTrackingController::getYawAngle(geometry_msgs::msg::PoseStamped& ps)
{
    tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
    double roll(0.0), pitch(0.0), yaw(0.0);
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
}

/**
 * @brief Whether to reach the target pose through rotation operation
 * @param cur  current pose of robot
 * @param goal goal pose of robot
 * @return true if robot should perform rotation
 */
bool PidTrackingController::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& cur, 
                                               const geometry_msgs::msg::PoseStamped& goal)
{
    return std::hypot(cur.pose.position.x - goal.pose.position.x, cur.pose.position.y - goal.pose.position.y) <
            goal_dist_tol_;
}

/**
 * @brief Whether to correct the tracking path with rotation operation
 * @param angle_to_path the angle deviation
 * @return true if robot should perform rotation
 */
bool PidTrackingController::shouldRotateToPath(double angle_to_path, double tolerance)
{
    return (tolerance && (angle_to_path > tolerance)) || (!tolerance && (angle_to_path > rotate_tol_));
}

/**
 * @brief linear velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param v_d           desired velocity magnitude
 * @return v            regulated linear velocity
 */
double PidTrackingController::linearRegularization(
  const geometry_msgs::msg::Twist & measured_twist, double v_d)
{
    std::ostringstream oss_v;
    
    double v = std::hypot(measured_twist.linear.x, measured_twist.linear.y);
    double v_inc = v_d - v;
    oss_v << std::fixed << std::setprecision(2) << "v: " << v << ", " <<"v_d: " << v_d << ", " << "v_inc: " << v_inc;
    if (std::fabs(v_inc) > max_v_inc_)
        v_inc = std::copysign(max_v_inc_, v_inc);
    
    double v_cmd = v + v_inc;
    oss_v << std::fixed << std::setprecision(2) <<", v_cmd: " << v_cmd;
    // LOG_INFO("linear Regularization: {}", oss_v.str() );

    if (std::fabs(v_cmd) > max_v_)
        v_cmd = std::copysign(max_v_, v_cmd);
    else if (std::fabs(v_cmd) < min_v_)
        v_cmd = std::copysign(min_v_, v_cmd);
    
    return v_cmd;
}

/**
 * @brief angular velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param w_d           desired angular velocity
 * @return w            regulated angular velocity
 */
double PidTrackingController::angularRegularization(
  const geometry_msgs::msg::Twist & measured_twist, double w_d)
{
    if (std::fabs(w_d) > max_w_)
        w_d = std::copysign(max_w_, w_d);
    
    double w = measured_twist.angular.z;
    double w_inc = w_d - w;
    // LOG_INFO("w_d:{}, w_inc: {}", w_inc);
    if (std::fabs(w_inc) > max_w_inc_)
        w_inc = std::copysign(max_w_inc_, w_inc);
    
    double w_cmd = w + w_inc;
    // LOG_INFO("11111111111111111  w: {}, w_inc: {},  w_cmd: {}", w, w_inc, w_cmd);
    if (std::fabs(w_cmd) > max_w_)
        w_cmd = std::copysign(max_w_, w_cmd);
    else if (std::fabs(w_cmd) < min_w_)
        w_cmd = std::copysign(min_w_, w_cmd);
    
    return w_cmd;
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
        speed_plan = holonomic_speed_planner_->plan(reference, yaw, dt, cap);
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
            reseed ? 0.0 : dt, cap);
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
                "rotation_settled=%.6f ack_candidate=%s",
                plan_revision_, holonomicReferenceModeName(reference.mode), reseed ? "false" : "true",
                reseed ? std::numeric_limits<double>::quiet_NaN() : speed_plan.reference_velocity_target,
                speed_plan.reference_velocity,
                reseed ? "not_planned" : holonomicLimitReasonName(speed_plan.speed_limit_reason), cap,
                raw.vx, raw.vy, raw.omega, limited.command.vx, limited.command.vy, limited.command.omega,
                limited.translation_lambda, reseed_reason, gap ? "true" : "false",
                rotating ? std::to_string(pending_rotation_segment_index_).c_str() : "none",
                rotating ? (braking ? "braking" : "rotating") : "none",
                rotation_entry_gate_->stoppedDuration(),
                completion.settled_duration, rotation_ack_candidate_ ? "true" : "false");
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
        if (global_plan_.empty() || !prepared_path_) {
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

/**
 * @brief Execute PID control process (no model pid)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PidTrackingController::NoModelPidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
    Eigen::Vector2d u;
    Eigen::Vector3d e = s_d - s;
    
    double e_x = e[0];
    double e_y = e[1];
    double e_theta = e[2];
    
    double v_d = std::hypot(e_x, e_y) / d_t_;
    double w_d = e_theta / d_t_;
    
    if (std::fabs(v_d) > max_v_)
        v_d = std::copysign(max_v_, v_d);
    if (std::fabs(w_d) > max_w_)
        w_d = std::copysign(max_w_, w_d);
    
    double e_v = v_d - u_r[0];
    double e_w = w_d - u_r[1];
    
    i_v_ += e_v * d_t_;
    i_w_ += e_w * d_t_;
    
    double d_v = (e_v - e_v_) / d_t_;
    double d_w = (e_w - e_w_) / d_t_;
    
    e_v_ = e_v;
    e_w_ = e_w;
    
    double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;
    double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;
    
    if (std::fabs(v_inc) > max_v_inc_)
        v_inc = std::copysign(max_v_inc_, v_inc);
    if (std::fabs(w_inc) > max_w_inc_)
        w_inc = std::copysign(max_w_inc_, w_inc);
    
    double v_cmd = u_r[0] + v_inc;
    if (std::fabs(v_cmd) > max_v_)
        v_cmd = std::copysign(max_v_, v_cmd);
    else if (std::fabs(v_cmd) < min_v_)
        v_cmd = std::copysign(min_v_, v_cmd);
    
    double w_cmd = u_r[1] + w_inc;
    if (std::fabs(w_cmd) > max_w_)
        w_cmd = std::copysign(max_w_, w_cmd);
    else if (std::fabs(w_cmd) < min_w_)
        w_cmd = std::copysign(min_w_, w_cmd);
    
    u[0] = v_cmd;
    u[1] = w_cmd;
    
    return u;
}

/**
 * @brief Execute PID control process (with model)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PidTrackingController::UseModelPidControl(Eigen::Vector3d  s, 
                                                          Eigen::Vector3d  s_d, 
                                                          Eigen::Vector2d  /*u_r*/)
{
    const auto plan = planModelVelocity(
            s, s_d, makeLegacyCarrotVelocityPlannerConfig(
                    model_k_v_, model_k_w_, l_, cruise_speed_, max_w_,
                    angular_speed_reserve_));
    Eigen::Vector2d u(plan.raw_linear_command, plan.raw_angular_command);
    return u;
}

}  // namespace pid_tracking_controller

PLUGINLIB_EXPORT_CLASS(
  pid_tracking_controller::PidTrackingController,
  nav2_core::Controller)
