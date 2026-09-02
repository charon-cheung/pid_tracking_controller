/*
 *	Copyright (C), suzhou, muhai
 *	@author:	ZZP
 *	@date:		2025/11/03
 *	@note:
 */
#include "pid_tracking_controller/pid_tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "common/geometry/vec2d.h"
#include "common/math/math_helper.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
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

bool eigenVectorIsFinite(const Eigen::Vector2d& vector)
{
    return vector.allFinite();
}

}  // namespace

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

    double pid_frequency;
    node->get_parameter(controller_plugin_name + ".pid_frequency", pid_frequency);
    d_t_ = 1 / pid_frequency;
    
    node->get_parameter(controller_plugin_name + ".use_model", use_model_);
    
    costmap_ros_ = std::move(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();
    
    goal_reached_ = false;
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
    goal_x_ = goal_y_ = std::numeric_limits<double>::max();

    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pid_tracking_controller/carrot", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    tracking_statistics_ = TrackingStatistics();
    plan_revision_ = 0;
    tracking_start_time_valid_ = false;
    force_diagnostics_log_ = true;
    start_heading_check_pending_ = false;
    start_heading_alignment_active_ = false;
}

void PidTrackingController::activate()
{
    if (carrot_pub_) {
        carrot_pub_->on_activate();
    }
    force_diagnostics_log_ = true;
}

void PidTrackingController::deactivate()
{
    if (carrot_pub_) {
        carrot_pub_->on_deactivate();
    }
}

void PidTrackingController::cleanup()
{
    carrot_pub_.reset();
    costmap_ = nullptr;
    costmap_ros_.reset();
    tf_.reset();
    clock_.reset();
}

// 先执行 setPlan，再执行旋转，转到位后，再执行 computeVelocityCommands
void PidTrackingController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
    RCLCPP_INFO(logger_, "PidTrackingController received a path");
    global_plan_.clear();
    if (orig_global_plan.poses.empty() || !pathIsFinite(orig_global_plan))
    {
        throw nav2_core::InvalidPath(
                controller_plugin_name + " received an empty or non-finite path");
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
  bool alignment_phase,
  const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan,
  double lookahead_distance,
  double theta_trj,
  double kappa,
  const geometry_msgs::msg::PointStamped& carrot)
{
    double lateral_error = 0.0;
    bool segment_found = false;
    for (std::size_t index = 0; index + 1 < prune_plan.size(); ++index)
    {
        const auto& start = prune_plan[index].pose.position;
        const auto& end = prune_plan[index + 1].pose.position;
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared <= rmp::common::math::kMathEpsilon)
        {
            continue;
        }

        const double relative_x = pose.pose.position.x - start.x;
        const double relative_y = pose.pose.position.y - start.y;
        const double projection = std::clamp(
                (relative_x * dx + relative_y * dy) / length_squared, 0.0, 1.0);
        const double projected_x = start.x + projection * dx;
        const double projected_y = start.y + projection * dy;
        const double distance = std::hypot(
                pose.pose.position.x - projected_x, pose.pose.position.y - projected_y);
        const double cross = dx * relative_y - dy * relative_x;
        lateral_error = cross < 0.0 ? -distance : distance;
        segment_found = true;
        break;
    }
    if (!segment_found)
    {
        lateral_error = std::hypot(
                pose.pose.position.x - prune_plan.front().pose.position.x,
                pose.pose.position.y - prune_plan.front().pose.position.y);
    }

    const double absolute_lateral_error = std::abs(lateral_error);
    const double actual_planar_speed = std::hypot(velocity.linear.x, velocity.linear.y);
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
    if (std::abs(command.twist.linear.x) >= max_v_ - rmp::common::math::kMathEpsilon)
    {
        ++tracking_statistics_.linear_saturation_count;
    }
    if (std::abs(command.twist.angular.z) >= max_w_ - rmp::common::math::kMathEpsilon)
    {
        ++tracking_statistics_.angular_saturation_count;
    }
    if (alignment_phase)
    {
        ++tracking_statistics_.alignment_sample_count;
        if (std::abs(command.twist.angular.z) >= max_w_ - rmp::common::math::kMathEpsilon)
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
        if (std::abs(command.twist.linear.x) >= max_v_ - rmp::common::math::kMathEpsilon)
        {
            ++tracking_statistics_.tracking_linear_saturation_count;
        }
        if (std::abs(command.twist.angular.z) >= max_w_ - rmp::common::math::kMathEpsilon)
        {
            ++tracking_statistics_.tracking_angular_saturation_count;
        }
        if (std::abs(raw_command.y()) > max_w_)
        {
            ++tracking_statistics_.tracking_raw_angular_saturation_count;
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
    if (tracking_statistics_.tracking_sample_count > 0u)
    {
        tracking_lateral_error_rms = std::sqrt(
                tracking_statistics_.tracking_sum_squared_lateral_error /
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
    const double elapsed_tracking_time = std::max(
            0.0, now_seconds - tracking_start_time_seconds_);

    RCLCPP_INFO(
            logger_,
            "PID_LOG plan_revision=%zu phase=%s sample_count=%zu "
            "tracking_sample_count=%zu alignment_sample_count=%zu lateral_error=%.6f "
            "lateral_error_rms=%.6f lateral_error_p95_abs=%.6f "
            "lateral_error_max_abs=%.6f tracking_lateral_error_rms=%.6f "
            "tracking_lateral_error_p95_abs=%.6f "
            "tracking_lateral_error_max_abs=%.6f traveled_distance=%.6f "
            "average_actual_planar_speed=%.6f commanded_linear_speed=%.6f "
            "actual_linear_speed=%.6f commanded_angular_speed=%.6f "
            "actual_angular_speed=%.6f linear_saturation_ratio=%.6f "
            "angular_saturation_ratio=%.6f raw_linear_command=%.6f "
            "raw_angular_command=%.6f tracking_linear_saturation_ratio=%.6f "
            "tracking_angular_saturation_ratio=%.6f "
            "alignment_angular_saturation_ratio=%.6f "
            "tracking_raw_angular_saturation_ratio=%.6f lookahead_distance=%.6f "
            "theta_trj=%.6f kappa=%.6f carrot_x=%.6f carrot_y=%.6f "
            "elapsed_tracking_time=%.6f",
            plan_revision_, alignment_phase ? "START_ALIGNMENT" : "TRACKING",
            tracking_statistics_.sample_count, tracking_statistics_.tracking_sample_count,
            tracking_statistics_.alignment_sample_count, lateral_error,
            lateral_error_rms, p95, tracking_statistics_.maximum_absolute_lateral_error,
            tracking_lateral_error_rms, tracking_lateral_error_p95,
            tracking_statistics_.tracking_maximum_absolute_lateral_error,
            tracking_statistics_.traveled_distance, average_actual_planar_speed,
            command.twist.linear.x, actual_planar_speed, command.twist.angular.z,
            velocity.angular.z, linear_saturation_ratio, angular_saturation_ratio,
            raw_command.x(), raw_command.y(), tracking_linear_saturation_ratio,
            tracking_angular_saturation_ratio, alignment_angular_saturation_ratio,
            tracking_raw_angular_saturation_ratio,
            lookahead_distance, theta_trj, kappa, carrot.point.x, carrot.point.y,
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

    return rmp::common::math::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

void PidTrackingController::setSpeedLimit(const double &, const bool &)
{
    
}

/**
 * @brief find the point on the path that is exactly the lookahead distance away from the robot
 * @param lookahead_dist    the lookahead distance
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 * @param pt                the lookahead point
 * @param theta             the angle on traj
 * @param kappa             the curvature on traj
 */
void PidTrackingController::getLookAheadPoint(double lookahead_dist, geometry_msgs::msg::PoseStamped robot_pose_global,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan,
                                              geometry_msgs::msg::PointStamped& pt, double& theta, double& kappa)
{
    if (prune_plan.empty())
    {
        throw nav2_core::InvalidPath(controller_plugin_name + " has no path for carrot selection");
    }

    double rx = robot_pose_global.pose.position.x;
    double ry = robot_pose_global.pose.position.y;
    
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(prune_plan.begin(), prune_plan.end(), [&](const geometry_msgs::msg::PoseStamped& ps) {
        return std::hypot(ps.pose.position.x - robot_pose_global.pose.position.x,
                          ps.pose.position.y - robot_pose_global.pose.position.y) >= lookahead_dist;
    });
    
    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == prune_plan.end())
    {
        goal_pose_it = std::prev(prune_plan.end());
        pt.point.x = goal_pose_it->pose.position.x;
        pt.point.y = goal_pose_it->pose.position.y;
        kappa = 0;
        theta = atan2(pt.point.y - ry, pt.point.x - rx);
    }
    else
    {
        // find the point on the line segment between the two poses
        // that is exactly the lookahead distance away from the robot pose (the origin)
        // This can be found with a closed form for the intersection of a segment and a circle
        // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
        // and goal_pose is guaranteed to be outside the circle.
        double px, py;
        double gx = goal_pose_it->pose.position.x;
        double gy = goal_pose_it->pose.position.y;
        if (goal_pose_it == prune_plan.begin())
        {
            px = rx;
            py = ry;
        }
        else
        {
            auto prev_pose_it = std::prev(goal_pose_it);
            px = prev_pose_it->pose.position.x;
            py = prev_pose_it->pose.position.y;
        }
        
        // transform to the robot frame so that the circle centers at (0,0)
        rmp::common::geometry::Vec2d prev_p(px - rx, py - ry);
        rmp::common::geometry::Vec2d goal_p(gx - rx, gy - ry);
        
        std::vector<rmp::common::geometry::Vec2d> i_points =
                rmp::common::math::circleSegmentIntersection(prev_p, goal_p, lookahead_dist);
        if (i_points.empty())
        {
            throw nav2_core::NoValidControl(
                    controller_plugin_name + " could not compute a finite lookahead carrot");
        }
        
        double dist_to_goal = std::numeric_limits<double>::max();
        for (const auto& i_point : i_points)
        {
            double dist = std::hypot(i_point.x() + rx - gx, i_point.y() + ry - gy);
            if (dist < dist_to_goal)
            {
                dist_to_goal = dist;
                pt.point.x = i_point.x() + rx;
                pt.point.y = i_point.y() + ry;
            }
        }
        if (dist_to_goal == std::numeric_limits<double>::max())
        {
            throw nav2_core::NoValidControl(
                    controller_plugin_name + " could not select a finite lookahead carrot");
        }
        
        auto next_pose_it = std::next(goal_pose_it);
        if (next_pose_it != prune_plan.end())
        {
            rmp::common::geometry::Vec2d p1(px, py), p2(gx, gy),
                    p3(next_pose_it->pose.position.x, next_pose_it->pose.position.y);
            kappa = rmp::common::math::arcCenter(p1, p2, p3, false);
        }
        else
        {
            kappa = 0.0;
        }
        
        theta = atan2(gy - py, gx - px);
    }
    pt.header.frame_id = goal_pose_it->header.frame_id;
    pt.header.stamp = goal_pose_it->header.stamp;
}


/**
 * @brief Prune the path, removing the waypoints that the robot has already passed and distant waypoints
 * @param robot_pose_global the robot's pose  [global]
 * @return pruned path
 */
std::vector<geometry_msgs::msg::PoseStamped> PidTrackingController::prune(const geometry_msgs::msg::PoseStamped robot_pose_global)
{
    auto calPoseDistance = [](const geometry_msgs::msg::PoseStamped& ps_1, const geometry_msgs::msg::PoseStamped& ps_2) {
        return std::hypot(ps_1.pose.position.x - ps_2.pose.position.x, ps_1.pose.position.y - ps_2.pose.position.y);
    };
    
    auto firstIntegratedDistance =
            [](std::vector<geometry_msgs::msg::PoseStamped>::iterator begin, std::vector<geometry_msgs::msg::PoseStamped>::iterator end,
            std::function<double(const geometry_msgs::msg::PoseStamped&, const geometry_msgs::msg::PoseStamped&)> dist,
            double cmp_val) {
        if (begin == end)
            return end;
        double d = 0.0;
        for (auto it = begin; it != end - 1; it++)
        {
            d += dist(*it, *(it + 1));
            if (d > cmp_val)
                return it + 1;
        }
        return end;
    };
    
    auto getMinFuncVal = [](std::vector<geometry_msgs::msg::PoseStamped>::iterator begin,
            std::vector<geometry_msgs::msg::PoseStamped>::iterator end,
            std::function<double(const geometry_msgs::msg::PoseStamped&)> cal)
    {
        if (begin == end)
            return end;
        
        auto min_val = cal(*begin);
        auto min_iter = begin;
        for (auto it = ++begin; it != end; it++)
        {
            auto val = cal(*it);
            if (val <= min_val)
            {
                min_val = val;
                min_iter = it;
            }
        }
        return min_iter;
    };
    auto closest_pose_upper_bound = firstIntegratedDistance(global_plan_.begin(), global_plan_.end(), calPoseDistance,
                                                            costmap_->getSizeInMetersX() / 2.0);
    // find the closest pose on the path to the robot
    auto transform_begin =
            getMinFuncVal(global_plan_.begin(), closest_pose_upper_bound,
                          [&](const geometry_msgs::msg::PoseStamped& ps) { return calPoseDistance(robot_pose_global, ps); });
    // Transform the near part of the global plan into the robot's frame of reference.
    std::vector<geometry_msgs::msg::PoseStamped> prune_path;
    for (auto it = transform_begin; it < global_plan_.end(); it++)
        prune_path.push_back(*it);
    
    // path pruning: remove the portion of the global plan that already passed so don't process it on the next iteration
    global_plan_.erase(std::begin(global_plan_), transform_begin);
    
    return prune_path;
}


geometry_msgs::msg::TwistStamped PidTrackingController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                                const geometry_msgs::msg::Twist& velocity,
                                                                                nav2_core::GoalChecker* /*goal_checker*/)
{
    if (!poseIsFinite(pose) || !twistIsFinite(velocity))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " received a non-finite pose or velocity");
    }
    if (global_plan_.empty())
    {
        throw nav2_core::InvalidPath(controller_plugin_name + " has no path to track");
    }
    if (costmap_ == nullptr || !clock_)
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " is not configured for control");
    }

    std::ostringstream oss_pose;
    oss_pose << std::fixed << std::setprecision(2) << pose.pose.position.x << ", " << pose.pose.position.y;
    // LOG_INFO("current pose: ({})", oss_pose.str() );
    
    // LOG_INFO("PID input_linear: {}, input_angular: {}", velocity.linear.x, velocity.angular.z);
    geometry_msgs::msg::TwistStamped cmd_vel;
    
    // calculate look-ahead distance
    double vt = std::hypot(velocity.linear.x, velocity.linear.y);
    double wt = velocity.angular.z;
    
    geometry_msgs::msg::PoseStamped current_ps_map = pose;
    std::vector<geometry_msgs::msg::PoseStamped> prune_plan = prune(current_ps_map);
    if (prune_plan.empty())
    {
        throw nav2_core::InvalidPath(controller_plugin_name + " produced an empty pruned path");
    }
    
    double L = getLookAheadDistance(vt);
    if (!scalarIsFinite(L))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " produced a non-finite lookahead distance");
    }
    
    // get the particular point on the path at the lookahead distance
    geometry_msgs::msg::PointStamped lookahead_pt;
    double theta_d, theta_dir, theta_trj, kappa;
    getLookAheadPoint(L, current_ps_map, prune_plan, lookahead_pt, theta_trj, kappa);
    if (!pointIsFinite(lookahead_pt.point) || !scalarIsFinite(theta_trj) ||
        !scalarIsFinite(kappa))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " produced a non-finite lookahead reference");
    }
    
    geometry_msgs::msg::PoseStamped target_ps_map;
    target_ps_map.pose.position.x = lookahead_pt.point.x;
    target_ps_map.pose.position.y = lookahead_pt.point.y;
    theta_dir = atan2((target_ps_map.pose.position.y - current_ps_map.pose.position.y),
                      (target_ps_map.pose.position.x - current_ps_map.pose.position.x));
    if (!scalarIsFinite(theta_dir))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " produced a non-finite target direction");
    }

    theta_d = angles::normalize_angle((1 - k_theta_) * theta_trj + k_theta_ * theta_dir);
    if (!scalarIsFinite(theta_d))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " produced a non-finite target heading");
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_d);
    tf2::convert(q, target_ps_map.pose.orientation);
    
    double theta = tf2::getYaw(current_ps_map.pose.orientation);
    if (!scalarIsFinite(theta))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " could not compute a finite robot heading");
    }
    Eigen::Vector2d raw_command = Eigen::Vector2d::Zero();
    
#if 0
    // position reached
    if (shouldRotateToGoal(current_ps_map, global_plan_.back()))
    {
        LOG_INFO("1111111111111111111111111");
        double e_theta = nav2_util::geometry_utils::normalize_angle(goal_theta_ - theta);
        
        // orientation reached
        if (!shouldRotateToPath(std::fabs(e_theta), 0.06))
        {
            LOG_INFO("333333333333333333333333333");
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            goal_reached_ = true;
        }
        // orientation not reached
        else
        {
            LOG_INFO("4444444444444444444444444444444");
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = angularRegularization(velocity, e_theta / d_t_);
        }
    }
    // posistion not reached
    else
#endif
    {
        if (start_heading_check_pending_)
        {
            start_heading_check_pending_ = false;
            const double carrot_distance = std::hypot(
                    target_ps_map.pose.position.x - current_ps_map.pose.position.x,
                    target_ps_map.pose.position.y - current_ps_map.pose.position.y);
            if (carrot_distance > rmp::common::math::kMathEpsilon)
            {
                start_heading_target_ = theta_dir;
                const double start_heading_error = angles::normalize_angle(
                        start_heading_target_ - theta);
                if (std::abs(start_heading_error) > start_heading_enter_tolerance_)
                {
                    start_heading_alignment_active_ = true;
                    RCLCPP_INFO(
                            logger_,
                            "PID_LOG start_heading_alignment=entered plan_revision=%zu "
                            "target_yaw=%.6f current_yaw=%.6f heading_error=%.6f",
                            plan_revision_, start_heading_target_, theta, start_heading_error);
                }
            }
        }

        const double start_heading_error = angles::normalize_angle(
                start_heading_target_ - theta);
        if (start_heading_alignment_active_ &&
            std::abs(start_heading_error) <= start_heading_exit_tolerance_)
        {
            start_heading_alignment_active_ = false;
            RCLCPP_INFO(
                    logger_,
                    "PID_LOG start_heading_alignment=completed plan_revision=%zu "
                    "target_yaw=%.6f current_yaw=%.6f heading_error=%.6f",
                    plan_revision_, start_heading_target_, theta, start_heading_error);
        }

        if (start_heading_alignment_active_)
        {
            raw_command[1] = start_heading_kp_ * start_heading_error;
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = angularRegularization(
                    velocity, raw_command[1]);
        }
        else
        {
            Eigen::Vector3d s(
                    current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);
            Eigen::Vector3d s_d(
                    target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_d);
            Eigen::Vector2d u_r(vt, wt);
            // ZZP: 根据参数选择
            Eigen::Vector2d u;
            if(use_model_)
                u = UseModelPidControl(s, s_d, u_r);
            else
                u = NoModelPidControl(s, s_d, u_r);
            if (!eigenVectorIsFinite(u))
            {
                throw nav2_core::NoValidControl(
                        controller_plugin_name + " produced a non-finite raw control command");
            }
            raw_command = u;

            cmd_vel.twist.linear.x = std::max(
                    0.0, linearRegularization(velocity, u[0]));
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = angularRegularization(velocity, u[1]);
        }
    }
    // publish next target_ps_map pose
#if 0
    target_ps_map.header.frame_id = "map";
    target_ps_map.header.stamp = ros::Time::now();
    target_pose_pub_.publish(target_ps_map);
    
    // publish robot pose
    current_ps_map.header.frame_id = "map";
    current_ps_map.header.stamp = ros::Time::now();
    current_pose_pub_.publish(current_ps_map);
#endif
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = base_frame_;
    cmd_vel.twist.linear.y = 0.0;
    if (!twistIsFinite(cmd_vel.twist))
    {
        throw nav2_core::NoValidControl(
                controller_plugin_name + " produced a non-finite velocity command");
    }

    target_ps_map.header.frame_id = lookahead_pt.header.frame_id;
    target_ps_map.header.stamp = cmd_vel.header.stamp;
    if (carrot_pub_ && carrot_pub_->is_activated())
    {
        carrot_pub_->publish(target_ps_map);
    }
    updateAndLogTrackingDiagnostics(
            pose, velocity, cmd_vel, raw_command, start_heading_alignment_active_,
            prune_plan, L, theta_trj, kappa, lookahead_pt);

    return cmd_vel;
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
    Eigen::Vector2d u;
    Eigen::Vector3d e(s_d - s);
    const double ex_body = cos(s[2]) * e[0] + sin(s[2]) * e[1];
    const double ey_body = -sin(s[2]) * e[0] + cos(s[2]) * e[1];
    u[0] = model_k_v_ * ex_body;
    u[1] = model_k_w_ / l_ * ey_body;
    
    return u;
}

}  // namespace pid_tracking_controller

PLUGINLIB_EXPORT_CLASS(
  pid_tracking_controller::PidTrackingController,
  nav2_core::Controller)
