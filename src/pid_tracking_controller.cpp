/*
 *	Copyright (C), suzhou, muhai
 *	@author:	ZZP
 *	@date:		2025/11/03
 *	@note:
 */
#include "pid_tracking_controller/pid_tracking_controller.hpp"
#include "common/geometry/polygon2d.h"
#include "common/geometry/point.h"
#include "common/geometry/vec2d.h"
#include "common/math/math_helper.h"
#include <sstream>

using namespace nav2_util::geometry_utils;
using nav2_util::declare_parameter_if_not_declared;

namespace pid_tracking_controller {

PidTrackingController::PidTrackingController()
{
}

PidTrackingController::~PidTrackingController()
{
}


void PidTrackingController::configure(const rclcpp::Node::SharedPtr _node, std::string name, 
                                      std::shared_ptr<tf2_ros::Buffer> tf,
                                      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    controller_plugin_name = name;
    node_                = _node;
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".goal_dist_tolerance", rclcpp::ParameterValue(0.05) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".rotate_tolerance", rclcpp::ParameterValue(0.5) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".base_frame", rclcpp::ParameterValue("base_link") );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".map_frame", rclcpp::ParameterValue("map") );
    
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".lookahead_time", rclcpp::ParameterValue(0.5) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".min_lookahead_dist", rclcpp::ParameterValue(0.3) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".max_lookahead_dist", rclcpp::ParameterValue(0.9) );
    
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".max_v", rclcpp::ParameterValue(0.7));   // 0.46
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".min_v", rclcpp::ParameterValue(0.05) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".max_v_inc", rclcpp::ParameterValue(0.5) );
    
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".max_w", rclcpp::ParameterValue(0.2));   // 1.57
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".min_w", rclcpp::ParameterValue(0.0) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".max_w_inc", rclcpp::ParameterValue(0.2) );  // 1.57
    
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_v_p", rclcpp::ParameterValue(2.00) );   // 1
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_v_i", rclcpp::ParameterValue(0.01) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_v_d", rclcpp::ParameterValue(0.10) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_w_p", rclcpp::ParameterValue(1.00) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_w_i", rclcpp::ParameterValue(0.01) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_w_d", rclcpp::ParameterValue(0.10) );
    
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k_theta", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".k", rclcpp::ParameterValue(1.0) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".l", rclcpp::ParameterValue(0.2) );
    
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".pid_frequency", rclcpp::ParameterValue(15.0) );
    declare_parameter_if_not_declared(node_, controller_plugin_name + ".use_model", rclcpp::ParameterValue(true) );
    
    node_->get_parameter(controller_plugin_name + ".goal_dist_tolerance", goal_dist_tol_);
    node_->get_parameter(controller_plugin_name + ".rotate_tolerance", rotate_tol_);
    node_->get_parameter(controller_plugin_name + ".base_frame", base_frame_);
    node_->get_parameter(controller_plugin_name + ".map_frame", map_frame_);
    
    
    node_->get_parameter(controller_plugin_name + ".lookahead_time", lookahead_time_);
    node_->get_parameter(controller_plugin_name + ".min_lookahead_dist", min_lookahead_dist_);
    node_->get_parameter(controller_plugin_name + ".max_lookahead_dist", max_lookahead_dist_);
    node_->get_parameter(controller_plugin_name + ".max_v", max_v_);
    node_->get_parameter(controller_plugin_name + ".min_v", min_v_);
    node_->get_parameter(controller_plugin_name + ".max_v_inc", max_v_inc_);
    
    node_->get_parameter(controller_plugin_name + ".max_w", max_w_);
    node_->get_parameter(controller_plugin_name + ".min_w", min_w_);
    node_->get_parameter(controller_plugin_name + ".max_w_inc", max_w_inc_);
    
    node_->get_parameter(controller_plugin_name + ".k_v_p", k_v_p_);
    node_->get_parameter(controller_plugin_name + ".k_v_i", k_v_i_);
    node_->get_parameter(controller_plugin_name + ".k_v_d", k_v_d_);
    node_->get_parameter(controller_plugin_name + ".k_w_p", k_w_p_);
    node_->get_parameter(controller_plugin_name + ".k_w_i", k_w_i_);
    node_->get_parameter(controller_plugin_name + ".k_w_d", k_w_d_);
    
    node_->get_parameter(controller_plugin_name + ".k_theta", k_theta_);
    node_->get_parameter(controller_plugin_name + ".k", k_);
    node_->get_parameter(controller_plugin_name + ".l", l_);
    
    double pid_frequency;
    node_->get_parameter(controller_plugin_name + ".pid_frequency", pid_frequency);
    d_t_ = 1 / pid_frequency;
    
    node_->get_parameter(controller_plugin_name + ".use_model", use_model_);
    
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    
    goal_reached_ = false;
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
    goal_x_ = goal_y_ = DBL_MAX;
    m_clock = _node->get_clock();
    
    carrot_pub = std::make_shared<Msg::rcmsg_pub<geometry_msgs::msg::PoseStamped>>("/carrot");
    odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(_node);
}

void PidTrackingController::activate()
{
}

void PidTrackingController::deactivate()
{
}

void PidTrackingController::cleanup()
{
    
}

// 先执行 setPlan，再执行旋转，转到位后，再执行 computeVelocityCommands
void PidTrackingController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
    LOG_INFO("PidTracking Controller setPlan");
    global_plan_.clear();
    global_plan_ = orig_global_plan.poses;
    // receive a plan for a new goal
    if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
    {
        goal_x_ = global_plan_.back().pose.position.x;
        goal_y_ = global_plan_.back().pose.position.y;
        goal_theta_ = getYawAngle(global_plan_.back());
        goal_reached_ = false;
        
        e_v_ = i_v_ = 0.0;
        e_w_ = i_w_ = 0.0;
    }
    std::ostringstream oss_goal;
    oss_goal << std::fixed << std::setprecision(2) << goal_x_ << ", " << goal_y_ << ",  " << goal_theta_;
    // LOG_INFO("PID goal: ({})", oss_goal.str() );
}

bool PidTrackingController::isGoalReached()
{
    if (goal_reached_)
    {
        LOG_INFO("\033[42;37m PID Goal Reached !\033[0m");
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
double PidTrackingController::linearRegularization(nav_2d_msgs::msg::Twist2D& odom_twist, double v_d)
{
    std::ostringstream oss_v;
    
    double v = std::hypot(odom_twist.x, odom_twist.y);
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
double PidTrackingController::angularRegularization(nav_2d_msgs::msg::Twist2D& odom_twist, double w_d)
{
    if (std::fabs(w_d) > max_w_)
        w_d = std::copysign(max_w_, w_d);
    
    double w = odom_twist.theta;
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

void PidTrackingController::setSpeedLimit(const double& speed_limit, const bool& percentage)
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
                                                            costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0);
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
    std::ostringstream oss_pose;
    oss_pose << std::fixed << std::setprecision(2) << pose.pose.position.x << ", " << pose.pose.position.y;
    // LOG_INFO("current pose: ({})", oss_pose.str() );
    
    // LOG_INFO("PID input_linear: {}, input_angular: {}", velocity.linear.x, velocity.angular.z);
    geometry_msgs::msg::TwistStamped cmd_vel;
    
    nav_2d_msgs::msg::Twist2D twist_2d = odom_sub_->getTwist();
    // calculate look-ahead distance
    double vt = std::hypot(twist_2d.x, twist_2d.y);
    double wt = twist_2d.theta;
    LOG_INFO("vt: {}, wt: {}", vt, wt);
    
    geometry_msgs::msg::PoseStamped current_ps_map = pose;
    std::vector<geometry_msgs::msg::PoseStamped> prune_plan = prune(current_ps_map);
    
    double L = getLookAheadDistance(vt);
    
    // get the particular point on the path at the lookahead distance
    geometry_msgs::msg::PointStamped lookahead_pt;
    double theta_d, theta_dir, theta_trj, kappa;
    getLookAheadPoint(L, current_ps_map, prune_plan, lookahead_pt, theta_trj, kappa);
    
    geometry_msgs::msg::PoseStamped target_ps_map;
    target_ps_map.pose.position.x = lookahead_pt.point.x;
    target_ps_map.pose.position.y = lookahead_pt.point.y;
    theta_dir = atan2((target_ps_map.pose.position.y - current_ps_map.pose.position.y),
                      (target_ps_map.pose.position.x - current_ps_map.pose.position.x));
    
    theta_d = nav2_util::geometry_utils::normalize_angle((1 - k_theta_) * theta_trj + k_theta_ * theta_dir);
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_d);
    tf2::convert(q, target_ps_map.pose.orientation);
    
    double theta = tf2::getYaw(current_ps_map.pose.orientation);
    
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
            cmd_vel.twist.angular.z = angularRegularization(twist_2d, e_theta / d_t_);
        }
    }
    // posistion not reached
    else
#endif
    {
        
        Eigen::Vector3d s(current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);
        Eigen::Vector3d s_d(target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_d);
        Eigen::Vector2d u_r(vt, wt);
        // ZZP: 根据参数选择
        Eigen::Vector2d u;
        if(use_model_)
            u = UseModelPidControl(s, s_d, u_r);
        else
            u = NoModelPidControl(s, s_d, u_r);
        
        cmd_vel.twist.linear.x = linearRegularization(twist_2d, u[0]);
        cmd_vel.twist.angular.z = angularRegularization(twist_2d, u[1]);
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
                                                          Eigen::Vector2d  u_r)
{
    Eigen::Vector2d u;
    Eigen::Vector3d e(s_d - s);
    Eigen::Vector2d sx_dot(k_ * e[0], k_ * e[1]);
    // LOG_INFO("sx_dot:  {}, {}", k_ * e[0], k_ * e[1]);
    Eigen::Matrix2d R_inv;
    R_inv << cos(s[2]), sin(s[2]), -sin(s[2]) / l_, cos(s[2]) / l_;

    std::ostringstream oss_inv;
    oss_inv << std::fixed << std::setprecision(2) << cos(s[2]) << "  " << sin(s[2]) << "  " << -sin(s[2]) / l_ << "  " << cos(s[2]) / l_;
    // LOG_INFO("R inv: {}", oss_inv.str() );

    u = R_inv * sx_dot;
    
    return u;
}

}  // namespace pid_tracking_controller
CLASS_LOADER_REGISTER_CLASS(pid_tracking_controller::PidTrackingController, nav2_core::Controller)