#ifndef NAV2_LARERAL_CONTROLLER__LATERAL_CONTROLLER_HPP_
#define NAV2_LARERAL_CONTROLLER__LATERAL_CONTROLLER_HPP_

#include <algorithm>

#include "class_loader/multi_library_class_loader.h"
#include "class_loader/register_macro.h"
#include "nav2_core/controller.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "pluginlib/private_class_loader.h"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "common/geometry/vec2d.h"
#include <Eigen/Dense>

using  namespace  rmp::common::geometry;

namespace pid_tracking_controller {

typedef  geometry_msgs::msg::Pose2D  Pose2D;

class PidTrackingController : public nav2_core::Controller {
public:
    /**
     * @brief Constructor that brings up pluginlib loaders
     */
    PidTrackingController();
    ~PidTrackingController();

    void configure( const rclcpp::Node::SharedPtr _node, std::string name,
                    std::shared_ptr<tf2_ros::Buffer> tf,
                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    void setPlan(const nav_msgs::msg::Path& path) override;

    bool isGoalReached();
    /**
     * @param pose Current robot pose
     * @param velocity Current robot velocity
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
     * @return The best command for the robot to drive
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                             const geometry_msgs::msg::Twist& velocity,
                                                             nav2_core::GoalChecker* /*goal_checker*/) override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;
    void updateParam(bool update, double curvature){ }

    void setFactor(double factor);
    double getYawAngle(geometry_msgs::msg::PoseStamped& ps);
    bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& cur, const geometry_msgs::msg::PoseStamped& goal);
    bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);

    double linearRegularization(nav_2d_msgs::msg::Twist2D& odom_twist, double v_d);
    double angularRegularization(nav_2d_msgs::msg::Twist2D& odom_twist, double w_d);
    std::vector<geometry_msgs::msg::PoseStamped> prune(const geometry_msgs::msg::PoseStamped robot_pose_global);
    double getLookAheadDistance(double vt);
    void getLookAheadPoint(double lookahead_dist, geometry_msgs::msg::PoseStamped robot_pose_global,
                           const std::vector<geometry_msgs::msg::PoseStamped>& prune_plan, 
                           geometry_msgs::msg::PointStamped& pt,
                           double& theta, double& kappa);

   /**
   * @brief Execute PID control process (no model pid)
   * @param s   current state
   * @param s_d desired state
   * @param u_r refered input
   * @return u  control vector
   */
    Eigen::Vector2d NoModelPidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r);
    Eigen::Vector2d UseModelPidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r);

private:
    double factor_;  // obstacle factor(greater means obstacles)

    double max_v_, min_v_, max_v_inc_;
    double max_w_, min_w_, max_w_inc_;

    // if the distance is less than the tolerance value, 
    // it is considered to have reached the target position
    double goal_dist_tol_;

    // if the angle deviation is greater than this threshold, perform rotation first
    double rotate_tol_;

    std::string base_frame_, map_frame_, odom_frame_;
    double lookahead_time_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;

    bool goal_reached_;

    // pid controller params
    double k_v_p_, k_v_i_, k_v_d_;
    double k_w_p_, k_w_i_, k_w_d_;
    double k_theta_;
    double k_, l_;

    double e_v_, e_w_;
    double i_v_, i_w_;

    double d_t_;  // control time step
    double goal_x_, goal_y_, goal_theta_;

    geometry_msgs::msg::PoseStamped  current_pose;
    std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
    std::string  controller_plugin_name;
    bool use_model_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Msg::rcmsg_pub<geometry_msgs::msg::PoseStamped>> carrot_pub;
    std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
    // ros::Publisher target_pose_pub_, current_pose_pub_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    rclcpp::Clock::SharedPtr m_clock;
};

}  // namespace pid_tracking_controller
#endif