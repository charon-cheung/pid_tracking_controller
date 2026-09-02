#ifndef PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_
#define PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_

#include <vector>

#include "pid_tracking_controller/pid_tracking_controller.hpp"
#include "rclcpp/clock.hpp"

namespace pid_tracking_controller
{

class PidTrackingControllerTestAccess
{
public:
  static void setModelParameters(
    PidTrackingController & controller, double k, double l)
  {
    controller.k_ = k;
    controller.l_ = l;
  }

  static void setLookaheadParameters(
    PidTrackingController & controller, double time, double minimum, double maximum)
  {
    controller.lookahead_time_ = time;
    controller.min_lookahead_dist_ = minimum;
    controller.max_lookahead_dist_ = maximum;
  }

  static void setNoModelParameters(PidTrackingController & controller, double dt)
  {
    controller.d_t_ = dt;
    controller.k_v_p_ = 2.0;
    controller.k_v_i_ = 0.01;
    controller.k_v_d_ = 0.10;
    controller.k_w_p_ = 1.0;
    controller.k_w_i_ = 0.01;
    controller.k_w_d_ = 0.10;
    controller.max_v_ = 0.7;
    controller.min_v_ = 0.05;
    controller.max_v_inc_ = 0.5;
    controller.max_w_ = 0.2;
    controller.min_w_ = 0.0;
    controller.max_w_inc_ = 0.2;
    controller.e_v_ = 0.0;
    controller.e_w_ = 0.0;
    controller.i_v_ = 0.0;
    controller.i_w_ = 0.0;
  }

  static void replaceGlobalPlan(
    PidTrackingController & controller,
    const std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    controller.global_plan_ = plan;
  }

  static void setCostmap(
    PidTrackingController & controller, nav2_costmap_2d::Costmap2D * costmap)
  {
    controller.costmap_ = costmap;
  }

  static void setClock(
    PidTrackingController & controller, const rclcpp::Clock::SharedPtr & clock)
  {
    controller.clock_ = clock;
  }

  static void setMaximumVelocityIncrements(
    PidTrackingController & controller, double linear, double angular)
  {
    controller.max_v_inc_ = linear;
    controller.max_w_inc_ = angular;
  }

  static const std::vector<geometry_msgs::msg::PoseStamped> & globalPlan(
    const PidTrackingController & controller)
  {
    return controller.global_plan_;
  }

  static const std::vector<geometry_msgs::msg::PoseStamped> & plan(
    const PidTrackingController & controller)
  {
    return controller.global_plan_;
  }
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__TEST__CONTROLLER_TEST_ACCESS_HPP_
