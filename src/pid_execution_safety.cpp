#include "pid_tracking_controller/pid_tracking_controller.hpp"

#include "execution_safety_gate.hpp"
#include "execution_safety_inputs.hpp"

namespace pid_tracking_controller
{
void PidTrackingController::startExecutionSafety()
{
  stopExecutionSafety();
  if (!motion_observation_) {return;}
  const auto node = node_.lock();
  if (!node) {return;}
  try {
    motion_observation_->activate();
    safety_pose_stamp_ = 0; safety_pose_received_ = {};
    auto inputs = std::make_shared<ExecutionSafetyInputs>(safetyInputOptions(*costmap_ros_));
    inputs->subscribe(node, costmap_ros_);
    if (execution_safety_gate_) {
      safety_subscriptions_.push_back(node->create_subscription<msg::PathRiskState>(
          "path_risk/control_state", rclcpp::QoS(1).reliable(),
          [inputs](msg::PathRiskState::ConstSharedPtr risk) {inputs->recordPathRisk(std::move(risk));}));
      RCLCPP_INFO(logger_, "PID_GATE started stage=2-C motion=2-A evaluation=2-B basis=nominal "
        "trigger=yellow_or_red_or_braking clear=skip_only_tracking "
        "unavailable=protect capture=control_cycle grid=local_costmap_direct budget_ms=10 "
        "release_fresh_observations=3 stop_observation=required execution_evidence=nominal physical_response=unverified "
        "simulation=%d input_recovery=%s", node->get_parameter("use_sim_time").as_bool(),
        node->get_parameter("use_sim_time").as_bool() ? "current_safe_motion" : "stopped_confirmation");
    }
    execution_safety_inputs_ = std::move(inputs);
  } catch (const std::exception & error) {
    stopExecutionSafety();
    RCLCPP_WARN(logger_, "PID_SAFETY inputs unavailable reason=%s", error.what());
  }
}

void PidTrackingController::stopExecutionSafety()
{
  if (execution_safety_inputs_) {
    RCLCPP_INFO(logger_, "PID_MOTION stage=2-A event=stopped history=discarded");
  }
  safety_subscriptions_.clear();
  execution_safety_inputs_.reset();
  if (motion_observation_) {motion_observation_->deactivate();}
}
}  // namespace pid_tracking_controller
