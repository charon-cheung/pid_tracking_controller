#include "pid_tracking_controller/pid_tracking_controller.hpp"

#include "execution_safety_gate.hpp"
#include "execution_safety_inputs.hpp"
#include "motion_transition_observer.hpp"

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
      RCLCPP_INFO(logger_, "PID_GATE started basis=nominal trigger=yellow_or_red clear=skip_sweep "
        "unavailable=protect capture=control_cycle grid=local_costmap_direct budget_ms=10 "
        "release_clear_cycles=3 execution_evidence=missing physical_response=unverified");
    }
    for (const auto & entry : {std::pair<MotionObservation::Stage, std::string>{
          MotionObservation::Published, "cmd_vel"}, {MotionObservation::Smoothed, observed_smoothed_topic_}})
    {
      safety_subscriptions_.push_back(node->create_subscription<geometry_msgs::msg::TwistStamped>(
          entry.second, rclcpp::SensorDataQoS().keep_last(1),
          [motion = motion_observation_, stage = entry.first](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
            motion->record(stage, m->header, m->twist, m->header.frame_id);
          }));
    }
    transition_observer_ = execution_safety_gate_ ? nullptr : std::make_shared<MotionTransitionObserver>();
    const auto observer = transition_observer_;
    if (observer) {
      RCLCPP_INFO(logger_, "PID_TRANSITION started mode=observe_only action=none "
        "executor=controller pose_anchor=footprint_stamp poll_ms=100 "
        "basis=nominal execution_evidence=missing physical_response=unverified");
    }
    // Callback state owns its inputs, never the controller. A canceled old timer
    // or in-flight subscription cannot write a later activation's input cache.
    observation_timer_ = node->create_wall_timer(std::chrono::milliseconds(100),
      [inputs, observer, motion = motion_observation_, costmap = costmap_ros_, tf = tf_,
        nominal = *nominal_transition_, clock = clock_, logger = logger_]() {
        motion->report(logger, clock->now().nanoseconds());
        if (!observer) {return;}
        const auto started = ExecutionSafetyInputs::Wall::now();
        auto geometry = inputs->captureObserved(clock->now(), *tf, *costmap->getLayeredCostmap());
        const auto captured = ExecutionSafetyInputs::Wall::now();
        const auto history = motion->history();
        const auto copied = ExecutionSafetyInputs::Wall::now();
        const auto now = clock->now().nanoseconds();
        const auto pose_stamp = geometry.snapshot ? geometry.snapshot->pose_stamp_ns : 0;
        const auto result = observer->evaluate(geometry, history, pose_stamp, now, started, nullptr, &nominal);
        observer->report(logger, result, history, now,
          std::chrono::duration<double, std::milli>(captured - started).count(),
          std::chrono::duration<double, std::milli>(copied - captured).count());
      });
    execution_safety_inputs_ = std::move(inputs);
  } catch (const std::exception & error) {
    stopExecutionSafety();
    RCLCPP_WARN(logger_, "PID_SAFETY inputs unavailable reason=%s", error.what());
  }
}

void PidTrackingController::stopExecutionSafety()
{
  if (observation_timer_) {observation_timer_->cancel();}
  observation_timer_.reset();
  if (transition_observer_) {transition_observer_->stopped(logger_);}
  transition_observer_.reset();
  safety_subscriptions_.clear();
  execution_safety_inputs_.reset();
  if (motion_observation_) {motion_observation_->deactivate();}
}
}  // namespace pid_tracking_controller
