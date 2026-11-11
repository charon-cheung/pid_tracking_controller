#ifndef PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_ADAPTER_HPP_
#define PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_ADAPTER_HPP_

#include <atomic>
#include <thread>
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "execution_safety_inputs.hpp"
#include "motion_observation.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pid_tracking_controller
{
// Owns observation subscriptions and a worker; never changes a velocity.
// Lifecycle methods are serialized by the controller; stop joins before destruction.
class ExecutionSafetyAdapter
{
public:
  ExecutionSafetyAdapter(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    rclcpp_lifecycle::LifecycleNode::SharedPtr costmap, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<MotionObservation> motion = nullptr,
    std::string smoothed_topic = "cmd_vel_smoothed");
  ~ExecutionSafetyAdapter();
  void start();
  void stop();

private:
  using Wall = ExecutionSafetyInputs::Wall;
  struct Input
  {
    std::string name, topic, type;
    std_msgs::msg::Header header{};
    Wall::time_point received{};
    uint64_t count{0};
    bool valid{false};
  };
  void connect(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void observe();
  void received(Input & input, const std_msgs::msg::Header & header, bool valid);
  void report(const SafetyInputResult & result, double elapsed_ms);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<MotionObservation> motion_;
  std::string smoothed_topic_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  SafetyInputOptions options_;
  std::unique_ptr<ExecutionSafetyInputs> inputs_;
  Input map_info_{"map", "", "Costmap"}, footprint_info_{"footprint", "", "PolygonStamped"};
  std::vector<Input> sources_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::shared_ptr<std::atomic<bool>> config_changed_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::string last_status_;
  Wall::time_point last_report_{};
  uint64_t ready_count_{0}, unavailable_count_{0};
};
}  // namespace pid_tracking_controller
#endif  // PID_TRACKING_CONTROLLER__EXECUTION_SAFETY_ADAPTER_HPP_
