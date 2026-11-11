#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "execution_safety_adapter.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rcutils/logging.h"

namespace pid_tracking_controller
{
namespace
{
using namespace std::chrono_literals;
std::mutex log_mutex;
std::string logs;
void capture(const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char buffer[4096]; va_list copy; va_copy(copy, *args);
  vsnprintf(buffer, sizeof(buffer), format, copy); va_end(copy);
  std::lock_guard<std::mutex> lock(log_mutex); logs += std::string(buffer) + '\n';
}
class AdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    previous = rcutils_logging_get_output_handler(); rcutils_logging_set_output_handler(capture);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("observer_test");
    map_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap", "/adapter_test");
    map_node->declare_parameter("global_frame", "map");
    map_node->declare_parameter("robot_base_frame", "base");
    map_node->declare_parameter("always_send_full_costmap", true);
    map_node->declare_parameter("update_frequency", 10.);
    map_node->declare_parameter("plugins", std::vector<std::string>{"obstacle"});
    map_node->declare_parameter("obstacle.enabled", true);
    map_node->declare_parameter("obstacle.observation_sources", "scan duplicate cloud");
    for (auto source : {"scan", "duplicate", "cloud"}) {
      const std::string p = std::string("obstacle.") + source + ".";
      const bool cloud = std::string(source) == "cloud";
      map_node->declare_parameter(p + "topic", cloud ? "cloud" : "scan");
      map_node->declare_parameter(p + "data_type", cloud ? "PointCloud2" : "LaserScan");
      map_node->declare_parameter(p + "marking", true);
      map_node->declare_parameter(p + "clearing", false);
      map_node->declare_parameter(p + "expected_update_rate", 0.);
    }
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    publisher = std::make_shared<rclcpp::Node>("inputs_test", "/adapter_test");
    map_pub = publisher->create_publisher<nav2_msgs::msg::Costmap>("costmap_raw",
        rclcpp::QoS(1).reliable().transient_local());
    footprint_pub = publisher->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "published_footprint", 1);
    scan_pub = publisher->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    cloud_pub = publisher->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());
    adapter = std::make_unique<ExecutionSafetyAdapter>(node, map_node, tf); adapter->start();
  }
  void TearDown() override
  {
    adapter.reset();
    {std::lock_guard<std::mutex> lock(log_mutex); std::fputs(logs.c_str(), stdout); logs.clear();}
    rcutils_logging_set_output_handler(previous);
    map_pub.reset(); footprint_pub.reset(); scan_pub.reset(); cloud_pub.reset();
    publisher.reset(); tf.reset(); map_node.reset(); node.reset(); rclcpp::shutdown();
  }
  void clearLogs() {std::lock_guard<std::mutex> lock(log_mutex); log_start = logs.size();}
  bool contains(const std::string & text)
  {std::lock_guard<std::mutex> lock(log_mutex); return logs.find(text, log_start) != std::string::npos;}
  void feed(bool cloud = true, bool valid_scan = true)
  {
    auto now = node->now();
    nav2_msgs::msg::Costmap map; map.header.frame_id = "map"; map.header.stamp = now;
    if (publish_tf) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = map.header; transform.child_frame_id = "base";
      transform.transform.rotation.w = 1.; tf->setTransform(transform, "test", false);
    }
    map.metadata.size_x = map.metadata.size_y = 120; map.metadata.resolution = .05;
    map.metadata.origin.orientation.w = 1.; map.data.assign(14400, 254); map_pub->publish(map);
    geometry_msgs::msg::PolygonStamped footprint; footprint.header = map.header;
    for (auto xy : {std::pair<float, float>{-.2, -.2}, {.2, -.2}, {.2, .2}, {-.2, .2}}) {
      geometry_msgs::msg::Point32 p; p.x = xy.first; p.y = xy.second;
      footprint.polygon.points.push_back(p);
    }
    footprint_pub->publish(footprint);
    sensor_msgs::msg::LaserScan scan; scan.header = map.header;
    scan.angle_max = .1; scan.angle_increment = .1; scan.range_min = .1; scan.range_max = 10.;
    if (valid_scan) {scan.ranges = {1., 2.};}
    if (scan_fault == 1) {scan.time_increment = std::numeric_limits<float>::quiet_NaN();}
    if (scan_fault == 2) {scan.ranges.assign(2, std::numeric_limits<float>::quiet_NaN());}
    if (scan_fault == 3) {scan.ranges.assign(2, std::numeric_limits<float>::infinity());}
    scan_pub->publish(scan);
    if (cloud) {
      sensor_msgs::msg::PointCloud2 points; points.header = map.header;
      points.height = 1; points.width = 0; points.point_step = 12;
      for (int i = 0; i < 3; ++i) {
        sensor_msgs::msg::PointField f; f.name = std::string(1, "xyz"[i]);
        f.offset = 4 * i; f.count = 1; f.datatype = f.FLOAT32; points.fields.push_back(f);
      }
      cloud_pub->publish(points);  // A structurally valid empty cloud is not a missing stream.
    }
  }
  bool waitFor(const std::string & text, bool cloud = true, bool valid_scan = true, bool publish = true)
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < end) {
      if (publish) {feed(cloud, valid_scan);}
      if (contains(text)) {return true;} std::this_thread::sleep_for(30ms);
    }
    return contains(text);
  }
  bool publish_tf{true};
  int scan_fault{0};
  size_t log_start{0};
  rcutils_logging_output_handler_t previous{};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, map_node;
  rclcpp::Node::SharedPtr publisher;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<ExecutionSafetyAdapter> adapter;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
};

TEST_F(AdapterTest, RequiresEverySourceAndRecoversWithoutDeclaringCollisionSafety)
{
  ASSERT_TRUE(waitFor("reason=observation_missing_or_stale source=obstacle.cloud", false));
  EXPECT_EQ(scan_pub->get_subscription_count(), 1u);  // Duplicate logical source, one wire.
  ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
  EXPECT_TRUE(contains("action=none")); EXPECT_TRUE(contains("ros_age="));
  clearLogs();
  ASSERT_TRUE(waitFor("reason=observation_missing_or_stale source=obstacle.scan", true, false));
  ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
  clearLogs();
  ASSERT_TRUE(waitFor("reason=map_missing_or_stale", true, true, false));
}

TEST_F(AdapterTest, RejectsMalformedScanAndRequiresExactTimeDynamicTf)
{
  ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
  for (int fault : {1, 2}) {
    clearLogs(); scan_fault = fault;
    ASSERT_TRUE(waitFor("reason=observation_missing_or_stale source=obstacle.scan"));
    EXPECT_TRUE(contains("validation=payload_invalid"));
    clearLogs(); scan_fault = 0;
    ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
  }
  clearLogs(); publish_tf = false;
  ASSERT_TRUE(waitFor("reason=robot_tf_unavailable"));
  clearLogs(); publish_tf = true; scan_fault = 3;
  ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
}

TEST_F(AdapterTest, ReportsFourIndependentMotionStagesAndMissingStreams)
{
  adapter->stop();
  auto motion = std::make_shared<MotionObservation>("base");
  adapter = std::make_unique<ExecutionSafetyAdapter>(node, map_node, tf, motion, "/adapter_test/smooth");
  auto published = publisher->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 1);
  auto smoothed = publisher->create_publisher<geometry_msgs::msg::TwistStamped>("smooth", 1);
  adapter->start(); clearLogs();
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < end && !contains("stage=smoothed state=valid")) {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node->now(); command.header.frame_id = "base";
    command.twist.linear.x = .4; command.twist.linear.y = -.2; command.twist.angular.z = .1;
    motion->record(MotionObservation::Candidate, command.header, command.twist, "base", 3);
    published->publish(command);
    command.header.stamp.sec -= 2; command.twist.linear.x = .3;
    smoothed->publish(command); feed(); std::this_thread::sleep_for(30ms);
  }
  ASSERT_TRUE(contains("stage=candidate state=valid"));
  ASSERT_TRUE(contains("stage=published state=valid"));
  ASSERT_TRUE(contains("stage=smoothed state=valid"));
  EXPECT_TRUE(contains("stage=odometry state=missing"));
  EXPECT_TRUE(contains("alignment=asynchronous")); EXPECT_TRUE(contains("plan_revision=3"));
  EXPECT_TRUE(contains("vx=0.300000 vy=-0.200000 omega=0.100000"));
  clearLogs(); EXPECT_TRUE(waitFor("stage=smoothed state=receipt_stale"));
  adapter->stop(); clearLogs(); adapter->start();
  EXPECT_TRUE(waitFor("stage=candidate state=missing"));
}

TEST_F(AdapterTest, ReactivationDiscardsCachedInputsAndConfigurationChangesInvalidate)
{
  ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
  adapter->stop(); clearLogs();
  feed(); std::this_thread::sleep_for(150ms);
  EXPECT_FALSE(contains("PID_OBSERVE status"));
  adapter->start();
  ASSERT_TRUE(waitFor("ready=false", true, true, false));
  ASSERT_TRUE(waitFor("mode=observe_only ready=true reason=ready"));
  clearLogs();
  map_node->set_parameter(rclcpp::Parameter("obstacle.enabled", false));
  ASSERT_TRUE(waitFor("reason=configuration_changed_reactivate"));
  EXPECT_FALSE(contains("mode=observe_only ready=true"));
  adapter->stop(); adapter->start();
  EXPECT_TRUE(contains("no_active_observation_sources"));
}
}  // namespace
}  // namespace pid_tracking_controller
