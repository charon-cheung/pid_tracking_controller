#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "execution_safety_inputs.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pid_tracking_controller
{
namespace
{
using Wall = std::chrono::steady_clock;

std_msgs::msg::Header header(double seconds, const std::string & frame = "map")
{
  std_msgs::msg::Header h;
  h.frame_id = frame;
  h.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)));
  return h;
}

SafetyInputOptions options()
{
  SafetyInputOptions o;
  o.map_frame = "map"; o.base_frame = "base";
  o.sources = {{"scan", .6}};
  o.budget_seconds = 1.;  // Functional tests must not depend on machine load.
  return o;
}

nav2_msgs::msg::Costmap::SharedPtr mapMessage(double t = 9.9)
{
  auto m = std::make_shared<nav2_msgs::msg::Costmap>();
  m->header = header(t);
  m->metadata.size_x = m->metadata.size_y = 60;
  m->metadata.resolution = .1;
  m->metadata.origin.position.x = m->metadata.origin.position.y = -3.;
  m->metadata.origin.orientation.w = 1.;
  m->data.assign(3600, 0);
  return m;
}

geometry_msgs::msg::PolygonStamped::SharedPtr footprint(double t = 9.9)
{
  auto m = std::make_shared<geometry_msgs::msg::PolygonStamped>();
  m->header = header(t, "base");
  for (const auto & xy : {std::pair<float, float>{-.3, -.2}, {.3, -.2}, {.3, .2}, {-.3, .2}}) {
    geometry_msgs::msg::Point32 p; p.x = xy.first; p.y = xy.second;
    m->polygon.points.push_back(p);
  }
  return m;
}

geometry_msgs::msg::PoseStamped pose(double t = 10.)
{
  geometry_msgs::msg::PoseStamped p;
  p.header = header(t); p.pose.orientation.w = 1.;
  return p;
}

struct Fixture
{
  rclcpp::Clock::SharedPtr clock{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  tf2_ros::Buffer tf{clock};
  Wall::time_point wall{Wall::now()};
  ExecutionSafetyInputs inputs{options()};

  Fixture()
  {
    transform("base", "laser");
    transform("map", "base");
  }
  void transform(
    const std::string & parent, const std::string & child,
    double x = 0., double y = 0., double yaw = 0.)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header = header(9., parent); t.child_frame_id = child;
    t.transform.translation.x = x; t.transform.translation.y = y;
    t.transform.rotation.z = std::sin(yaw / 2); t.transform.rotation.w = std::cos(yaw / 2);
    ASSERT_TRUE(tf.setTransform(t, "test", true));
  }
  void fill(double t = 10.)
  {
    inputs.recordMap(mapMessage(t - .1), wall);
    inputs.recordFootprint(footprint(t - .1), wall);
    inputs.recordObservation("scan", header(t - .3, "laser"), true, wall);
    inputs.recordObservation("scan", header(t - .1, "laser"), true, wall);
  }
  SafetyInputResult read(double t = 10.)
  {
    return inputs.capture(pose(t), rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME), tf,
          wall, wall);
  }
};

TEST(ExecutionSafetyInputs, MissingInputsDoNotProduceASafeSnapshotAndRecover)
{
  Fixture f;
  EXPECT_FALSE(f.read().snapshot);
  f.fill();
  auto result = f.read();
  ASSERT_TRUE(result.snapshot) << result.reason;
  EXPECT_EQ(result.snapshot->map->getSizeInCellsX(), 60u);
  EXPECT_NEAR(result.snapshot->footprint[0].x, -.3, 1e-6);
}

TEST(ExecutionSafetyInputs, SnapshotOwnsMapGeometryCellsAndFootprintAcrossUpdates)
{
  Fixture f; f.fill();
  auto old = f.read(); ASSERT_TRUE(old.snapshot);
  auto changed = mapMessage(10.);
  changed->metadata.size_x = 20; changed->metadata.size_y = 30;
  changed->metadata.origin.position.x = -1.; changed->data.assign(600, 254);
  f.inputs.recordMap(changed, f.wall);
  auto body = footprint(10.); body->polygon.points[0].x = -.8;
  f.inputs.recordFootprint(body, f.wall);
  auto next = f.read(); ASSERT_TRUE(next.snapshot) << next.reason;
  EXPECT_EQ(old.snapshot->map->getCost(5, 5), 0u);
  EXPECT_EQ(old.snapshot->map->getSizeInCellsX(), 60u);
  EXPECT_DOUBLE_EQ(old.snapshot->map->getOriginX(), -3.);
  EXPECT_NEAR(old.snapshot->footprint[0].x, -.3, 1e-6);
  EXPECT_EQ(next.snapshot->map->getCost(5, 5), 254u);
  EXPECT_EQ(next.snapshot->map->getSizeInCellsX(), 20u);
  EXPECT_NEAR(next.snapshot->footprint[0].x, -.8, 1e-6);
}

TEST(ExecutionSafetyInputs, FreshMapCannotHideStaleObservation)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  f.inputs.recordMap(mapMessage(10.8), f.wall);
  f.inputs.recordFootprint(footprint(10.8), f.wall);
  auto result = f.read(10.8);
  EXPECT_FALSE(result.snapshot);
  EXPECT_EQ(result.source, "scan");
  // A new scan after the gap alone cannot certify the already published map.
  f.inputs.recordObservation("scan", header(10.8, "laser"), true, f.wall);
  EXPECT_FALSE(f.read(10.8).snapshot);
  f.inputs.recordMap(mapMessage(11.), f.wall);
  f.inputs.recordFootprint(footprint(11.), f.wall);
  f.inputs.recordObservation("scan", header(11., "laser"), true, f.wall);
  EXPECT_TRUE(f.read(11.).snapshot);
}

TEST(ExecutionSafetyInputs, EveryConfiguredSourceIsRequired)
{
  Fixture f;
  auto o = options(); o.sources.push_back({"cloud", .6});
  ExecutionSafetyInputs inputs(o);
  inputs.recordMap(mapMessage(), f.wall); inputs.recordFootprint(footprint(), f.wall);
  inputs.recordObservation("scan", header(9.7, "laser"), true, f.wall);
  auto result = inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.wall, f.wall);
  EXPECT_FALSE(result.snapshot); EXPECT_EQ(result.source, "cloud");
  inputs.recordObservation("cloud", header(9.7, "laser"), true, f.wall);
  EXPECT_TRUE(inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.wall,
        f.wall).snapshot);
}

TEST(ExecutionSafetyInputs, FrozenRosClockStillExpiresBySteadyReceiptAge)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  f.wall += std::chrono::seconds(1);
  EXPECT_FALSE(f.read().snapshot);
  // Replayed duplicate headers cannot acquire fresh receipt times.
  f.inputs.recordMap(mapMessage(), f.wall);
  f.inputs.recordFootprint(footprint(), f.wall);
  f.inputs.recordObservation("scan", header(9.9, "laser"), true, f.wall);
  EXPECT_FALSE(f.read().snapshot);
  f.fill(11.); EXPECT_TRUE(f.read(11.).snapshot);
}

TEST(ExecutionSafetyInputs, EachStreamMustBeFreshInRosAndSteadyTime)
{
  for (int stream = 0; stream < 3; ++stream) {
    Fixture f; f.fill();
    const auto old = f.wall - std::chrono::seconds(1);
    if (stream == 0) {f.inputs.recordMap(mapMessage(10.), old);}
    if (stream == 1) {f.inputs.recordFootprint(footprint(10.), old);}
    if (stream == 2) {f.inputs.recordObservation("scan", header(10., "laser"), true, old);}
    EXPECT_FALSE(f.read().snapshot) << stream;
  }
  Fixture f; f.fill();
  EXPECT_FALSE(f.read(10.7).snapshot);
}

TEST(ExecutionSafetyInputs, ClockRegressionAndExplicitResetRequireNewInputs)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  EXPECT_FALSE(f.read(1.).snapshot);
  f.fill(2.); EXPECT_TRUE(f.read(2.).snapshot);
  f.inputs.reset(); EXPECT_FALSE(f.read(2.).snapshot);
  f.fill(3.); EXPECT_TRUE(f.read(3.).snapshot);
}

TEST(ExecutionSafetyInputs, BadOrReorderedObservationInvalidatesOldEvidence)
{
  for (const double t : {0., 9.8, 9.9, 10.1}) {
    Fixture f; f.fill();
    f.inputs.recordObservation("scan", header(t, "laser"), true, f.wall);
    EXPECT_FALSE(f.read().snapshot) << t;
  }
  Fixture f; f.fill();
  f.inputs.recordObservation("scan", header(10., "laser"), false, f.wall);
  EXPECT_FALSE(f.read().snapshot);
  f.fill(11.); EXPECT_TRUE(f.read(11.).snapshot);
}

TEST(ExecutionSafetyInputs, InvalidMapGeometryOrDataNeverReusesAnOldMap)
{
  for (int bad = 0; bad < 8; ++bad) {
    Fixture f; f.fill(); auto m = mapMessage(10.);
    switch (bad) {
      case 0: m->data.pop_back(); break;
      case 1: m->metadata.resolution = 0.; break;
      case 2: m->metadata.size_x = 0; break;
      case 3: m->metadata.size_x = std::numeric_limits<uint32_t>::max(); break;
      case 4: m->metadata.origin.position.x = std::numeric_limits<double>::quiet_NaN(); break;
      case 5: m->metadata.origin.orientation.z = .1; break;
      case 6: m->header.frame_id = "odom"; break;
      case 7: m->header = header(11.); break;
    }
    f.inputs.recordMap(m, f.wall); EXPECT_FALSE(f.read().snapshot) << bad;
  }
}

TEST(ExecutionSafetyInputs, InvalidFootprintsAndPosesAreUnavailable)
{
  for (int bad = 0; bad < 4; ++bad) {
    Fixture f; f.fill(); auto body = footprint(10.);
    if (bad == 0) {body->polygon.points.clear();}
    if (bad == 1) {body->polygon.points[0].x = std::numeric_limits<float>::quiet_NaN();}
    if (bad == 2) {
      for (auto & p : body->polygon.points) {
        p.y = 0.;
      }
    }
    if (bad == 3) {body->header.frame_id = "unconnected";}
    f.inputs.recordFootprint(body, f.wall); EXPECT_FALSE(f.read().snapshot) << bad;
  }
  for (int bad = 0; bad < 6; ++bad) {
    Fixture f; f.fill(); auto p = pose();
    if (bad == 0) {p.header = header(9.);}
    if (bad == 1) {p.header = header(10.1);}
    if (bad == 2) {p.pose.orientation.w = 0.;}
    if (bad == 3) {p.pose.position.x = std::numeric_limits<double>::infinity();}
    if (bad == 4) {p.header.frame_id = "unconnected";}
    if (bad == 5) {p.header.stamp = builtin_interfaces::msg::Time();}
    EXPECT_FALSE(f.inputs.capture(p, rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.wall,
          f.wall).snapshot) << bad;
  }
}

TEST(ExecutionSafetyInputs, SensorTfMustExistAtTheObservationTime)
{
  Fixture f; f.fill();
  f.inputs.recordObservation("scan", header(10., "missing_sensor"), true, f.wall);
  EXPECT_FALSE(f.read().snapshot);
  f.transform("base", "missing_sensor");
  // Frame change discarded old anchors; wait for a map following this source.
  f.inputs.recordMap(mapMessage(10.2), f.wall);
  f.inputs.recordFootprint(footprint(10.2), f.wall);
  f.inputs.recordObservation("scan", header(10.2, "missing_sensor"), true, f.wall);
  EXPECT_TRUE(f.read(10.2).snapshot);
}

TEST(ExecutionSafetyInputs, PoseAndFootprintUseTheirOwnStampedTransforms)
{
  Fixture f; f.fill(); f.transform("map", "odom", 1., 2., M_PI / 2);
  f.transform("base", "outline", .1, 0., M_PI / 2);
  auto body = footprint(10.); body->header.frame_id = "outline";
  f.inputs.recordFootprint(body, f.wall);
  auto p = pose(); p.header.frame_id = "odom"; p.pose.position.x = .5;
  auto result = f.inputs.capture(p, rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.wall, f.wall);
  ASSERT_TRUE(result.snapshot) << result.reason;
  EXPECT_NEAR(result.snapshot->pose.x, 1., 1e-9);
  EXPECT_NEAR(result.snapshot->pose.y, 2.5, 1e-9);
  EXPECT_NEAR(result.snapshot->pose.yaw, M_PI / 2, 1e-9);
  EXPECT_NEAR(result.snapshot->footprint[0].x, .3, 1e-6);
  EXPECT_NEAR(result.snapshot->footprint[0].y, -.3, 1e-6);
}

TEST(ExecutionSafetyInputs, SnapshotFeedsTheExistingSweepWithoutChangingUnknownCells)
{
  Fixture f; f.fill(); auto m = mapMessage(10.);
  m->data[30 * 60 + 30] = nav2_costmap_2d::NO_INFORMATION;
  f.inputs.recordMap(m, f.wall);
  auto result = f.read(); ASSERT_TRUE(result.snapshot);
  ExecutionSafetyOptions o; o.budget_seconds = 1.; ExecutionSafety sweep(o);
  CommandSweepInput command{result.snapshot->pose, {}, 0., {.5, .5, 1.}};
  auto checked = sweep.checkCommand(command, *result.snapshot->map, result.snapshot->footprint);
  EXPECT_EQ(checked.status, SweepStatus::kUnavailable);
  m = mapMessage(10.1); m->data[30 * 60 + 30] = nav2_costmap_2d::LETHAL_OBSTACLE;
  f.inputs.recordMap(m, f.wall); result = f.read(10.1); ASSERT_TRUE(result.snapshot);
  EXPECT_EQ(sweep.checkCommand(command, *result.snapshot->map, result.snapshot->footprint).status,
    SweepStatus::kCollision);
}

TEST(ExecutionSafetyInputs, DynamicTfCannotUseALatestTransformForANewerScan)
{
  Fixture f; f.fill();
  geometry_msgs::msg::TransformStamped t;
  t.header = header(9.7); t.child_frame_id = "dynamic_laser";
  t.transform.rotation.w = 1.;
  ASSERT_TRUE(f.tf.setTransform(t, "test", false));
  f.inputs.recordObservation("scan", header(10., "dynamic_laser"), true, f.wall);
  EXPECT_FALSE(f.read().snapshot);
  t.header = header(10.3); ASSERT_TRUE(f.tf.setTransform(t, "test", false));
  f.inputs.recordMap(mapMessage(10.2), f.wall);
  f.inputs.recordFootprint(footprint(10.2), f.wall);
  f.inputs.recordObservation("scan", header(10.2, "dynamic_laser"), true, f.wall);
  EXPECT_TRUE(f.read(10.2).snapshot);
}

TEST(ExecutionSafetyInputs, WorkLimitsReturnUnavailableRatherThanPartialSnapshots)
{
  Fixture f;
  for (const bool tiny_budget : {false, true}) {
    auto o = options();
    if (tiny_budget) {o.budget_seconds = 1e-12;} else {o.max_cells = 100;}
    ExecutionSafetyInputs inputs(o);
    inputs.recordMap(mapMessage(), f.wall); inputs.recordFootprint(footprint(), f.wall);
    inputs.recordObservation("scan", header(9.7, "laser"), true, f.wall);
    EXPECT_FALSE(inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.wall,
          f.wall).snapshot);
  }
}

TEST(ExecutionSafetyInputs, ConcurrentMapReplacementNeverMixesGeometryAndCells)
{
  Fixture f; f.fill();
  std::atomic<bool> begin{false};
  std::thread writer([&]() {
      while (!begin.load()) {std::this_thread::yield();}
      for (int i = 1; i <= 100; ++i) {
        auto m = mapMessage(9.9 + i * .0005);
        m->metadata.size_x = i % 2 ? 20 : 60;
        m->data.assign(m->metadata.size_x * m->metadata.size_y, m->metadata.size_x);
        f.inputs.recordMap(m, f.wall);
      }
    });
  begin = true;
  for (int i = 0; i < 100; ++i) {
    auto result = f.read();
    if (!result.snapshot) {continue;}  // A busy collector may fail closed.
    auto & map = *result.snapshot->map;
    const auto x = map.getSizeInCellsX(), y = map.getSizeInCellsY();
    const auto cost = map.getCost(0, 0);
    EXPECT_TRUE(cost == 0 || cost == x);
    EXPECT_EQ(map.getCost(x - 1, y - 1), cost);
  }
  writer.join();
  EXPECT_TRUE(f.read().snapshot);
}

TEST(ExecutionSafetyInputs, InvalidObservationFencesOlderDelayedMessages)
{
  Fixture f; f.fill();
  f.inputs.recordObservation("scan", header(10., "laser"), false, f.wall);
  f.inputs.recordObservation("scan", header(9.95, "laser"), true, f.wall);
  f.inputs.recordMap(mapMessage(10.2), f.wall);
  EXPECT_FALSE(f.read(10.2).snapshot);
  f.inputs.recordObservation("scan", header(10.1, "laser"), true, f.wall);
  EXPECT_TRUE(f.read(10.2).snapshot);
}

TEST(ExecutionSafetyInputs, TiltedFootprintTransformDoesNotShrinkTheBodyProjection)
{
  Fixture f; f.fill();
  geometry_msgs::msg::TransformStamped t;
  t.header = header(9., "base"); t.child_frame_id = "tilted";
  t.transform.rotation.x = std::sin(.2); t.transform.rotation.w = std::cos(.2);
  ASSERT_TRUE(f.tf.setTransform(t, "test", true));
  auto body = footprint(10.); body->header.frame_id = "tilted";
  f.inputs.recordFootprint(body, f.wall);
  EXPECT_FALSE(f.read().snapshot);
}

TEST(ExecutionSafetyInputs, FrozenRosTimeDoesNotKeepAnOldPoseFresh)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  const auto later = f.wall + std::chrono::milliseconds(400);
  // Other streams are still within .6 s; the pose exceeded its .3 s deadline.
  auto result = f.inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.wall, later);
  EXPECT_FALSE(result.snapshot);
  EXPECT_EQ(result.reason, "pose_invalid_or_stale");
}

TEST(ExecutionSafetyInputs, ResumedRosClockCannotReusePrePauseObservationAnchors)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  f.wall += std::chrono::seconds(10);
  f.inputs.recordMap(mapMessage(10.1), f.wall);
  f.inputs.recordFootprint(footprint(10.1), f.wall);
  f.inputs.recordObservation("scan", header(10.1, "laser"), true, f.wall);
  // Header age alone would allow old 9.7/9.9 anchors; their receipt times expired.
  EXPECT_FALSE(f.read(10.1).snapshot);
  f.inputs.recordMap(mapMessage(10.3), f.wall);
  f.inputs.recordFootprint(footprint(10.3), f.wall);
  f.inputs.recordObservation("scan", header(10.3, "laser"), true, f.wall);
  EXPECT_TRUE(f.read(10.3).snapshot);
}

TEST(ExecutionSafetyInputs, RejectsConfigurationThatCouldDisableFreshness)
{
  auto o = options(); o.sources.clear();
        EXPECT_THROW(ExecutionSafetyInputs{o}, std::invalid_argument);
  o = options(); o.max_age = 0.; EXPECT_THROW(ExecutionSafetyInputs{o}, std::invalid_argument);
  o = options(); o.sources[0].max_age = 0.;
  EXPECT_THROW(ExecutionSafetyInputs{o}, std::invalid_argument);
  o = options(); o.sources.push_back(o.sources[0]);
  EXPECT_THROW(ExecutionSafetyInputs{o}, std::invalid_argument);
}
}  // namespace
}  // namespace pid_tracking_controller
