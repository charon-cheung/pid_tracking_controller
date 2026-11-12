#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "direct_costmap_fixture.hpp"
#include <future>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
  o.sources = {{"scan", .6, "", ""}};
  o.budget_seconds = 1.;  // Functional tests must not depend on machine load.
  return o;
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
  nav2_costmap_2d::LayeredCostmap map{"map", false, false};
  std::shared_ptr<TestCostmapLayer> layer{std::make_shared<TestCostmapLayer>()};

  Fixture()
  {
    map.resizeMap(60, 60, .1, -3., -3.);
    map.addPlugin(layer); map.updateMap(0., 0., 0.);
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
    inputs.recordFootprint(footprint(t - .1), wall);
    inputs.recordObservation("scan", header(t - .3, "laser"), true, wall);
    inputs.recordObservation("scan", header(t - .1, "laser"), true, wall);
  }
  SafetyInputResult read(double t = 10.)
  {
    return inputs.capture(pose(t), rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME), tf, map,
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
  auto & live = *f.map.getCostmap();
  live.resizeMap(20, 30, .1, -1., -3.);
  std::fill_n(live.getCharMap(), 600, 254);
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
  f.inputs.recordFootprint(footprint(10.8), f.wall);
  auto result = f.read(10.8);
  EXPECT_FALSE(result.snapshot);
  EXPECT_EQ(result.source, "scan");
  // A new scan after the gap alone cannot certify the already published map.
  f.inputs.recordObservation("scan", header(10.8, "laser"), true, f.wall);
  EXPECT_FALSE(f.read(10.8).snapshot);
  f.inputs.recordFootprint(footprint(11.), f.wall);
  f.inputs.recordObservation("scan", header(11., "laser"), true, f.wall);
  EXPECT_TRUE(f.read(11.).snapshot);
}

TEST(ExecutionSafetyInputs, EveryConfiguredSourceIsRequired)
{
  Fixture f;
  auto o = options(); o.sources.push_back({"cloud", .6, "", ""});
  ExecutionSafetyInputs inputs(o);
  inputs.recordFootprint(footprint(), f.wall);
  inputs.recordObservation("scan", header(9.7, "laser"), true, f.wall);
  auto result = inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.map, f.wall, f.wall);
  EXPECT_FALSE(result.snapshot); EXPECT_EQ(result.source, "cloud");
  inputs.recordObservation("cloud", header(9.7, "laser"), true, f.wall);
  EXPECT_TRUE(inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.map, f.wall,
        f.wall).snapshot);
}

TEST(ExecutionSafetyInputs, FrozenRosClockStillExpiresBySteadyReceiptAge)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  f.wall += std::chrono::seconds(1);
  EXPECT_FALSE(f.read().snapshot);
  // Replayed duplicate headers cannot acquire fresh receipt times.
  f.inputs.recordFootprint(footprint(), f.wall);
  f.inputs.recordObservation("scan", header(9.9, "laser"), true, f.wall);
  EXPECT_FALSE(f.read().snapshot);
  f.fill(11.); EXPECT_TRUE(f.read(11.).snapshot);
}

TEST(ExecutionSafetyInputs, EachStreamMustBeFreshInRosAndSteadyTime)
{
  for (int stream = 0; stream < 2; ++stream) {
    Fixture f; f.fill();
    const auto old = f.wall - std::chrono::seconds(1);
    if (stream == 0) {f.inputs.recordFootprint(footprint(10.), old);}
    if (stream == 1) {f.inputs.recordObservation("scan", header(10., "laser"), true, old);}
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

TEST(ExecutionSafetyInputs, InvalidLiveMapGeometryNeverReusesAnOldSnapshot)
{
  for (int bad = 0; bad < 4; ++bad) {
    Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
    auto & map = *f.map.getCostmap();
    switch (bad) {
      case 0: map.resizeMap(0, 60, .1, -3., -3.); break;
      case 1: map.resizeMap(60, 60, 0., -3., -3.); break;
      case 2: map.resizeMap(2000, 1000, .1, -3., -3.); break;
      case 3: map.resizeMap(60, 60, .1, std::numeric_limits<double>::quiet_NaN(), -3.); break;
    }
    EXPECT_EQ(f.read().reason, "invalid_map_geometry") << bad;
  }
}

TEST(ExecutionSafetyInputs, InitializedAndCurrentMapStillRequiresFreshUpdateWitness)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  f.layer->setCurrent(false);
  EXPECT_EQ(f.read().reason, "map_not_current");
  f.layer->setCurrent(true);
  f.inputs.recordObservation("scan", header(10.8, "laser"), true, f.wall);
  // Polling a current, unchanged live map cannot refresh the stopped update stream.
  EXPECT_EQ(f.read(10.8).reason, "footprint_missing_or_stale");
  nav2_costmap_2d::LayeredCostmap uninitialized("map", false, false);
  uninitialized.resizeMap(60, 60, .1, -3., -3.);
  f.inputs.recordFootprint(footprint(10.8), f.wall);
  EXPECT_EQ(f.inputs.capture(pose(10.8), rclcpp::Time(10800000000LL, RCL_ROS_TIME),
      f.tf, uninitialized, f.wall, f.wall).reason, "map_not_current");
}

TEST(ExecutionSafetyInputs, BusyLiveMapReturnsUnavailableWithoutWaitingForTheUpdater)
{
  Fixture f; f.fill();
  std::promise<void> locked, release;
  auto release_signal = release.get_future();
  std::thread writer([&]() {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*f.map.getCostmap()->getMutex());
      locked.set_value(); release_signal.wait_for(std::chrono::seconds(1));
    });
  locked.get_future().wait();
  const auto result = f.read();
  release.set_value(); writer.join();
  EXPECT_EQ(result.reason, "map_busy");
  EXPECT_TRUE(f.read().snapshot);
}

// Hold the real map mutex on another thread; the recursive mutex cannot be
// contended by locking it from the capture thread itself.
struct BusyMap
{
  std::promise<void> locked, release;
  std::future<void> released{release.get_future()};
  std::thread writer;
  explicit BusyMap(nav2_costmap_2d::LayeredCostmap & map)
  : writer([&]() {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getCostmap()->getMutex());
      locked.set_value(); released.wait();
    }) {locked.get_future().wait();}
  ~BusyMap() {release.set_value(); writer.join();}
};

TEST(ExecutionSafetyInputs, BusyMapReusesUnexpiredGeometryButUsesTheCurrentPose)
{
  Fixture f; f.fill();
  f.map.getCostmap()->setCost(30, 30, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto first = f.read(); ASSERT_TRUE(first.snapshot);
  // A consumer must not be able to modify the retained map.
  first.snapshot->map->setCost(30, 30, 0);
  f.map.getCostmap()->setCost(30, 30, 0);
  auto body = footprint(10.); body->polygon.points[0].x = -.8;
  f.inputs.recordFootprint(body, f.wall);
  f.inputs.recordObservation("scan", header(10., "laser"), true, f.wall);
  {
    BusyMap busy(f.map);
    auto robot = pose(10.1); robot.pose.position.x = 1.;
    auto reused = f.inputs.capture(robot, rclcpp::Time(10100000000LL, RCL_ROS_TIME),
      f.tf, f.map, f.wall, f.wall);
    ASSERT_TRUE(reused.snapshot) << reused.reason;
    EXPECT_DOUBLE_EQ(reused.snapshot->pose.x, 1.);
    EXPECT_EQ(reused.snapshot->pose_stamp_ns, 10100000000LL);
    EXPECT_EQ(reused.snapshot->header.stamp, header(9.9).stamp);
    EXPECT_NEAR(reused.snapshot->footprint[0].x, -.3, 1e-6);
    EXPECT_EQ(reused.snapshot->map->getCost(30, 30), nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  auto updated = f.read(10.1); ASSERT_TRUE(updated.snapshot);
  EXPECT_EQ(updated.snapshot->header.stamp, header(10.).stamp);
  EXPECT_EQ(updated.snapshot->map->getCost(30, 30), 0u);
  EXPECT_NEAR(updated.snapshot->footprint[0].x, -.8, 1e-6);
}

TEST(ExecutionSafetyInputs, ReuseCannotRenewMapAgeInRosOrSteadyTime)
{
  for (bool wall_expiry : {false, true}) {
    Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
    BusyMap busy(f.map);
    f.inputs.recordFootprint(footprint(10.1), f.wall);
    f.inputs.recordObservation("scan", header(10.1, "laser"), true, f.wall);
    ASSERT_TRUE(f.read(10.2).snapshot);
    // New footprint/scan arrivals cannot refresh the age of the retained grid.
    if (wall_expiry) {f.wall += std::chrono::milliseconds(601);}
    const double now = wall_expiry ? 10.3 : 10.6;
    f.inputs.recordFootprint(footprint(now - .1), f.wall);
    f.inputs.recordObservation("scan", header(now, "laser"), true, f.wall);
    auto result = f.read(now);
    EXPECT_FALSE(result.snapshot);
    EXPECT_EQ(result.reason, "cached_map_stale");
  }
}

TEST(ExecutionSafetyInputs, CachedMapDoesNotBypassObservationPoseOrTfValidation)
{
  for (int bad = 0; bad < 3; ++bad) {
    Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
    BusyMap busy(f.map);
    auto robot = pose();
    if (bad == 0) {f.inputs.recordObservation("scan", header(10., "laser"), false, f.wall);}
    if (bad == 1) {robot.header.stamp = header(9.).stamp;}
    if (bad == 2) {robot.header.frame_id = "missing";}
    EXPECT_FALSE(f.inputs.capture(robot, rclcpp::Time(10, 0, RCL_ROS_TIME),
        f.tf, f.map, f.wall, f.wall).snapshot) << bad;
  }
}

TEST(ExecutionSafetyInputs, ResetClockRegressionOrInvalidMapFencesCachedGeometry)
{
  for (int invalidation = 0; invalidation < 3; ++invalidation) {
    Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
    if (invalidation == 0) {f.inputs.reset(); f.fill(10.1);}
    if (invalidation == 1) {EXPECT_FALSE(f.read(9.).snapshot); f.fill(10.1);}
    if (invalidation == 2) {
      f.layer->setCurrent(false); EXPECT_FALSE(f.read().snapshot);
      f.layer->setCurrent(true);
    }
    BusyMap busy(f.map);
    EXPECT_FALSE(f.read(10.1).snapshot) << invalidation;
  }
}

TEST(ExecutionSafetyInputs, InvalidFootprintsAndPosesAreUnavailable)
{
  for (int bad = 0; bad < 5; ++bad) {
    Fixture f; f.fill(); auto body = footprint(10.);
    if (bad == 0) {body->polygon.points.clear();}
    if (bad == 1) {body->polygon.points[0].x = std::numeric_limits<float>::quiet_NaN();}
    if (bad == 2) {
      for (auto & p : body->polygon.points) {
        p.y = 0.;
      }
    }
    if (bad == 3) {body->header.frame_id = "unconnected";}
    if (bad == 4) {body->header.stamp = header(11.).stamp;}
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
    EXPECT_FALSE(f.inputs.capture(p, rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.map, f.wall,
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
  auto result = f.inputs.capture(p, rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.map, f.wall, f.wall);
  ASSERT_TRUE(result.snapshot) << result.reason;
  EXPECT_NEAR(result.snapshot->pose.x, 1., 1e-9);
  EXPECT_NEAR(result.snapshot->pose.y, 2.5, 1e-9);
  EXPECT_NEAR(result.snapshot->pose.yaw, M_PI / 2, 1e-9);
  EXPECT_NEAR(result.snapshot->footprint[0].x, .3, 1e-6);
  EXPECT_NEAR(result.snapshot->footprint[0].y, -.3, 1e-6);
}

TEST(ExecutionSafetyInputs, SnapshotFeedsTheExistingSweepWithoutChangingUnknownCells)
{
  Fixture f; f.fill(); auto & live = *f.map.getCostmap();
  live.setCost(30, 30, nav2_costmap_2d::NO_INFORMATION);
  auto result = f.read(); ASSERT_TRUE(result.snapshot);
  ExecutionSafetyOptions o; o.budget_seconds = 1.; ExecutionSafety sweep(o);
  CommandSweepInput command{result.snapshot->pose, {}, 0., {.5, .5, 1.}};
  auto checked = sweep.checkCommand(command, *result.snapshot->map, result.snapshot->footprint);
  EXPECT_EQ(checked.status, SweepStatus::kUnavailable);
  live.setCost(30, 30, nav2_costmap_2d::LETHAL_OBSTACLE);
  result = f.read(10.1); ASSERT_TRUE(result.snapshot);
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
    inputs.recordFootprint(footprint(), f.wall);
    inputs.recordObservation("scan", header(9.7, "laser"), true, f.wall);
    EXPECT_FALSE(inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.map, f.wall,
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
        auto & map = *f.map.getCostmap();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
        const auto x = i % 2 ? 20u : 60u;
        map.resizeMap(x, 60, .1, -3., -3.);
        std::fill_n(map.getCharMap(), x * 60, x);
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
  f.inputs.recordFootprint(footprint(10.2), f.wall);
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
  auto result = f.inputs.capture(pose(), rclcpp::Time(10, 0, RCL_ROS_TIME), f.tf, f.map, f.wall, later);
  EXPECT_FALSE(result.snapshot);
  EXPECT_EQ(result.reason, "pose_invalid_or_stale");
}

TEST(ExecutionSafetyInputs, ResumedRosClockCannotReusePrePauseObservationAnchors)
{
  Fixture f; f.fill(); ASSERT_TRUE(f.read().snapshot);
  f.wall += std::chrono::seconds(10);
  f.inputs.recordFootprint(footprint(10.1), f.wall);
  f.inputs.recordObservation("scan", header(10.1, "laser"), true, f.wall);
  // Header age alone would allow old 9.7/9.9 anchors; their receipt times expired.
  EXPECT_FALSE(f.read(10.1).snapshot);
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

class DirectInputSubscriptionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("direct_input_controller");
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("direct_input_map");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("global_frame", "map"));
    map->set_parameter(rclcpp::Parameter("robot_base_frame", "base"));
    map->configure();
    map->declare_parameter("obstacle.enabled", true);
    map->declare_parameter("obstacle.observation_sources", "scan duplicate cloud");
    for (const std::string source : {"scan", "duplicate", "cloud"}) {
      const auto key = "obstacle." + source + ".";
      const bool cloud = source == "cloud";
      map->declare_parameter(key + "topic", cloud ? "cloud" : "scan");
      map->declare_parameter(key + "data_type", cloud ? "PointCloud2" : "LaserScan");
      map->declare_parameter(key + "expected_update_rate", 0.);
      map->declare_parameter(key + "marking", true);
      map->declare_parameter(key + "clearing", true);
    }
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"obstacle"}));
    map->getCostmap()->resizeMap(60, 60, .1, -3., -3.);
    map->getLayeredCostmap()->addPlugin(std::make_shared<TestCostmapLayer>());
    map->getLayeredCostmap()->updateMap(0., 0., 0.);
    publisher = std::make_shared<rclcpp::Node>("direct_input_publisher");
    const auto topic = [&](const char * name) {
        return map->get_node_topics_interface()->resolve_topic_name(name);
      };
    body_pub = publisher->create_publisher<geometry_msgs::msg::PolygonStamped>(topic("published_footprint"), 1);
    scan_pub = publisher->create_publisher<sensor_msgs::msg::LaserScan>(topic("scan"), rclcpp::SensorDataQoS());
    cloud_pub = publisher->create_publisher<sensor_msgs::msg::PointCloud2>(topic("cloud"), rclcpp::SensorDataQoS());
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map"; transform.header.stamp = node->now();
    transform.child_frame_id = "base"; transform.transform.rotation.w = 1.;
    tf->setTransform(transform, "test", true);
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node->get_node_base_interface());
    connect();
  }
  void TearDown() override
  {
    inputs.reset(); executor.reset();
    body_pub.reset(); scan_pub.reset(); cloud_pub.reset(); publisher.reset();
    tf.reset(); map->cleanup(); map.reset(); node.reset(); rclcpp::shutdown();
  }
  void connect()
  {
    inputs = std::make_shared<ExecutionSafetyInputs>(safetyInputOptions(*map));
    inputs->subscribe(node, map);
  }
  SafetyInputResult read()
  {
    auto robot = pose(); robot.header.stamp = node->now();
    const auto received = Wall::now();
    return inputs->capture(robot, node->now(), *tf, *map->getLayeredCostmap(), received);
  }
  void feed(bool cloud = true, int fault = 0)
  {
    auto body = footprint(); body->header.stamp = node->now(); body_pub->publish(*body);
    sensor_msgs::msg::LaserScan scan; scan.header = body->header;
    scan.angle_max = .1; scan.angle_increment = .1; scan.range_min = .1; scan.range_max = 10.;
    scan.ranges = {1., 2.};
    if (fault == 1) {scan.time_increment = std::numeric_limits<float>::quiet_NaN();}
    if (fault == 2) {scan.ranges.assign(2, std::numeric_limits<float>::quiet_NaN());}
    if (fault == 3) {scan.ranges.assign(2, std::numeric_limits<float>::infinity());}
    scan_pub->publish(scan);
    if (cloud) {
      sensor_msgs::msg::PointCloud2 points; points.header = body->header;
      points.height = 1; points.point_step = 12;
      for (int i = 0; i < 3; ++i) {
        sensor_msgs::msg::PointField field;
        field.name = std::string(1, "xyz"[i]); field.offset = i * 4;
        field.count = 1; field.datatype = field.FLOAT32; points.fields.push_back(field);
      }
      cloud_pub->publish(points);
    }
  }
  bool wait(const std::string & reason, bool cloud = true, int fault = 0, const std::string & source = "")
  {
    const auto deadline = Wall::now() + std::chrono::seconds(3);
    while (Wall::now() < deadline) {
      feed(cloud, fault); executor->spin_some();
      const auto result = read();
      if (result.reason == reason && (source.empty() || result.source == source)) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return false;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  rclcpp::Node::SharedPtr publisher;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<ExecutionSafetyInputs> inputs;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr body_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
};

TEST_F(DirectInputSubscriptionTest, AllSourcesAreRequiredWithoutFullCostmapPublication)
{
  ASSERT_TRUE(wait("observation_missing_or_stale", false, 0, "obstacle.cloud"));
  EXPECT_EQ(read().source, "obstacle.cloud");
  ASSERT_TRUE(wait("ready"));
  // No costmap message was published. Changing the owned map must change the snapshot.
  map->getCostmap()->setCost(30, 30, 254);
  auto result = read(); ASSERT_TRUE(result.snapshot) << result.reason;
  EXPECT_EQ(result.snapshot->map->getCost(30, 30), 254);
  EXPECT_EQ(scan_pub->get_subscription_count(), 1u);
  std::this_thread::sleep_for(std::chrono::milliseconds(650));
  EXPECT_FALSE(read().snapshot);
}

TEST_F(DirectInputSubscriptionTest, InvalidSensorPayloadAndConfigurationChangesFailClosed)
{
  ASSERT_TRUE(wait("ready"));
  for (int fault : {1, 2}) {
    ASSERT_TRUE(wait("observation_missing_or_stale", true, fault));
    EXPECT_EQ(read().source, "obstacle.scan");
    ASSERT_TRUE(wait("ready"));
  }
  ASSERT_TRUE(wait("ready", true, 3));
  map->set_parameter(rclcpp::Parameter("obstacle.enabled", false));
  {
    BusyMap busy(*map->getLayeredCostmap());
    EXPECT_EQ(read().reason, "configuration_changed_reactivate");
  }
  map->set_parameter(rclcpp::Parameter("obstacle.enabled", true));
  inputs.reset(); connect();
  EXPECT_FALSE(read().snapshot);
  ASSERT_TRUE(wait("ready"));
}

TEST_F(DirectInputSubscriptionTest, PathRiskMustMatchCurrentLoadAndExpiresAcrossReactivation)
{
  using Risk = msg::PathRiskState;
  msg::TrackingProgress current;
  current.valid = true; current.controller_instance = "test";
  current.plan_revision = 1; current.path_fingerprint = 123;
  current.path_header.frame_id = "map"; current.loaded_at = node->now();
  EXPECT_EQ(inputs->pathRisk(current, node->now()).state, Risk::UNAVAILABLE);
  for (auto state : {Risk::CLEAR, Risk::CLEARANCE_LOW, Risk::COLLISION_AHEAD, Risk::CLEAR}) {
    current.header.stamp = node->now();
    auto risk = std::make_shared<Risk>();
    risk->header = current.header; risk->header.frame_id = "map";
    risk->path_header = current.path_header; risk->controller_instance = current.controller_instance;
    risk->plan_revision = current.plan_revision; risk->path_fingerprint = current.path_fingerprint;
    risk->progress_stamp = risk->map_stamp = risk->header.stamp; risk->state = state;
    inputs->recordPathRisk(risk);
    EXPECT_EQ(inputs->pathRisk(current, node->now()).state, state);
  }
  ++current.plan_revision;
  EXPECT_EQ(inputs->pathRisk(current, node->now()).state, Risk::UNAVAILABLE);
  --current.plan_revision;
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  current.header.stamp = node->now();
  EXPECT_EQ(inputs->pathRisk(current, node->now()).state, Risk::UNAVAILABLE);
  inputs.reset(); connect();
  EXPECT_EQ(inputs->pathRisk(current, node->now()).state, Risk::UNAVAILABLE);
}
}  // namespace
}  // namespace pid_tracking_controller
