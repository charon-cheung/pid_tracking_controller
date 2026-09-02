# Legacy Differential Tracking Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the legacy differential `pid_tracking_controller` to a ROS 2 Jazzy Nav2 plugin, command a holonomic chassis with `vy = 0`, and log simulation-only baseline diagnostics without changing finite-input legacy control behavior.

**Architecture:** Keep the existing controller as the owner of `prune -> Lookahead -> carrot -> legacy controller -> regularization`. Replace only obsolete ROS/build integration, add numerical-invalid-state rejection, and log observational diagnostics that never feed the control law. Compile the two required legacy geometry sources directly into the plugin rather than preserving the obsolete nested build target.

**Tech Stack:** C++17, ROS 2 Jazzy, Nav2 `nav2_core::Controller`, `ament_cmake`, pluginlib, Eigen3, lifecycle publishers, GTest.

**Spec:** `docs/superpowers/specs/2026-09-02-legacy-differential-baseline-design.md`

## Global Constraints

- Modify only the `pid_tracking_controller` repository.
- Do not modify `nav2_common.yaml`, `webot_nav.py`, or any other repository.
- Do not create commits, branches, or worktrees.
- Keep `prune + erase`, persistent-state semantics, Lookahead, carrot definition, `theta_d`, `kappa`, both legacy control functions, fixed `pid_frequency / dt`, and both regularization formulas unchanged for normal finite inputs.
- Command `linear.x = legacy_v`, `linear.y = 0.0`, and `angular.z = legacy_w`.
- Do not add path append, collision checking, local avoidance, curvature limiting, Feedforward, a new yaw controller, or a terminal state machine.
- Only invalid-memory, zero-division, invalid-square-root, and `NaN/Inf` paths may change mathematical output.
- Keep `setSpeedLimit()` as a no-op in Phase 1.
- Install `param/jazzy/pid_tracking_controller.yaml` under `share/pid_tracking_controller/param/jazzy`.
- Use package-scoped incremental builds from `/home/CNF2026987648/s100_docker`; never create a temporary build tree or clean the existing build cache.
- Source `/opt/ros/jazzy/setup.bash` in every ROS command and use `MAKEFLAGS="-j12"` for builds.
- Limit implementation to at most seven explicit build invocations. Task 1 used
  three: one mechanical-compatibility failure, one successful build establishing
  the plugin-registry Red result, and one Green build. Tasks 2 and 3 must share
  the remaining four planned build invocations.

---

### Task 1: Create the Jazzy package and loadable plugin shell

**Files:**

- Create: `package.xml`
- Create: `pid_tracking_controller.xml`
- Modify: `CMakeLists.txt`
- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Modify: `src/pid_tracking_controller.cpp`
- Create: `test/test_plugin_loading.cpp`

**Interfaces:**

- Consumes: Jazzy `nav2_core::Controller` and the existing legacy controller implementation.
- Produces: plugin type `pid_tracking_controller::PidTrackingController`, exported under base type `nav2_core::Controller`.
- Produces lifecycle methods:

```cpp
void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
void activate() override;
void deactivate() override;
void cleanup() override;
```

- Keeps the public Nav2 methods `setPlan()`, `computeVelocityCommands()`, and `setSpeedLimit()`.

- [x] **Step 1: Replace the obsolete build metadata with a discoverable ament package**

Create `package.xml` with format 3 and these dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pid_tracking_controller</name>
  <version>0.1.0</version>
  <description>Legacy differential tracking baseline for a holonomic Nav2 chassis.</description>
  <maintainer email="CNF2025495045@todo.todo">RBS Navigation</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>angles</depend>
  <depend>eigen3_cmake_module</depend>
  <depend>eigen</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_core</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>nav_msgs</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/pid_tracking_controller.xml" />
  </export>
</package>
```

Rewrite `CMakeLists.txt` so the library sources are exactly:

```cmake
add_library(${PROJECT_NAME} SHARED
  src/pid_tracking_controller.cpp
  common/src/geometry/vec2d.cpp
  common/src/math/math_helper.cpp
)
```

Use `find_package(angles REQUIRED)`, `find_package(eigen3_cmake_module REQUIRED)`,
`find_package(Eigen3 REQUIRED)`, the ROS dependencies from `package.xml`,
`ament_target_dependencies()`, and
`target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)`. Add include paths for
`include` and `common/include`. Install the target and both include trees. Add a
`BUILD_TESTING` block with `ament_cmake_gtest`, `ament_lint_auto`, and the
plugin-loading test target. Do not export a plugin description yet; that
omission creates the first meaningful Red result.

- [x] **Step 2: Perform the mechanical Jazzy source adaptation needed to compile**

In the header:

- replace obsolete `nav2_core/controller.h` and private loader headers with Jazzy `.hpp` headers;
- remove `using namespace` from global header scope;
- change `configure()` to the lifecycle weak-pointer signature;
- replace the private `Msg::rcmsg_pub` member with:

```cpp
rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
  carrot_pub_;
```

- store `rclcpp::Logger logger_{rclcpp::get_logger("PidTrackingController")}` and the node clock;
- remove `nav_2d_utils::OdomSubscriber` ownership;
- give existing scalar state members their current source defaults so construction cannot leave arithmetic state indeterminate;
- add `friend class PidTrackingControllerTestAccess;` without adding test-only production methods.

In the source:

- lock the lifecycle node and throw `std::runtime_error` if it is unavailable;
- retain every existing parameter name and default;
- use `nav2_util::declare_parameter_if_not_declared` with the plugin-name prefix;
- replace `LOG_INFO` with event-level `RCLCPP_INFO` or remove per-cycle logs;
- use `angles::normalize_angle()` as the Jazzy equivalent of the removed
  `nav2_util::geometry_utils::normalize_angle()` call;
- create the carrot lifecycle publisher on relative topic `pid_tracking_controller/carrot`;
- activate, deactivate, and reset that publisher in the matching lifecycle methods;
- replace the class registration macro with:

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pid_tracking_controller::PidTrackingController,
  nav2_core::Controller)
```

- change regularization arguments from `nav_2d_msgs::msg::Twist2D` to
  `const geometry_msgs::msg::Twist &` and preserve the calculations by reading
  `hypot(linear.x, linear.y)` and `angular.z`;
- use the `velocity` argument in `computeVelocityCommands()` as the measured
  velocity source;
- assign `cmd_vel.twist.linear.y = 0.0` without adding any other control term;
- change `prune()` to read the already-stored `costmap_` pointer rather than
  calling through `costmap_ros_`; both pointers refer to the same costmap after
  `configure()`.

Do not add input validation or diagnostics in this task.

- [x] **Step 3: Write the plugin discovery test before exporting the XML**

Create `test/test_plugin_loading.cpp`:

```cpp
#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"

TEST(PidTrackingControllerPlugin, loadsFromNav2CoreRegistry)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  std::shared_ptr<nav2_core::Controller> controller;
  EXPECT_NO_THROW(
    controller = loader.createSharedInstance(
      "pid_tracking_controller::PidTrackingController"));
  ASSERT_NE(controller, nullptr);
}
```

The production change that makes this test pass is exporting the plugin
description so the real pluginlib registry can discover the class.

- [x] **Step 4: Run the first incremental build and verify the registry is still missing**

From `/home/CNF2026987648/s100_docker` run build invocation 1:

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
source install/setup.bash
colcon test --packages-select pid_tracking_controller \
  --ctest-args -R test_plugin_loading --output-on-failure
colcon test-result --verbose
```

Expected: compilation succeeds and `test_plugin_loading` fails with a
pluginlib class-not-found exception because the description is not exported.
This is the Red result for plugin discoverability.

- [x] **Step 5: Export the plugin description**

Create `pid_tracking_controller.xml`:

```xml
<library path="pid_tracking_controller">
  <class
    name="pid_tracking_controller::PidTrackingController"
    type="pid_tracking_controller::PidTrackingController"
    base_class_type="nav2_core::Controller">
    <description>
      Legacy differential path-tracking baseline commanding vx and wz with vy fixed to zero.
    </description>
  </class>
</library>
```

Add to `CMakeLists.txt`:

```cmake
pluginlib_export_plugin_description_file(
  nav2_core
  pid_tracking_controller.xml
)
```

- [x] **Step 6: Rebuild and verify plugin loading**

Run build invocation 2 and the targeted test:

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
source install/setup.bash
colcon test --packages-select pid_tracking_controller \
  --ctest-args -R test_plugin_loading --output-on-failure
colcon test-result --verbose
```

Expected: `test_plugin_loading` passes and instantiates a non-null controller.

- [x] **Step 7: Review checkpoint**

Run:

```bash
git diff --check
git status --short
```

Confirm every changed path is inside `pid_tracking_controller`, the controller
source remains below 1500 lines, and Task 1 contains only build/lifecycle
adaptation rather than algorithm improvements.

---

### Task 2: Lock legacy mathematics and fix circle-intersection invalid arithmetic

**Files:**

- Modify: `common/src/math/math_helper.cpp`
- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Create: `test/controller_test_access.hpp`
- Create: `test/test_math_helper.cpp`
- Create: `test/test_legacy_behavior.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**

- Consumes: existing `rmp::common::math::circleSegmentIntersection()` and public legacy controller methods.
- Produces: unchanged finite-input legacy outputs plus an empty intersection for invalid geometry.
- Produces test access only through `PidTrackingControllerTestAccess`, declared as a friend; no test-only public production API is added.

- [x] **Step 1: Add hand-derived legacy characterization tests**

Create `test/controller_test_access.hpp` in namespace
`pid_tracking_controller`. Give it static methods that set the existing private
legacy scalars, replace `global_plan_`, assign a non-owning `costmap_`, and read
the stored plan. It must not call alternative control formulas.

Create `test/test_legacy_behavior.cpp` with literal expectations. Include these
representative tests:

```cpp
TEST(LegacyBehavior, modelControllerKeepsHandDerivedBodyMapping)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setModelParameters(controller, 1.0, 0.2);

  const Eigen::Vector2d output = controller.UseModelPidControl(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.3, 0.1, 1.2),
    Eigen::Vector2d(0.6, -0.4));

  EXPECT_NEAR(output.x(), 0.3, 1e-12);
  EXPECT_NEAR(output.y(), 0.5, 1e-12);
}

TEST(LegacyBehavior, lookaheadKeepsSpeedTimeClamp)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setLookaheadParameters(
    controller, 0.5, 0.3, 0.9);

  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(0.0), 0.3);
  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(0.8), 0.4);
  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(3.0), 0.9);
}
```

Add a no-model test using `d_t = 0.1`, the repository PID gains, zero prior
error/integral state, `s = (0, 0, 0)`, `s_d = (0.01, 0, 0.01)`, and
`u_r = (0.05, 0)`. Assert the hand-derived first-cycle output
`v = 0.20005` and `w = 0.2` within `1e-10`.

Add regularization tests with literal results:

```cpp
geometry_msgs::msg::Twist measured;
measured.linear.x = 0.1;
EXPECT_DOUBLE_EQ(controller.linearRegularization(measured, 0.8), 0.6);

measured.angular.z = -0.1;
EXPECT_DOUBLE_EQ(controller.angularRegularization(measured, 0.2), 0.1);
```

These tests characterize existing finite behavior. They are expected to pass
once compiled and protect the later robustness edits from changing it.

- [x] **Step 2: Add geometry tests that expose invalid arithmetic**

Create `test/test_math_helper.cpp` with table-driven literal fixtures for
tangent, two-intersection, and no-intersection cases. Add these Red cases:

```cpp
TEST(CircleSegmentIntersection, zeroLengthSegmentReturnsNoPoints)
{
  const rmp::common::geometry::Vec2d point(0.5, 0.0);
  const auto intersections =
    rmp::common::math::circleSegmentIntersection(point, point, 1.0);
  EXPECT_TRUE(intersections.empty());
}

TEST(CircleSegmentIntersection, rejectedGeometryDoesNotRaiseInvalidFloatingPoint)
{
  std::feclearexcept(FE_ALL_EXCEPT);
  const auto intersections = rmp::common::math::circleSegmentIntersection(
    {-1.0, 2.0}, {1.0, 2.0}, 1.0);
  EXPECT_TRUE(intersections.empty());
  EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
}

TEST(CircleSegmentIntersection, tinyNegativeDiscriminantIsTreatedAsTangent)
{
  const double radius = std::nextafter(1.0, 0.0);
  const auto intersections = rmp::common::math::circleSegmentIntersection(
    {-1.0, 1.0}, {1.0, 1.0}, radius);
  ASSERT_EQ(intersections.size(), 1u);
  EXPECT_TRUE(std::isfinite(intersections.front().x()));
  EXPECT_TRUE(std::isfinite(intersections.front().y()));
}
```

For every non-empty result, loop over the returned points and assert both
coordinates are finite.

- [x] **Step 3: Register and run the Red geometry tests**

Register `test_math_helper` and `test_legacy_behavior` with
`ament_add_gtest()`, link both to `pid_tracking_controller`, and add their ROS
dependencies.

Run cumulative build invocation 4:

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
source install/setup.bash
colcon test --packages-select pid_tracking_controller \
  --ctest-args -R test_math_helper --output-on-failure
colcon test-result --verbose
```

Expected: the zero-length and floating-point-exception tests fail against the
legacy helper. Confirm the characterization test binary separately passes:

```bash
colcon test --packages-select pid_tracking_controller \
  --ctest-args -R test_legacy_behavior --output-on-failure
```

- [x] **Step 4: Implement the minimal numerical guard**

In `circleSegmentIntersection()` compute:

```cpp
const double dr2 = dx * dx + dy * dy;
if (!std::isfinite(dr2) || dr2 <= kMathEpsilon || !std::isfinite(r)) {
  return {};
}

const double determinant = x1 * y2 - x2 * y1;
double discriminant = r * r * dr2 - determinant * determinant;
const double discriminant_scale = std::max(
  {1.0, std::abs(r * r * dr2), std::abs(determinant * determinant)});
const double discriminant_tolerance = kMathEpsilon * discriminant_scale;
if (!std::isfinite(discriminant) || discriminant < -discriminant_tolerance) {
  return {};
}
discriminant = std::max(0.0, discriminant);
const double delta = std::sqrt(discriminant);
```

Keep the existing tangent/two-point formulas and point ordering. Before adding
an intersection to the result, require finite `x` and `y`. Remove the
`printf()` entirely.

- [x] **Step 5: Rebuild and verify geometry plus legacy equivalence**

Run cumulative build invocation 5:

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
source install/setup.bash
colcon test --packages-select pid_tracking_controller \
  --ctest-args -R 'test_math_helper|test_legacy_behavior' --output-on-failure
colcon test-result --verbose
```

Expected: all geometry and legacy characterization tests pass. A failure in a
legacy finite case must be fixed by restoring the original formula, not by
changing the expected literal.

- [x] **Step 6: Review checkpoint**

Inspect `git diff -- common/src/math/math_helper.cpp`. Confirm the only behavior
change is validation before square root/division and removal of `printf()`.

---

### Task 3: Add invalid-state semantics, fixed-zero vy, logged diagnostics, and standalone YAML

**Files:**

- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Modify: `src/pid_tracking_controller.cpp`
- Create: `param/jazzy/pid_tracking_controller.yaml`
- Create: `test/test_path_and_control.cpp`
- Create: `test/test_lifecycle_smoke.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**

- Consumes: a complete static `nav_msgs::msg::Path` from Nav2 `setPlan()`, current global-frame pose, and measured body `geometry_msgs::msg::Twist`.
- Produces: finite `TwistStamped` in `base_frame`, with `linear.y == 0.0`.
- Throws: `nav2_core::InvalidPath` for empty/non-finite paths and `nav2_core::NoValidControl` for non-finite runtime state or no finite carrot/command.
- Publishes: `pid_tracking_controller/carrot` as `PoseStamped`.
- Logs: all tracking measurements in a 1 Hz `PID_LOG` record.
- Installs: `share/pid_tracking_controller/param/jazzy/pid_tracking_controller.yaml`.

- [x] **Step 1: Write the path and command contract tests**

Create `test/test_path_and_control.cpp`. Use a real
`nav2_costmap_2d::Costmap2D(100, 100, 0.05, -2.5, -2.5)` assigned through
`PidTrackingControllerTestAccess`; do not mock path geometry.

Add these tests:

```cpp
TEST(PathValidation, nonFinitePathClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));

  auto invalid = makeStraightPath({0.0, 0.3});
  invalid.poses.back().pose.position.x =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(controller.setPlan(invalid), nav2_core::InvalidPath);
  EXPECT_TRUE(PidTrackingControllerTestAccess::plan(controller).empty());
}

TEST(PathValidation, emptyPathClearsOldPlanAndThrowsInvalidPath)
{
  PidTrackingController controller;
  controller.setPlan(makeStraightPath({0.0, 0.3}));
  EXPECT_THROW(controller.setPlan(nav_msgs::msg::Path()), nav2_core::InvalidPath);
  EXPECT_TRUE(PidTrackingControllerTestAccess::plan(controller).empty());
}
```

Add a one-point-path test calling `getLookAheadPoint()` and asserting the
carrot equals that point with finite `theta_trj` and `kappa == 0.0`.

Add a command test with a straight path at `x = {0.0, 0.3, 0.6}`, robot pose
at the origin, measured `linear.x = 0.4`, and test-configured
`max_v_inc = 0.1`. Assert the command uses the supplied measured velocity,
remains finite, and has:

```cpp
EXPECT_NEAR(command.twist.linear.x, 0.3, 1e-12);
EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
```

Add non-finite pose and velocity cases expecting `nav2_core::NoValidControl`.
Add a no-intersection/no-finite-carrot fixture expecting
`nav2_core::NoValidControl`.

- [x] **Step 2: Write the lifecycle output smoke test**

Create `test/test_lifecycle_smoke.cpp` using real ROS nodes:

```cpp
auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
  "pid_tracking_controller_test_node");
auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
  "pid_tracking_controller_test_costmap");
auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());

PidTrackingController controller;
controller.configure(parent, "PidTrackingController", tf, costmap);
controller.activate();
```

Create an observer node subscribed to `/pid_tracking_controller/carrot` and
capture the ROS logger output. Spin the parent and observer in a
`SingleThreadedExecutor`, wait up to one second for graph discovery, call
`setPlan()` and `computeVelocityCommands()`, then spin until the carrot callback
receives one message or one second expires. Assert:

```cpp
ASSERT_TRUE(carrot_received);
EXPECT_TRUE(std::isfinite(carrot.pose.position.x));
EXPECT_EQ(observer->count_publishers(
  "/pid_tracking_controller/tracking_diagnostics"), 0u);
EXPECT_NE(log_output.find("PID_LOG plan_revision=1"), std::string::npos);
```

Always call `deactivate()` and `cleanup()` before test teardown and call
`rclcpp::shutdown()` only after all ROS-owned objects have been destroyed.

- [x] **Step 3: Register the tests and verify the representative Red cases**

Register both new GTests and link them to the controller target. Add
`nav2_costmap_2d`, `rclcpp`, `rclcpp_lifecycle`, and `tf2_ros` test
dependencies.

Run cumulative build invocation 6:

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
source install/setup.bash
colcon test --packages-select pid_tracking_controller \
  --ctest-args -R 'test_path_and_control|test_lifecycle_smoke' --output-on-failure
colcon test-result --verbose
```

Expected: the non-finite path test fails because `setPlan()` accepts it, and the
lifecycle smoke test fails because `PID_LOG` does not exist and the old
diagnostics publisher remains. The empty-path
case belongs to the same invalid-path invariant group; do not repeatedly build
solely to manufacture a separate crash result.

- [x] **Step 4: Implement path and runtime validation without changing finite behavior**

Add anonymous-namespace finite helpers for scalar, quaternion, pose, twist, and
Eigen-vector validation. In `setPlan()`:

```cpp
global_plan_.clear();
if (path.poses.empty() || !pathIsFinite(path)) {
  throw nav2_core::InvalidPath(
          controller_plugin_name + " received an empty or non-finite path");
}
global_plan_ = path.poses;
```

Retain the existing goal-change comparison and PID-state reset after this
validation.

In `computeVelocityCommands()`:

- reject non-finite pose or velocity with `NoValidControl`;
- reject an empty stored/pruned plan with `InvalidPath`;
- compute `vt = hypot(velocity.linear.x, velocity.linear.y)` and
  `wt = velocity.angular.z`;
- keep the existing Lookahead, carrot, `theta_d`, control selection, and
  regularization sequence;
- reject non-finite carrot, `theta_trj`, `kappa`, raw Eigen output, or final
  twist with `NoValidControl`;
- set `header.stamp` from the controller clock, `header.frame_id` from
  `base_frame_`, and `linear.y = 0.0`.

In `getLookAheadPoint()`, preserve the legacy selected segment and intersection
ordering. If the helper returns no finite intersection for a branch that
requires one, throw `NoValidControl`; do not substitute a different carrot.

- [x] **Step 5: Add observational statistics without feeding the controller**

Add a private nested data-only `TrackingStatistics` struct with:

```cpp
std::size_t sample_count{0};
std::size_t linear_saturation_count{0};
std::size_t angular_saturation_count{0};
double sum_squared_lateral_error{0.0};
double maximum_absolute_lateral_error{0.0};
double traveled_distance{0.0};
double sum_actual_planar_speed{0.0};
std::vector<double> absolute_lateral_error_samples;
geometry_msgs::msg::Point previous_position;
bool previous_position_valid{false};
```

Add private timing state that forces the first `PID_LOG` record after activation
or a statistics reset.

After the final command has been validated, update statistics using a
diagnostic-only signed projection onto the first non-zero local segment from
the start of `prune_plan`. Clamp the projection to that segment. For a one-point
path, use positive Euclidean distance. Never pass this value into Lookahead,
carrot, or control functions.

Log at most once per second. Compute P95 from a copy using:

```cpp
const std::size_t index =
  static_cast<std::size_t>(std::ceil(0.95 * samples.size())) - 1u;
std::nth_element(samples.begin(), samples.begin() + index, samples.end());
const double p95 = samples[index];
```

Write one `RCLCPP_INFO` record prefixed with `PID_LOG` and these key/value
entries:

```text
plan_revision
sample_count
lateral_error
lateral_error_rms
lateral_error_p95_abs
lateral_error_max_abs
traveled_distance
average_actual_planar_speed
commanded_linear_speed
actual_linear_speed
commanded_angular_speed
actual_angular_speed
linear_saturation_ratio
angular_saturation_ratio
lookahead_distance
theta_trj
kappa
carrot_x
carrot_y
elapsed_tracking_time
```

Increment `plan_revision` on every accepted `setPlan()`. Reset aggregate
statistics only on the same terminal-goal change that resets the legacy PID
state.

- [x] **Step 6: Publish the exact control carrot**

After carrot validation and before returning the command, publish a
`PoseStamped` containing the carrot position and the already-computed
`theta_d`. Use the path frame and controller clock. Publish only while the
lifecycle publisher is active. Do not recompute or smooth the carrot for
visualization.

- [x] **Step 7: Add and install the standalone YAML**

Create `param/jazzy/pid_tracking_controller.yaml`:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 15.0
    failure_tolerance: 0.0
    publish_zero_velocity: true

    PidTrackingController:
      plugin: "pid_tracking_controller::PidTrackingController"
      goal_dist_tolerance: 0.05
      rotate_tolerance: 0.5
      base_frame: "base_link"
      map_frame: "map"
      lookahead_time: 0.5
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      max_v: 0.7
      min_v: 0.05
      max_v_inc: 0.5
      max_w: 0.2
      min_w: 0.0
      max_w_inc: 0.2
      k_v_p: 2.0
      k_v_i: 0.01
      k_v_d: 0.10
      k_w_p: 1.0
      k_w_i: 0.01
      k_w_d: 0.10
      k_theta: 0.5
      k: 1.0
      l: 0.2
      pid_frequency: 15.0
      use_model: true
```

Install it with:

```cmake
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)
```

- [x] **Step 8: Rebuild and run the complete package test set**

Run cumulative build invocation 7:

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
source install/setup.bash
colcon test --packages-select pid_tracking_controller --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all tests pass with no errors or warnings from the package tests.
Specifically confirm the lifecycle smoke receives the carrot, captures
`PID_LOG`, observes no diagnostics topic, and every command test observes
`linear.y == 0.0`.

Execution note: cumulative build invocation 7 succeeded with two generated ROS
message value-initialization warnings. The warnings were removed without
behavior changes and cumulative build invocation 8 verified a clean build.

- [x] **Step 9: Review checkpoint**

Review the full controller diff. For every changed arithmetic expression,
classify it as one of:

```text
unchanged legacy formula
ROS message field adaptation
finite-state validation
diagnostic-only computation
```

Remove or revert any arithmetic change that does not fit one of those classes.
Confirm there are no control-loop `printf()` calls and no per-cycle INFO logs.

---

### Task 4: Final package verification and handoff

**Files:**

- Verify: all modified and created files in `pid_tracking_controller`
- Verify installed artifact: `install/pid_tracking_controller/share/pid_tracking_controller/param/jazzy/pid_tracking_controller.yaml`
- Verify installed artifact: `install/pid_tracking_controller/share/pid_tracking_controller/pid_tracking_controller.xml`
- Verify installed artifact: `install/pid_tracking_controller/lib/libpid_tracking_controller.so`

**Interfaces:**

- Consumes: the completed package and existing workspace build/install trees.
- Produces: evidence that the package is loadable, tested, scoped correctly, and ready for the user's separate navigation integration.

- [ ] **Step 1: Ensure navigation is not running before ROS verification**

Inspect processes without stopping anything:

```bash
ps -eo pid,args | rg 'controller_server|bt_navigator|webot.*nav|rbs_navigation' || true
```

If and only if related RBS navigation processes are present, run:

```bash
cd /home/CNF2026987648/s100_docker/src/webots_ros2_demo
./kill_ros.sh --rbs
```

Do not run the kill script when the process check is empty.

- [ ] **Step 2: Run tests without another build invocation**

```bash
cd /home/CNF2026987648/s100_docker
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select pid_tracking_controller --event-handlers console_direct+
colcon test-result --verbose
```

Expected: every `pid_tracking_controller` test passes.

- [ ] **Step 3: Verify installed artifacts and plugin registry**

```bash
test -f install/pid_tracking_controller/share/pid_tracking_controller/param/jazzy/pid_tracking_controller.yaml
test -f install/pid_tracking_controller/share/pid_tracking_controller/pid_tracking_controller.xml
test -f install/pid_tracking_controller/lib/libpid_tracking_controller.so
ros2 pkg prefix pid_tracking_controller
```

Run the plugin-loading test once more through CTest if pluginlib discovery was
not included in the preceding test output.

- [ ] **Step 4: Review scope, whitespace, and file growth**

From the package repository run:

```bash
git diff --check
git diff --stat
git status --short
wc -l src/pid_tracking_controller.cpp \
  include/pid_tracking_controller/pid_tracking_controller.hpp
```

Confirm:

- no file outside `pid_tracking_controller` changed;
- neither controller file is oversized;
- the controller source did not grow by unrelated refactoring;
- no README, license, or contribution file changed;
- no temporary logs, generated measurements, or build artifacts are tracked;
- the YAML installs under `share/pid_tracking_controller/param/jazzy`;
- all Phase 1 Non-goals remain absent.

- [ ] **Step 5: Prepare the implementation report**

Report:

- files modified and created;
- that the retained responsibility is a legacy differential baseline with
  `vy = 0`;
- exact diff scope and whether either source file crossed 1500 lines;
- the actual build invocation count and package tests executed;
- plugin loading and installed-YAML evidence;
- that runtime integration into `nav2_common.yaml` and `webot_nav.py` remains
  user-owned;
- remaining simulation risk: the repository defaults, especially
  `max_w = 0.2`, may need experimental tuning after the baseline is recorded,
  but no tuning belongs in this implementation.
