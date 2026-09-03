# Phase 1.4 Ordered Path Progress and Observable Carrot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent spatially nearby future path branches from taking over tracking, generate the carrot by ordered path arc length, use the same projection for lateral diagnostics, and publish the exact carrot as `geometry_msgs/msg/PointStamped`.

**Architecture:** Add an implementation-private, stateful `PathProgressTracker` that owns ordered projection, a motion-funded forward-progress budget, arc-length carrot interpolation, and signed lateral error. `PidTrackingController` orchestrates this component and passes its carrot to the unchanged Phase 1.3 velocity planner. The old destructive nearest-point pruning and Euclidean-radius carrot selection leave the production path.

**Tech Stack:** C++17, ROS 2 Jazzy, Nav2 controller plugin API, `geometry_msgs`, GoogleTest, `ament_cmake`, `colcon`.

**Spec:** `docs/superpowers/specs/2026-09-03-phase-1-4-path-progress-and-carrot-design.md`

## Global Constraints

- Modify only `pid_tracking_controller`.
- Do not change Phase 1.3 velocity equations, speed parameters, regularization, forward-only behavior, start-heading alignment, or FinalApproach handoff.
- Do not add dynamic path append, obstacle checking, reverse tracking, or a new terminal-state machine.
- Keep `/pid_tracking_controller/carrot` as the topic name and change only its message type to `geometry_msgs/msg/PointStamped`.
- Do not create commits, branches, or worktrees.
- Before each ROS build or test command, source `/opt/ros/jazzy/setup.bash`; source `install/setup.bash` when using workspace artifacts.
- Use the existing workspace `build/`, `install/`, and `log/` directories and build only `pid_tracking_controller`.
- Do not read the navigation test log; runtime simulation acceptance belongs to the user.

---

### Task 1: Ordered Path Progress and Arc-Length Carrot Component

**Files:**
- Create: `src/path_progress_tracker.hpp`
- Create: `src/path_progress_tracker.cpp`
- Create: `test/test_path_progress_tracker.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: a complete ordered `nav_msgs::msg::Path`, a finite robot `geometry_msgs::msg::Point`, and a finite nonnegative lookahead distance.
- Produces:

```cpp
struct PathProgressReference
{
  geometry_msgs::msg::Point projection;
  geometry_msgs::msg::Point carrot;
  std::string frame_id;
  std::size_t segment_index{0};
  double segment_fraction{0.0};
  double progress_arc_length{0.0};
  double carrot_arc_length{0.0};
  double tangent{0.0};
  double curvature{0.0};
  double signed_lateral_error{0.0};
};

class PathProgressTracker
{
public:
  void setPath(const nav_msgs::msg::Path & path);
  PathProgressReference update(
    const geometry_msgs::msg::Point & robot_position,
    double lookahead_distance);
  bool hasPath() const;
};
```

- Invariant: within one installed path, `progress_arc_length` never decreases and cannot repeatedly increase when `robot_position` does not change.
- Forward budget: initialize and cap it at twice the median finite nonzero pose spacing; replenish it only by finite planar robot displacement and consume accepted arc-length progress.

- [ ] **Step 1: Register the new source and test target**

Add `src/path_progress_tracker.cpp` to the existing shared library and add:

```cmake
ament_add_gtest(test_path_progress_tracker
  test/test_path_progress_tracker.cpp
)
target_include_directories(test_path_progress_tracker PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(test_path_progress_tracker
  geometry_msgs
  nav_msgs
)
target_link_libraries(test_path_progress_tracker
  ${PROJECT_NAME}
)
```

- [ ] **Step 2: Write failing ordered-progress tests**

Create literal path fixtures and tests with these observable assertions:

```cpp
TEST(PathProgressTracker, futureNearbyBranchCannotReplaceCurrentOrderedBranch)
{
  PathProgressTracker tracker;
  tracker.setPath(makeDenseHairpinPath());

  for (double x = 0.0; x <= 0.40; x += 0.05) {
    tracker.update(makePoint(x, 0.0), 0.10);
  }
  const auto reference = tracker.update(makePoint(0.40, 0.08), 0.10);

  EXPECT_NEAR(reference.projection.y, 0.0, 1e-12);
  EXPECT_LT(reference.segment_index, first_return_branch_segment_index);
  EXPECT_GT(reference.carrot.x, reference.projection.x);
}

TEST(PathProgressTracker, unchangedRobotPoseCannotRatchetProgressForward)
{
  PathProgressTracker tracker;
  tracker.setPath(makeDenseHairpinPath());
  tracker.update(makePoint(0.40, 0.0), 0.10);

  const double first = tracker.update(
    makePoint(0.40, 0.08), 0.10).progress_arc_length;
  for (int sample = 0; sample < 20; ++sample) {
    EXPECT_DOUBLE_EQ(
      tracker.update(makePoint(0.40, 0.08), 0.10).progress_arc_length,
      first);
  }
}
```

`makeDenseHairpinPath()` must use literal 0.05 m spacing: travel from `(0,0)` to
`(1,0)`, turn through points near `x=1`, then return toward `(0,0.10)`. The robot
at `(0.40,0.08)` is closer to the future return branch but still belongs to the
outbound branch.

- [ ] **Step 3: Write failing arc-length, diagnostics, and degenerate-path tests**

Add tests that independently derive expected geometry:

```cpp
TEST(PathProgressTracker, carrotAdvancesByPathArcLength)
{
  PathProgressTracker tracker;
  tracker.setPath(makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}));

  const auto reference = tracker.update(makePoint(0.0, 0.0), 1.20);

  EXPECT_NEAR(reference.carrot.x, 1.0, 1e-12);
  EXPECT_NEAR(reference.carrot.y, 0.20, 1e-12);
  EXPECT_NEAR(reference.carrot_arc_length, 1.20, 1e-12);
}

TEST(PathProgressTracker, lateralErrorUsesAcceptedProgressSegment)
{
  PathProgressTracker tracker;
  tracker.setPath(makeDenseHairpinPath());
  for (double x = 0.0; x <= 0.40; x += 0.05) {
    tracker.update(makePoint(x, 0.0), 0.10);
  }
  const auto reference = tracker.update(makePoint(0.40, 0.08), 0.10);

  EXPECT_NEAR(reference.signed_lateral_error, 0.08, 1e-12);
}

TEST(PathProgressTracker, onePointPathReturnsFiniteStationaryReference)
{
  PathProgressTracker tracker;
  tracker.setPath(makePath({{0.2, -0.1}}));
  const auto reference = tracker.update(makePoint(0.2, -0.1), 0.30);

  EXPECT_DOUBLE_EQ(reference.carrot.x, 0.2);
  EXPECT_DOUBLE_EQ(reference.carrot.y, -0.1);
  EXPECT_DOUBLE_EQ(reference.progress_arc_length, 0.0);
  EXPECT_DOUBLE_EQ(reference.signed_lateral_error, 0.0);
}
```

Also assert that empty paths, non-finite pose coordinates, and non-finite or negative
lookahead inputs throw `std::invalid_argument` or `std::domain_error` before producing
a reference.

- [ ] **Step 4: Build once and verify the new tests fail for the missing component**

Run from `/home/CNF2026987648/s100_docker`:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller \
  --cmake-args -DBUILD_TESTING=ON
```

Expected: compilation fails because `path_progress_tracker.hpp` or its declared
implementation is missing. This is the representative RED cycle for the component.

- [ ] **Step 5: Implement path validation and cumulative geometry**

Implement `setPath()` so it:

```cpp
if (path.empty()) {
  throw std::invalid_argument("path must not be empty");
}
```

Then validate every pose `x/y`, retain `path.header.frame_id` as the publication
frame, falling back to the first pose frame when the path header is empty. Build
cumulative segment lengths while allowing zero-length segments, compute the median of
finite nonzero segment lengths, reset progress to zero, clear the previous robot
position, and initialize the forward budget to twice that median spacing. Treat a path
with no nonzero segment as a single-point path with zero total length and zero budget.

- [ ] **Step 6: Implement continuity-constrained projection**

In `update()`:

```cpp
if (previous_robot_position_valid_) {
  progress_budget_ = std::min(
    maximum_progress_budget_,
    progress_budget_ + planarDistance(robot_position, previous_robot_position_));
}
const double admissible_end = std::min(
  total_path_length_, progress_arc_length_ + progress_budget_);
```

Project the robot onto each nonzero segment portion intersecting
`[progress_arc_length_, admissible_end]`. Clamp each projection to that interval,
select the projection with the smallest squared Euclidean distance, and prefer the
smaller arc length on exact ties. Accept only monotonic progress:

```cpp
const double advance = candidate_arc_length - progress_arc_length_;
progress_arc_length_ = candidate_arc_length;
progress_budget_ = std::max(0.0, progress_budget_ - advance);
```

Always update `previous_robot_position_` after a finite result. A repeated pose adds
zero budget and therefore cannot expose progressively later segments.

- [ ] **Step 7: Implement arc-length interpolation and signed lateral error**

Use:

```cpp
const double carrot_s = std::min(
  progress_arc_length_ + lookahead_distance, total_path_length_);
```

Interpolate `projection` and `carrot` on their containing original segments. Derive
the tangent with `std::atan2(dy, dx)`. Derive signed lateral error from the projection
segment cross product:

```cpp
const double cross = segment_dx * robot_relative_y -
  segment_dy * robot_relative_x;
signed_lateral_error = std::copysign(
  planarDistance(robot_position, projection), cross);
```

Use the existing finite curvature helper on adjacent nondegenerate path points and
fall back to `0.0` when three suitable points are unavailable. Validate that every
returned scalar and point coordinate is finite.

- [ ] **Step 8: Rebuild and run the focused component test**

Run:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller \
  --cmake-args -DBUILD_TESTING=ON

source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ROS_LOG_DIR=/tmp/pid_tracking_controller_test_logs \
ctest --test-dir build/pid_tracking_controller \
  -R '^test_path_progress_tracker$' --output-on-failure
```

Expected: the new component test passes with no non-finite output.

- [ ] **Step 9: Review checkpoint**

Inspect only Task 1 changes. Confirm the test would fail if the admissible interval
were replaced by a whole-path nearest search, or if accepted progress did not consume
the motion budget. Run `git diff --check` without creating a commit.

---

### Task 2: Controller Integration and Unified Lateral Diagnostics

**Files:**
- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Modify: `src/pid_tracking_controller.cpp`
- Modify: `test/controller_test_access.hpp`
- Modify: `test/test_path_and_control.cpp`

**Interfaces:**
- Consumes: `PathProgressTracker::setPath()` in `setPlan()` and
  `PathProgressTracker::update()` in `computeVelocityCommands()`.
- Produces: the same `TwistStamped` controller contract, now based on an ordered
  `PathProgressReference`; diagnostics consume its `signed_lateral_error`.

- [ ] **Step 1: Write a failing controller-level shortcut regression**

Add a `CommandContractTest` using the same dense hairpin geometry. Advance the robot
along the outbound branch, then place it slightly toward the nearby return branch.
Assert the controller still commands forward motion toward the next outbound carrot:

```cpp
TEST_F(CommandContractTest, nearbyFutureBranchDoesNotTakeOverTracking)
{
  controller_.setPlan(makeDenseHairpinPath());
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.30;
  for (double x = 0.0; x <= 0.40; x += 0.05) {
    controller_.computeVelocityCommands(makeRobotPoseAt(x, 0.0, 0.0), measured, nullptr);
  }

  const auto command = controller_.computeVelocityCommands(
    makeRobotPoseAt(0.40, 0.08, 0.0), measured, nullptr);

  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
}
```

Also assert through existing diagnostic test access that the latest absolute lateral
error is approximately `0.08`, not the approximately `0.02` distance to the future
return branch.

- [ ] **Step 2: Build and run the controller test to verify behavioral RED**

Run:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller \
  --cmake-args -DBUILD_TESTING=ON

source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ROS_LOG_DIR=/tmp/pid_tracking_controller_test_logs \
ctest --test-dir build/pid_tracking_controller \
  -R '^test_path_and_control$' --output-on-failure
```

Expected before integration: the legacy `prune()` selects or advances toward the
nearby return branch, causing the forward-command or intended lateral-error assertion
to fail.

- [ ] **Step 3: Add the tracker ownership boundary**

Forward-declare `PathProgressTracker` in the public controller header and store:

```cpp
std::unique_ptr<PathProgressTracker> path_progress_tracker_;
```

Move the controller constructor and destructor definitions out of line so the private
incomplete type is destroyed where its full definition is available:

```cpp
PidTrackingController::PidTrackingController()
: path_progress_tracker_(std::make_unique<PathProgressTracker>())
{
}

PidTrackingController::~PidTrackingController() = default;
```

Keep this implementation-private header out of the installed public include tree.

- [ ] **Step 4: Replace destructive pruning and Euclidean carrot selection**

In `setPlan()`, after existing path validation and assignment, call:

```cpp
path_progress_tracker_->setPath(orig_global_plan);
```

In `computeVelocityCommands()`, retain dynamic lookahead calculation, then replace
the `prune()` and `getLookAheadPoint()` production calls with:

```cpp
const PathProgressReference path_reference =
  path_progress_tracker_->update(pose.pose.position, lookahead_distance);

geometry_msgs::msg::PointStamped lookahead_point;
lookahead_point.header.frame_id = path_reference.frame_id;
lookahead_point.point = path_reference.carrot;
const double theta_trj = path_reference.tangent;
const double kappa = path_reference.curvature;
```

Translate tracker `std::logic_error`/`std::domain_error` failures into the existing
`nav2_core::NoValidControl` contract. Continue using `global_plan_.back()` for the
goal and do not erase `global_plan_` during control.

Remove the old controller-owned `prune()` and `getLookAheadPoint()` declarations and
definitions after all production and test call sites have moved to the tracker.

- [ ] **Step 5: Feed the accepted projection error into diagnostics**

Change `updateAndLogTrackingDiagnostics()` to consume:

```cpp
double signed_lateral_error
```

instead of a pruned path vector. Delete the loop that selects only the first valid
segment and use the supplied finite value for instantaneous and aggregate metrics.
Pass `path_reference.signed_lateral_error` from `computeVelocityCommands()`.

- [ ] **Step 6: Preserve one-point and zero-command behavior**

Replace direct legacy carrot helper tests with controller-level assertions that a
one-point path at the robot position returns:

```cpp
EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
```

Keep the existing `SpeedLimitReason::kZeroCarrotDistance` bypass unchanged.

- [ ] **Step 7: Rebuild and run controller behavior tests**

Run:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller \
  --cmake-args -DBUILD_TESTING=ON

source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ROS_LOG_DIR=/tmp/pid_tracking_controller_test_logs \
ctest --test-dir build/pid_tracking_controller \
  -R '^(test_path_progress_tracker|test_legacy_carrot_velocity_planner|test_legacy_behavior|test_path_and_control)$' \
  --output-on-failure
```

Expected: all four tests pass; the Phase 1.3 planner and regularization tests remain
unchanged and green.

- [ ] **Step 8: Review checkpoint**

Confirm `global_plan_.erase` is absent from the production control path, the tracker
is the sole source of carrot and lateral error, and no speed-planning equation or YAML
parameter changed. Run `git diff --check` without creating a commit.

---

### Task 3: Publish the Exact Carrot as PointStamped

**Files:**
- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Modify: `src/pid_tracking_controller.cpp`
- Modify: `test/test_lifecycle_smoke.cpp`

**Interfaces:**
- Topic: `/pid_tracking_controller/carrot`
- Old type: `geometry_msgs/msg/PoseStamped`
- New type: `geometry_msgs/msg/PointStamped`
- Payload: the exact `PathProgressReference::carrot` used during the same command cycle.

- [ ] **Step 1: Change the lifecycle test subscriber first**

Change the captured message and subscription to:

```cpp
geometry_msgs::msg::PointStamped received_carrot;
auto carrot_subscription =
  observer->create_subscription<geometry_msgs::msg::PointStamped>(
    "/pid_tracking_controller/carrot", rclcpp::QoS(10).reliable(),
    [&](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      received_carrot = *message;
      carrot_received = true;
    });
```

Assert observable payload semantics:

```cpp
ASSERT_TRUE(carrot_received);
EXPECT_EQ(received_carrot.header.frame_id, "map");
EXPECT_TRUE(std::isfinite(received_carrot.point.x));
EXPECT_TRUE(std::isfinite(received_carrot.point.y));
EXPECT_DOUBLE_EQ(received_carrot.point.z, 0.0);
```

- [ ] **Step 2: Build and run the lifecycle test to verify message-contract RED**

Run:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller \
  --cmake-args -DBUILD_TESTING=ON

source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ROS_LOG_DIR=/tmp/pid_tracking_controller_test_logs \
ctest --test-dir build/pid_tracking_controller \
  -R '^test_lifecycle_smoke$' --output-on-failure
```

Expected before the production type change: the PointStamped subscription receives no
message from the PoseStamped publisher and the assertion fails.

- [ ] **Step 3: Change the lifecycle publisher type and payload**

Change the controller member to:

```cpp
rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
  carrot_pub_;
```

Create it with `geometry_msgs::msg::PointStamped` and publish the same local
`lookahead_point` used by control after assigning the command timestamp:

```cpp
lookahead_point.header.stamp = cmd_vel.header.stamp;
if (carrot_pub_ && carrot_pub_->is_activated()) {
  carrot_pub_->publish(lookahead_point);
}
```

Do not construct or publish a separate pose or recompute the carrot.

- [ ] **Step 4: Rebuild and verify lifecycle GREEN**

Run the package-only incremental build and `test_lifecycle_smoke` again. Expected:
the publisher is discovered with the PointStamped type and the payload assertions pass.

- [ ] **Step 5: Review checkpoint**

Search the package for `create_publisher<geometry_msgs::msg::PoseStamped>` and
`create_subscription<geometry_msgs::msg::PoseStamped>` associated with the carrot
topic. Confirm none remain. Run `git diff --check` without creating a commit.

---

### Task 4: Final Regression and Scope Verification

**Files:**
- Verify all files changed by Tasks 1-3.
- Do not modify README, LICENSE, CONTRIBUTING, navigation launch files, or external YAML.

**Interfaces:**
- Verifies the installed shared library, plugin registry, path-progress component,
  controller behavior, and PointStamped lifecycle publisher as one package.

- [ ] **Step 1: Check that RBS navigation is not running**

Use a read-only process check. Run `webots_ros2_demo/kill_ros.sh --rbs` only if the
navigation process is actually active.

- [ ] **Step 2: Perform the final incremental package build**

Run:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller \
  --cmake-args -DBUILD_TESTING=ON
```

Expected: `pid_tracking_controller` finishes successfully.

- [ ] **Step 3: Run the relevant regression set**

Run:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ROS_LOG_DIR=/tmp/pid_tracking_controller_test_logs \
ctest --test-dir build/pid_tracking_controller \
  -R '^(test_plugin_loading|test_math_helper|test_path_progress_tracker|test_legacy_carrot_velocity_planner|test_legacy_behavior|test_path_and_control|test_lifecycle_smoke|cppcheck|xmllint)$' \
  --output-on-failure
```

Expected: every selected test passes. Do not treat unrelated existing whole-package
formatting failures as failures of the Phase 1.4 behavior, but report them accurately
if a full `colcon test` is also run.

- [ ] **Step 4: Inspect installed interfaces**

Source `install/setup.bash` and confirm the built plugin loads. The lifecycle test is
the automated evidence that the installed controller publishes PointStamped; runtime
RViz observation remains the user's next step.

- [ ] **Step 5: Review final scope**

Run `git diff --check` and `git diff --stat`. Confirm:

- only `pid_tracking_controller` changed;
- no Phase 1.3 speed or regularization math changed;
- no dynamic append, obstacle checking, reverse tracking, or final-approach behavior
  was introduced;
- the new component owns path progress rather than adding that responsibility to the
  controller class;
- no generated test, profiling, or formatting artifacts remain.

- [ ] **Step 6: Report the result without committing**

Report files modified, the ordered-progress and PointStamped responsibilities, build
and test commands/results, the fact that no simulation or navigation log was used,
and remaining runtime assumptions. Do not create a Git commit.
