# Phase 1.3 Independent Speed Planning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add independent cruise-speed generation and angular-feasibility speed limiting while preserving Legacy carrot steering geometry and `vy=0`.

**Architecture:** A package-private stateless planner computes normalized-carrot geometry, the angular speed cap, raw `vx/wz`, and diagnostic values. The existing controller orchestrates this planner before its unchanged regularization functions and logs the returned planning data.

**Tech Stack:** C++17, Eigen, ROS 2 Jazzy, Nav2 controller plugin, ament_cmake_gtest.

**Spec:** `docs/superpowers/specs/2026-09-02-phase-1-3-independent-speed-planning-design.md`

## Global Constraints

- Modify only `pid_tracking_controller`.
- Keep path processing, start alignment, FinalApproach handoff, `NoModelPidControl()`, and regularization semantics unchanged.
- Keep `linear.y` exactly zero and do not add reverse tracking.
- Do not use path `kappa` for control and do not add future curvature preview.
- Do not read navigation test logs or perform user-owned simulation testing.
- Do not create Git commits, branches, or worktrees.
- Use incremental package builds in the existing workspace.

---

### Task 1: Stateless Legacy Carrot Velocity Planner

**Files:**
- Create: `src/legacy_carrot_velocity_planner.hpp`
- Create: `src/legacy_carrot_velocity_planner.cpp`
- Create: `test/test_legacy_carrot_velocity_planner.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: body-frame `ex`, `ey` and `LegacyCarrotVelocityPlannerConfig`.
- Produces: `LegacyCarrotVelocityPlan planLegacyCarrotVelocity(...)` containing raw command and diagnostic fields.

- [x] **Step 1: Write failing planner tests**

Cover these observable invariants:

```cpp
EXPECT_NEAR(straight.raw_linear_command, 0.51, 1e-12);
EXPECT_NEAR(straight.raw_angular_command, 0.0, 1e-12);
EXPECT_EQ(straight.speed_limit_reason, SpeedLimitReason::kCruise);

EXPECT_NEAR(near_plan.raw_linear_command, far_plan.raw_linear_command, 1e-12);
EXPECT_NEAR(near_plan.raw_angular_command, far_plan.raw_angular_command, 1e-12);

EXPECT_LE(std::abs(turn.raw_angular_command), 0.5 + 1e-12);
EXPECT_EQ(turn.speed_limit_reason, SpeedLimitReason::kAngular);

EXPECT_DOUBLE_EQ(zero.raw_linear_command, 0.0);
EXPECT_DOUBLE_EQ(zero.raw_angular_command, 0.0);
EXPECT_TRUE(allPlanFieldsAreFinite(zero));
```

- [x] **Step 2: Run the planner test target and verify RED**

Run an incremental package build with `BUILD_TESTING=ON`. Expected failure: the
planner header/type does not exist yet.

- [x] **Step 3: Implement the minimal stateless planner**

Implement:

```cpp
LegacyCarrotVelocityPlan planLegacyCarrotVelocity(
  double ex_body,
  double ey_body,
  const LegacyCarrotVelocityPlannerConfig & config);
```

Use `std::hypot`, `std::atan2`, the existing math epsilon, finite effective speed
caps, and explicit reason values. Reject invalid configuration with
`std::invalid_argument` and non-finite geometry with `std::domain_error`.

- [x] **Step 4: Build and run the planner test target to verify GREEN**

Expected: all planner tests pass and the library compiles with warning flags.

### Task 2: Controller Integration and Parameters

**Files:**
- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Modify: `src/pid_tracking_controller.cpp`
- Modify: `test/controller_test_access.hpp`
- Modify: `test/test_legacy_behavior.cpp`
- Modify: `test/test_path_and_control.cpp`
- Modify: `param/jazzy/pid_tracking_controller.yaml`

**Interfaces:**
- Consumes: Task 1 planner config/result.
- Produces: the `use_model=true` raw command passed to existing regularization and the baseline Phase 1.3 YAML.

- [x] **Step 1: Write failing controller-level tests**

Replace Phase 1.2 raw-command expectations with Phase 1.3 externally meaningful
behavior:

```cpp
EXPECT_NEAR(model_output.x(), 0.51 * cos_alpha, tolerance);
EXPECT_LE(std::abs(model_output.y()), max_w - reserve + tolerance);
EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
```

Retain the existing start-alignment and forward-only tests unchanged.

- [x] **Step 2: Run affected tests and verify RED**

Expected: old distance-scaled `UseModelPidControl()` output differs from the new
independent-speed expectation.

- [x] **Step 3: Integrate the planner**

Declare/read:

```cpp
double cruise_speed_{0.51};
double angular_speed_reserve_{0.10};
```

Validate configuration once in `configure()`. In normal `use_model=true` tracking,
obtain the raw command from the planner, validate it with the existing finite
check, then call unchanged `linearRegularization()` and
`angularRegularization()`. Keep the final forward-only clamp and `linear.y=0`.

- [x] **Step 4: Update the baseline YAML**

Set exactly:

```yaml
model_k_v: 1.7
model_k_w: 1.9
cruise_speed: 0.51
angular_speed_reserve: 0.10
```

Preserve `max_v`, start-heading tolerances, and unrelated values.

- [x] **Step 5: Build and run controller tests to verify GREEN**

Run `test_legacy_behavior` and `test_path_and_control`. Expected: both pass without
changing path-front-end or alignment assertions.

### Task 3: PID_LOG Planner Diagnostics

**Files:**
- Modify: `include/pid_tracking_controller/pid_tracking_controller.hpp`
- Modify: `src/pid_tracking_controller.cpp`
- Modify: `test/test_lifecycle_smoke.cpp`

**Interfaces:**
- Consumes: Task 1 `LegacyCarrotVelocityPlan`.
- Produces: planner fields in the existing throttled `PID_LOG` record.

- [x] **Step 1: Add failing log assertions**

Require every planner field from the design and verify a normal straight tracking
sample reports `speed_limit_reason=cruise`.

- [x] **Step 2: Run lifecycle smoke test and verify RED**

Expected: the existing `PID_LOG` record lacks Phase 1.3 planner fields.

- [x] **Step 3: Pass planner result into diagnostics and extend one log statement**

Do not add publishers or per-field logs. Use `not_applicable` for alignment and
`use_model=false` cycles.

- [x] **Step 4: Run lifecycle smoke test to verify GREEN**

Expected: lifecycle smoke passes and continues to observe no tracking diagnostics
topic.

### Task 4: Final Verification

**Files:**
- Review all modified files; no new behavior is added in this task.

- [x] **Step 1: Confirm no RBS navigation process is running**

Only invoke `webots_ros2_demo/kill_ros.sh --rbs` if a matching process is active.

- [x] **Step 2: Run one final incremental package build**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
MAKEFLAGS='-j12' colcon build --packages-select pid_tracking_controller --cmake-args -DBUILD_TESTING=ON
```

- [x] **Step 3: Run relevant tests**

Run planner, plugin loading, math helper, controller behavior, lifecycle smoke,
cppcheck, and xmllint tests with `--output-on-failure`.

- [x] **Step 4: Review scope and whitespace**

Run `git diff --check`, `git diff --stat`, inspect the complete diff, verify YAML
values, and confirm no unrelated files changed.
