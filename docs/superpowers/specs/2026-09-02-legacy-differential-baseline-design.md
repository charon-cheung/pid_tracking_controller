# Legacy Differential Tracking Baseline on Holonomic Chassis

## Status

Approved in chat for implementation planning on 2026-09-02.

## Goal

Port the existing `pid_tracking_controller` to ROS 2 Jazzy/Nav2 as a loadable
controller plugin while preserving its legacy differential tracking behavior.
The four-wheel holonomic chassis is deliberately commanded as
`vx = legacy_v`, `vy = 0`, and `wz = legacy_w` so Phase 1 provides a credible
baseline for later curve testing.

This package replaces `rbs_mppi_controller` only in the controller role. It is
a path follower in Phase 1; it does not replace MPPI's local obstacle-avoidance
behavior.

## Repository Boundary

Only the `pid_tracking_controller` repository is in scope.

The user will modify these integration files separately:

- `robot_navigation/robot_navigation_bringup/param/jazzy/nav2_common.yaml`
- `robot_navigation/robot_navigation_bringup/launch/webot_nav.py`

The implementation must not modify either file or any other repository. The
package supplies its own controller parameter file at:

```text
param/jazzy/pid_tracking_controller.yaml
```

It is installed at:

```text
share/pid_tracking_controller/param/jazzy/pid_tracking_controller.yaml
```

No extra distribution-specific directory is used.

## Phase 1 Invariant

For normal finite inputs for which the original implementation produces a
finite command, the port must preserve the legacy intermediate values and
final control output within floating-point tolerance.

Given identical path, pose, velocity, parameters, and fixed control-period
assumption, the port must produce equivalent:

- pruned path;
- Lookahead distance;
- carrot;
- `theta_trj`;
- `kappa`;
- raw `UseModelPidControl()` or `NoModelPidControl()` output;
- linear and angular regularization output;
- final `vx` and `wz`.

The final `vy` must always be exactly zero.

The only permitted changes to mathematical output are numerical robustness
fixes for states where the original code would access invalid memory, divide by
zero, or produce `NaN`/`Inf`.

## Architecture and Data Flow

The controller remains one Nav2 `nav2_core::Controller` plugin named:

```text
pid_tracking_controller::PidTrackingController
```

The configured controller ID is `PidTrackingController`.

The runtime flow remains:

```text
rbs_lattice_planner creates the yellow smoothed lattice path
  -> Nav2 Controller Server calls setPlan()
  -> global_plan_
  -> prune()
  -> getLookAheadDistance()
  -> getLookAheadPoint() / legacy circle-segment carrot
  -> UseModelPidControl() by default, or NoModelPidControl() for comparison
  -> linearRegularization() / angularRegularization()
  -> TwistStamped(vx = v, vy = 0, wz = w)
```

The plugin does not subscribe directly to `/smoothed_lattice_plan`. The path
reaches it through the standard Nav2 `setPlan()` contract.

The implementation keeps the existing controller header/source as the owner of
the legacy control flow. It does not extract or redesign the tracking kernel in
Phase 1. The root build compiles only the existing `common` geometry sources
actually needed by the controller; it does not retain dependencies on private
targets such as `ros2::ros2`, `quick_log`, or `rcmsg`.

## Jazzy Engineering Adaptation

The port makes the following non-algorithmic changes:

- use the Jazzy `rclcpp_lifecycle::LifecycleNode::WeakPtr` configure signature;
- use `ament_cmake` and `pluginlib` package metadata;
- replace private logging with standard `RCLCPP_*` logging;
- replace the private carrot publisher with a standard lifecycle publisher;
- use the `velocity` supplied to `computeVelocityCommands()` instead of an
  additional odometry subscriber;
- populate a finite stamped command and set `linear.y` to exactly zero;
- install the plugin XML, headers, shared library, and standalone YAML;
- remove the control-loop `printf()` from the circle intersection helper.

Using the Controller Server velocity preserves the legacy meaning of measured
body speed: linear speed is still `hypot(vx, vy)` and angular speed is still
`wz`. It removes only the redundant velocity source.

The controller assumes that the Controller Server supplies the current pose in
the same global frame as the path. Phase 1 does not add path transforms.

The existing internal goal-rotation branch remains disabled. Goal completion
is owned by the configured Nav2 Goal Checker. `setSpeedLimit()` remains a no-op
so that normal legacy output is not changed by a new Phase 1 feature.

## Preserved Legacy Mathematics

### Path pruning and Lookahead

The existing `global_plan_.erase(...)` pruning behavior and nearest-point
search remain unchanged.

The Lookahead distance remains:

```text
L = clamp(abs(measured_linear_speed) * lookahead_time,
          min_lookahead_dist,
          max_lookahead_dist)
```

The carrot remains the existing circle/segment intersection selected by
`getLookAheadPoint()`. The current end-of-path fallback, tangent calculation,
and three-point `kappa` calculation remain unchanged for valid inputs.

### Model controller

`UseModelPidControl()` remains the default. With world-frame carrot error
`e = target - current`, it retains the existing computation:

```text
sx_dot = k * [e_x, e_y]

R_inv = [ cos(theta),           sin(theta)
         -sin(theta) / l,  cos(theta) / l ]

[v_raw, w_raw] = R_inv * sx_dot
```

The reference velocity and target yaw remain unused in this mode, as in the
legacy implementation.

### No-model controller

`NoModelPidControl()` remains available for comparison and retains its fixed
`d_t = 1 / pid_frequency`, desired-speed construction, PID state, saturation,
and historical angle behavior. Phase 1 does not reinterpret it as a spatial
PID and does not add angle-control improvements.

### Regularization

`linearRegularization()` and `angularRegularization()` retain their historical
semantics:

- start from measured velocity;
- clamp a per-cycle velocity increment using `max_v_inc` or `max_w_inc`;
- apply maximum and minimum command bounds;
- do not multiply the increment bound by `dt`;
- do not replace measured velocity with the previous command.

The parameter names and units remain legacy-compatible. In particular,
`max_v_inc` and `max_w_inc` are not renamed or converted into acceleration
limits.

## Parameter File

`param/jazzy/pid_tracking_controller.yaml` contains a standalone Controller Server
parameter section with controller ID `PidTrackingController` and plugin type
`pid_tracking_controller::PidTrackingController`.

The baseline uses the repository's current defaults:

| Parameter | Value |
| --- | ---: |
| `goal_dist_tolerance` | `0.05` |
| `rotate_tolerance` | `0.5` |
| `base_frame` | `base_link` |
| `map_frame` | `map` |
| `lookahead_time` | `0.5` |
| `min_lookahead_dist` | `0.3` |
| `max_lookahead_dist` | `0.9` |
| `max_v` | `0.7` |
| `min_v` | `0.05` |
| `max_v_inc` | `0.5` |
| `max_w` | `0.2` |
| `min_w` | `0.0` |
| `max_w_inc` | `0.2` |
| `k_v_p` | `2.0` |
| `k_v_i` | `0.01` |
| `k_v_d` | `0.10` |
| `k_w_p` | `1.0` |
| `k_w_i` | `0.01` |
| `k_w_d` | `0.10` |
| `k_theta` | `0.5` |
| `k` | `1.0` |
| `l` | `0.2` |
| `pid_frequency` | `15.0` |
| `use_model` | `true` |

The standalone YAML also freezes the Controller Server behavior required by
the Phase 1 failure contract:

```yaml
controller_frequency: 15.0
failure_tolerance: 0.0
publish_zero_velocity: true
```

The user-owned launch and common navigation configuration are responsible for
loading this file and registering `PidTrackingController`.

## Numerical Robustness and Failure Semantics

### Invalid paths

`setPlan()` first clears the stored plan when an empty path is supplied, then
throws `nav2_core::InvalidPath`. This prevents direct callers and tests from
continuing to use a stale path even though the Jazzy Controller Server normally
rejects an empty path before calling the plugin.

`setPlan()` clears the stored plan and rejects non-finite required path data
with `nav2_core::InvalidPath`, so an invalid replacement can never leave a
previous path active. Required data includes every pose's `x`, `y`, and
quaternion components needed to obtain yaw. A valid one-point path remains
supported and uses the existing end-point carrot behavior.

Calling `computeVelocityCommands()` without a stored path or with an empty
pruned path throws `nav2_core::InvalidPath`.

### Invalid control states

Non-finite current pose or velocity, a non-finite carrot, a non-finite control
intermediate, or a non-finite final command throws
`nav2_core::NoValidControl`. The controller does not invent a fallback carrot
or silently substitute a finite command.

With `failure_tolerance: 0.0`, `NoValidControl` immediately fails the current
FollowPath action. With `publish_zero_velocity: true`, the Controller Server
publishes a zero command from its exception handling path.

### Circle/segment intersection

Let the intersection discriminant be:

```text
discriminant = r * r * segment_length_squared - determinant * determinant
```

The helper applies this order:

1. if the segment length squared is at or below the math epsilon, return no
   intersection;
2. if the discriminant is below the negative numerical tolerance, return no
   intersection;
3. if the discriminant is in the small interval below zero, clamp it to zero;
4. only then evaluate `sqrt(discriminant)` and divide by segment length
   squared;
5. return only finite intersection values.

No valid finite carrot after the existing selection process produces
`nav2_core::NoValidControl`.

## Simulation Diagnostics

Diagnostics observe the legacy controller but never feed values back into it.

### Outputs

The plugin publishes:

- `pid_tracking_controller/carrot` (`geometry_msgs/msg/PoseStamped`): the exact
  carrot used by the current control cycle.

Current and aggregate measurements are written as one `RCLCPP_INFO` line with
the `PID_LOG` prefix at most once per second. The first control cycle after
activation or a statistics reset logs immediately. There is no tracking
diagnostics topic.

### Measurements

Each `PID_LOG` line contains:

- signed current lateral error;
- cumulative lateral-error RMS;
- cumulative P95 absolute lateral error;
- cumulative maximum absolute lateral error;
- actual traveled distance;
- actual average planar speed;
- current commanded and measured linear speed;
- current commanded and measured angular speed;
- ratio of sampled final linear commands at the configured magnitude limit;
- ratio of sampled final angular commands at the configured magnitude limit;
- current legacy `kappa`, `theta_trj`, Lookahead distance, and carrot;
- sample count, plan revision, and elapsed tracking time.

The lateral-error projection is diagnostic-only. It projects the current robot
position onto the nearest valid local segment at the start of the pruned path.
It is never used to select the carrot or compute a command. A one-point path
uses distance to that point for the diagnostic value.

Each `setPlan()` increments a plan revision. Aggregate run statistics reset
only when the path's terminal goal changes, matching the legacy controller's
state-reset boundary. Replanning to the same goal therefore does not erase the
run-level metrics.

The controller does not hard-code definitions of steady-state curvature bands
or recovery after a curvature reversal. Recording `kappa`, lateral error, and
traveled distance makes those quantities available for path-specific log
analysis without embedding experimental thresholds in the controller.

## Phase 1 Non-goals

Phase 1 explicitly does not:

- change `prune() + erase()`;
- introduce a persistent progress index;
- replace Lookahead with arc-length lookup;
- change the carrot generation definition;
- add curvature speed limiting;
- add Feedforward;
- use `vy`;
- add a new yaw controller;
- change the `UseModelPidControl()` mathematical formula;
- change the historical regularization semantics;
- support dynamic Path append;
- add controller-internal collision checking;
- implement a new terminal state machine;
- add local obstacle avoidance;
- add projection feedback, a holonomic controller, LQR, or MPC;
- modify planner, launch, Controller Server, or navigation configuration files
  outside this repository.

## Testing Strategy

Implementation follows test-driven development for retained behavior and
numerical fixes.

### Geometry tests

`circleSegmentIntersection()` tests cover:

- tangent intersection;
- two intersections;
- no intersection;
- a tiny negative discriminant clamped to zero;
- a clearly negative discriminant rejected;
- a zero-length segment rejected;
- finite output for every returned intersection.

### Path and failure tests

Tests cover:

- empty Path clears stored state and throws `InvalidPath`;
- non-finite Path data throws `InvalidPath`;
- a one-point Path retains the legacy end-point carrot behavior;
- no finite valid carrot throws `NoValidControl`;
- non-finite pose, velocity, intermediate, or output throws
  `NoValidControl`.

### Legacy equivalence tests

Representative straight, rotated, and curved cases compare the port against
the legacy equations for:

- `getLookAheadDistance()` and both clamps;
- pruned-path selection;
- carrot, `theta_trj`, and `kappa`;
- raw model-controller output;
- raw no-model-controller output and PID state update;
- linear and angular regularization;
- final `vx`, `vy`, and `wz`.

### Jazzy integration tests

Tests verify that:

- pluginlib discovers and instantiates the class from the installed plugin
  description;
- configure, activate, deactivate, and cleanup use Jazzy lifecycle APIs;
- the Controller Server velocity argument reaches the legacy measured-speed
  path;
- output headers and numeric fields are finite;
- `vy` remains exactly zero.

Diagnostics are simulation measurement instrumentation. A targeted smoke check
verifies that the carrot can be subscribed to, a `PID_LOG` line is emitted, and
no tracking diagnostics topic exists while the controller is active.

## Verification

Before a ROS build or test, check whether the RBS navigation program is running.
Only if it is running, stop it with the existing `kill_ros.sh --rbs` command.

Use the ROS 2 Jazzy environment and existing workspace artifacts:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/CNF2026987648/s100_docker
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
colcon test --packages-select pid_tracking_controller
colcon test-result --verbose
```

Verification remains package-scoped. No clean build, temporary build tree, or
full-workspace build is part of normal Phase 1 verification.

Review the complete package diff, `git diff --stat`, and `git diff --check`.
Confirm that no other repository changed.

## Simulation Evaluation Order

After integration by the user, evaluate in this order:

1. the original boustrophedon path;
2. a large-radius circle;
3. progressively smaller-radius circles;
4. an S-curve;
5. a smooth spline with continuously varying curvature;
6. the original boustrophedon path again after any parameter experiments.

Use the `PID_LOG` records and existing command/odometry topics to evaluate:

- lateral RMS, P95, and maximum error;
- traveled distance and actual average speed;
- commanded versus actual linear and angular velocity;
- angular-velocity saturation ratio;
- steady-state error at different curvatures;
- recovery distance after curvature sign changes.

The outcome of these tests decides whether the next phase keeps the legacy
algorithm, adds `vy` assistance, or replaces the control law with a holonomic
projection-feedback plus Feedforward design.

## Acceptance Criteria

Phase 1 is complete when:

- the package builds incrementally on ROS 2 Jazzy;
- pluginlib loads `pid_tracking_controller::PidTrackingController`;
- the standalone YAML is installed directly under
  `share/pid_tracking_controller/param/jazzy`;
- normal finite legacy cases pass the intermediate and final-output
  equivalence tests;
- `linear.y` is always zero;
- empty/non-finite paths and invalid control states produce the specified Nav2
  exception types;
- circle/segment geometry does not take an invalid square root or divide by a
  zero-length segment;
- the carrot and `PID_LOG` tracking diagnostics are observable in an active lifecycle;
- no Phase 1 Non-goal is implemented;
- only files in the `pid_tracking_controller` repository are changed.
