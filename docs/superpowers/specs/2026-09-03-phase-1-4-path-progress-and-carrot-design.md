# Phase 1.4 Ordered Path Progress and Observable Carrot Design

## Goal

Eliminate the Phase 1.3 shortcut failure in which Euclidean nearest-point pruning
can jump to a spatially nearby future branch, permanently erase untraversed path,
and make subsequent carrots continue toward the wrong branch. Publish the carrot
as `geometry_msgs/msg/PointStamped` so the exact control reference can be observed
directly in RViz and ROS tools.

## Scope

Phase 1.4 changes only the path-progress, carrot-selection, lateral-error diagnostic,
and carrot-publication responsibilities inside `pid_tracking_controller`.

The following behavior remains unchanged:

- Phase 1.3 independent speed planning and angular-feasibility speed reduction.
- `model_k_v`, `model_k_w`, `cruise_speed`, `max_v`, `max_w`, and regularization.
- Start-heading alignment and its once-per-goal semantics.
- Forward-only tracking and `linear.y = 0`.
- `NoModelPidControl()` behavior.
- FinalApproach controller handoff.
- No collision checking, reverse tracking, dynamic path append, or terminal-state
  machine is added.

## Root Cause

The current `prune()` searches a relatively long prefix of the remaining path for
the pose with minimum Euclidean distance to the robot and then permanently erases
all preceding poses. On a U-turn, S-curve, loop, or another self-near path, a future
branch can be closer than the branch the robot is currently traversing. Once chosen,
the skipped path is irrecoverable and repeated control cycles can ratchet progress
farther forward.

The current `getLookAheadPoint()` then searches by Euclidean radius from the robot.
When the first remaining pose is already outside the lookahead circle, it creates an
artificial robot-to-path segment. The resulting point is not guaranteed to lie on an
original path segment.

The diagnostic lateral error uses only the first valid segment of the already-pruned
path. It therefore reports the error to the newly selected future branch rather than
the error to the intended path progress.

## Architecture

Introduce a focused implementation-private `PathProgressTracker` component. It owns:

- the ordered path geometry and cumulative arc lengths;
- the accepted monotonic path progress;
- continuity protection against physically impossible forward jumps;
- projection of the robot onto the currently admissible path interval;
- arc-length carrot interpolation;
- the signed lateral error at the accepted projection.

`PidTrackingController` remains the orchestrator. `setPlan()` supplies a complete
replacement path to the tracker. `computeVelocityCommands()` asks the tracker for one
reference result and passes its carrot into the unchanged Phase 1.3 velocity planner.
The same projection result supplies the diagnostic lateral error, preventing control
and diagnostics from choosing different path branches.

No factory, interface hierarchy, or runtime algorithm selector is introduced.

## Ordered Progress Contract

Progress is represented as a segment index, interpolation fraction, and cumulative
arc length. It is monotonic during one installed plan.

The admissible forward-search distance is governed by a small internal travel budget:

1. On the first control sample after `setPlan()`, the budget permits local anchoring
   within at most two representative path-point spacings from the path start.
2. Each later sample replenishes the budget by the robot's measured planar position
   displacement since the previous sample.
3. Accepted forward arc-length progress consumes the same amount of budget.
4. The stored budget is capped at two representative path-point spacings.
5. Repeated calls with an unchanged robot position cannot repeatedly move progress
   forward.

The representative spacing is the median finite, nonzero distance between consecutive
path poses. Degenerate zero-length segments are ignored. For a single-point path,
progress and carrot both remain at that point.

Within the admissible ordered interval, the tracker selects the closest finite segment
projection. It never searches an arbitrary future portion of the path solely because
that portion is spatially close.

Every complete `setPlan()` replaces the tracker path and resets path progress. This is
not dynamic path append. Existing goal-coordinate logic continues to decide whether
start-heading alignment may re-enter.

## Carrot Contract

Given accepted projected arc length `s` and dynamic lookahead distance `L`:

```text
carrot_s = min(s + L, total_path_length)
```

The carrot is interpolated on the original path segment containing `carrot_s`.
Therefore:

- it always lies on an original path segment, except that a one-point path returns
  its only point;
- it cannot lie on an artificial robot-to-future-path connector;
- it cannot advance onto a branch excluded by the ordered progress guard;
- its tangent and diagnostic curvature are derived from neighboring original path
  segments.

The Phase 1.3 velocity planner continues to consume the carrot error in the robot
frame. No velocity equation changes in this phase.

## Carrot Publication

Keep the existing topic name:

```text
/pid_tracking_controller/carrot
```

Change its type from `geometry_msgs/msg/PoseStamped` to
`geometry_msgs/msg/PointStamped`.

Each successful control cycle publishes the exact carrot used to generate that
cycle's command:

- `point.x/y/z` are the interpolated carrot coordinates;
- `header.frame_id` is the installed path frame;
- `header.stamp` is the current command timestamp.

No second compatibility topic is added because this interface is currently used for
simulation observation and the requested message type is explicit.

## Diagnostics

`lateral_error` and its RMS/P95/maximum aggregates use the signed distance from the
robot to the accepted progress projection. They no longer scan only the first segment
of a destructively pruned suffix and do not search the entire path for a spatially
near future branch.

Existing `PID_LOG` names remain unchanged. The carrot coordinates in the log and the
published `PointStamped` must describe the same reference.

## Numerical and Failure Contract

- Empty or non-finite paths retain the existing `InvalidPath` behavior.
- Zero-length segments are skipped without division by zero.
- All projection, progress, tangent, curvature, and carrot outputs must be finite.
- Failure to obtain a valid ordered projection or carrot produces the existing
  `NoValidControl` failure rather than a non-finite command.
- A one-point path remains supported.
- The zero-carrot command continues to bypass the minimum linear-speed floor.

## Verification

Behavior-level tests must cover:

- a U-shaped/self-near path where a future branch is spatially closer than the
  intended branch;
- repeated control calls at an unchanged pose do not ratchet progress forward;
- the carrot lies on an original path segment and advances by path arc length;
- a one-point path returns a finite stationary carrot and zero command;
- signed lateral error comes from the accepted progress segment;
- the lifecycle publisher is discoverable as `PointStamped` and publishes the same
  finite carrot used by the control cycle;
- existing start alignment, forward-only behavior, Phase 1.3 speed planning, plugin
  loading, and lifecycle behavior remain intact.

Runtime simulation acceptance remains user-operated. No navigation test log is needed
for the implementation verification step.
