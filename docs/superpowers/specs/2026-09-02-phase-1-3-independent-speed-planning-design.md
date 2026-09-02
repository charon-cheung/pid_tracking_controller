# Phase 1.3 Independent Speed Planning Design

## Goal

Decouple translational speed from carrot distance while preserving the Phase 1.2
Legacy carrot steering geometry. Straight segments and gentle curves should run at
the configured cruise speed, while sharper turns should reduce the common command
scale before the angular command reaches the executable angular-speed limit.

## Scope

Phase 1.3 changes only the `use_model=true` velocity-generation path in
`pid_tracking_controller`.

The following behavior remains unchanged:

- Legacy `prune + erase`, dynamic lookahead, and circle-segment carrot selection.
- Start-heading alignment and its one-time-per-goal semantics.
- `NoModelPidControl()` when `use_model=false`.
- `linearRegularization()` and `angularRegularization()` order and historical
  semantics.
- Forward-only tracking and `linear.y = 0`.
- FinalApproach controller handoff.
- The existing path `kappa` remains diagnostic-only.
- No collision checking, future-path curvature scan, dynamic path append, reverse
  tracking, or new terminal-state machine is introduced.

## Architecture

A small stateless `legacy_carrot_velocity_planner` component owns normalized-carrot
geometry and angular-feasibility speed planning. It consumes body-frame carrot
error and validated scalar parameters and returns both the raw velocity command
and all values needed for diagnostics. `PidTrackingController` remains responsible
for path/carrot orchestration, start alignment, regularization, command validation,
and logging.

The component is implementation-private to this package. No new ROS interface,
publisher, plugin, or runtime algorithm-selection switch is added. The Phase 1.2
baseline remains available from Git history.

## Control Model

For body-frame carrot error `(ex, ey)`:

```text
carrot_distance = hypot(ex, ey)
cos_alpha = ex / carrot_distance
sin_alpha = ey / carrot_distance
alpha = atan2(ey, ex)
steering_gain_ratio = model_k_w / model_k_v
angular_planning_limit = max_w - angular_speed_reserve
required_angular_at_cruise =
  cruise_speed * steering_gain_ratio / l * sin_alpha
```

When `abs(sin_alpha)` is large enough to constrain speed:

```text
angular_limited_speed =
  angular_planning_limit * l /
  (steering_gain_ratio * abs(sin_alpha))
reference_speed = min(cruise_speed, angular_limited_speed)
```

For a near-zero `sin_alpha`, the angular cap is inactive and the finite effective
`angular_limited_speed` reported in diagnostics is `cruise_speed`.

The raw command is:

```text
linear_x_raw = reference_speed * cos_alpha
angular_z_raw =
  reference_speed * steering_gain_ratio / l * sin_alpha
linear_y_raw = 0
```

This preserves the Phase 1.2 spatial steering ratio whenever `cos_alpha` is
nonzero:

```text
angular_z_raw / linear_x_raw =
  steering_gain_ratio * tan(alpha) / l
```

The existing forward-only clamp remains after regularization. The planner does not
silently replace it with a different reverse-motion policy.

## Numerical and Configuration Contract

- If `carrot_distance` is at or below the existing math epsilon, return finite zero
  raw commands and `speed_limit_reason=zero_carrot_distance`.
- All planner outputs must be finite.
- `model_k_v`, `model_k_w`, `l`, `cruise_speed`, and `max_w` must be finite and
  strictly positive.
- `angular_speed_reserve` must be finite and satisfy
  `0 <= angular_speed_reserve < max_w`.
- `cruise_speed` must not exceed `max_v`; invalid configuration fails during
  controller configuration rather than entering the control loop.
- Non-finite body-frame carrot error fails the current control cycle through the
  existing `NoValidControl` contract.

## Parameters

Add two production parameters:

```yaml
cruise_speed: 0.51
angular_speed_reserve: 0.10
```

The first Phase 1.3 baseline uses:

```yaml
model_k_v: 1.7
model_k_w: 1.9
cruise_speed: 0.51
angular_speed_reserve: 0.10
```

`model_k_v` and `model_k_w` remain separate for comparison with Phase 1.2, although
only their ratio affects the Phase 1.3 model command. Existing `max_v=0.8`,
`max_w=0.6`, `l=0.2`, and start-heading parameters are preserved.

## Diagnostics

Extend the existing `PID_LOG` record without adding a diagnostics topic:

```text
carrot_alpha
carrot_alpha_deg
carrot_sin_alpha
carrot_cos_alpha
steering_gain_ratio
cruise_speed
angular_planning_limit
angular_limited_speed
reference_speed
speed_limit_reason
required_angular_at_cruise
```

The reason is one of `cruise`, `angular`, `zero_carrot_distance`, or
`not_applicable`. `not_applicable` is used during start alignment and for the
`use_model=false` path. Existing raw/final command and saturation fields remain.

## Verification

Unit tests cover straight motion, distance-scale independence, angular limiting,
preservation of the steering ratio, zero-distance behavior, and invalid
configuration. Controller-level tests verify `linear.y == 0`, existing
regularization still follows the planner, start alignment is unchanged, and all
new diagnostics appear in `PID_LOG`.

No navigation log or simulation result is required for the code implementation
step. Runtime acceptance remains a separate user-operated activity.
