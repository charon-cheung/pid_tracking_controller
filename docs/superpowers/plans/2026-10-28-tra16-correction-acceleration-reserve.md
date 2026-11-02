# TRA-16 correction acceleration reserve implementation plan

> Execute inline with superpowers:executing-plans. User explicitly authorized modifying the algorithm now and using existing 0.8 m/s real-robot cases as baseline. No additional approval, commits, branches or worktrees.

**Goal:** Reduce turn feedforward acceleration demand before the turn so position correction has headroom under the unchanged final limiter.
**Architecture:** Existing holonomic speed planner and acceleration envelope own this responsibility. Add a configurable planning-only reserve; retain full physical limits for current-command reachability, command limiting, braking and rotation.
**Tech Stack:** C++17, ROS 2 Jazzy, existing gtest and incremental colcon build.
**Spec:** Conversation-approved small repair: earlier turn slowing and correction reserve, no delay predictor, no I/D change, no offline simulator.

## Design and constraints
- Add `translation_accel_reserve_ratio`, finite in [0, 1), default and YAML initial value 0.20. Zero restores the previous preview budget. This is an initial engineering setting, not a fitted real-robot optimum.
- Preview axis budgets become `(1 - ratio) * max_ax/max_ay`. Apply to the tangent-rate cap, sparse direction jumps, curvature caps and backward joint acceleration propagation.
- Current-cycle feasibility and final limiter keep the full max_ax/max_ay. A reserve is not a guarantee against all future unknown feedback changes or chassis lag.
- Preserve nominal_tracking_speed=0.60 in the user's YAML; 0.8 comparison is selected for real-robot testing separately. No other tuning changes.
- Long straight cruise/acceleration, stopping budget, in-place rotation, external caps and yaw handling must preserve their contracts.
- No new module, node, dependency, control-loop logging or public controller interface. No README changes.

## Review focus
- Reserve must not accidentally reduce the acceleration available to correction or stopping.
- Both body axes, both turn signs and rotated map/body frames behave consistently.
- Zero reserve retains old behavior; invalid values fail configuration.
- A new lower preview target must still obey command continuity; do not bypass final limits to meet it.
- Controller tests establish semantics only, not quantified real-robot improvement.

## Task 1: Implement and verify reserve behavior
- [x] Add behavioral regression: direction jump consumes the full unreserved axis delta and clips a normal correction; reserved preview permits the same correction with unchanged physical bounds.
- [x] Check earlier braking, straight/stopping/rotation behavior and invalid reserve values. Use existing controller and limiter tests for protection.
- [x] Run one representative Red build/test in the existing workspace.
- [x] Implement preview-only reserve and ROS/YAML configuration.
- [x] Incremental package build; run relevant planner, controller, limiter and lifecycle tests, addressing environment failures separately.
- [x] Review complete diff and scope; fresh final review alongside local verification.

## Execution ledger
- Started from clean HEAD ff69b97. Existing relevant source sizes: planner 640, envelope 206, controller 746 lines.
- No RBS/navigation process detected before build/test; no kill command used.
- User instructions override skill commit/worktree and extra approval steps. No history changes.
- Build 1: targeted incremental build passed. All three new regressions failed for intended missing behavior (no reserve applied / no parameter validation). In the direction-change fixture, requested normal correction 0.01 m/s became 0.000173786 m/s.
- Implemented preview-only reserve in existing envelope/window caps; added parameter validation and ROS/YAML configuration. Physical limiter/reachability code unchanged.
- Build 2: full affected-package incremental build passed. All 12 active gtest targets passed (198 cases); final limiter, rotation, progress, curve controller and lifecycle checks included. Stale XML for removed legacy/offline tests is not counted.
- New corner regression now preserves the full 0.01 m/s correction with 20% reserve, while zero-reserve control retains its demonstrated clipping. Early preview braking and unchanged straight/stopping/reachability checks passed.
- CYCLONEDDS_URI unset for local tests; used localhost discovery/domain 231 and /tmp ROS logs. No live navigation process stopped, no real-robot command sent.
- Scope: 6 tracked files, +135/-8 lines (113 test additions), plus this plan. Production sources stay below 1500 lines; no oversized file grew. No YAML speed change or hardware deployment.
- Fresh independent final review: no blocking correctness or scope issues. Confirmed preview-only application and unchanged physical/stopping/rotation budgets. Important limitation: the corner test assumes equal feedforward speeds entering/leaving the corner and establishes local budget behavior, not a guaranteed 20% headroom in every real control cycle or a real-robot improvement percentage.
- Complete: source/config/tests ready for the user's real-robot comparison; no commit, cross-compilation or deployment performed. Initial reserve 0.20 remains subject to measured accuracy/time tradeoffs.
