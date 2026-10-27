# TRA-16 acceleration envelope implementation plan

> **For agentic workers:** Use superpowers:executing-plans. User authorized implementation in the current checkout; no commits, branches, worktrees or README edits.

**Goal:** Reduce outward commands after raw requests inward correction, without increasing hardware limits.
**Architecture:** A private acceleration-envelope function supports the existing speed planner. Local body-direction constraints are propagated backward; the current cycle intersects FF + position feedback with the previous final command's reachable rectangle. The final limiter retains braking and rotation semantics.
**Tech Stack:** C++17, ROS 2 Jazzy, existing gtest/colcon targets.
**Spec:** Approved TRA-16 design in the conversation: non-cancelling turn preview, joint body-axis acceleration, previous-final-command reachability, unchanged final limiter and rotation admission.

## Global constraints
- Baseline b46e22a includes TRA-15, raw topic and user YAML changes (ax/ay 1.0, alpha 0.5). Preserve them.
- No new ROS node, dependency, I/D term, or tuning parameter.
- Current-cycle acceleration interval applies to ordinary translation; stopping/rotation/reseeding keep existing behavior.
- Empty feasibility is reported and falls back to the existing limiter; zero FF is not assumed reachable.
- Reuse workspace build/install, source Jazzy, MAKEFLAGS=-j12; target <=5 incremental build invocations.
- Fixed-pose offline replay is not evidence of changed closed-loop or real chassis behavior.

## Review focus
- Opposing turns, heading changes and cross-pi angles must not disappear through averaging.
- Preview must be continuous across segment boundaries; sparse sharp corners need a per-cycle direction-change cap.
- Positive lower bounds and empty intervals must not force zero or bypass velocity/stopping constraints.
- Consume actual previous output and ignore stale state on reseed/control gaps.
- Preserve straight tracking, TRA-15, stopping and TRA-7 rotation behavior.

## Task 1: Direction preview and envelope
- [x] Add an abrupt short-turn/opposite-turn regression and demonstrate missing early braking.
- [x] Add private holonomic_acceleration_envelope.hpp/.cpp with a free function returning the preview cap. Interpolate nominal tangent between segment midpoints; retain segment boundaries and target-yaw changes. Bound actual tangent jumps using acceleration * dt.
- [x] Bound each preview interval by C*abs(a_ref)+D*v^2 per axis. Propagate limits backward using joint remaining acceleration. Forward acceleration is checked from actual previous output each cycle.
- [x] Accumulate absolute turn in legacy windows; retain all existing speed/yaw/stopping caps.

## Task 2: Current-cycle reachability
- [x] Add optional previous final body command to planner.plan and const seeded-state accessor on limiter.
- [x] Intersect v*t+feedback with command acceleration rectangle, scalar reference rise/fall and existing hard caps. Clamp only a nonempty interval; otherwise preserve fallback and flag unreachable.
- [x] Wire controller; append compact fields to existing diagnostics. Preserve reset/braking.
- [x] Test both axes/signs, moving frames, positive lower/empty intervals, caps and state handoff.

## Task 3: Verification
- [x] Incremental affected-package build and focused tests; batch fixes.
- [x] Deterministic closed-loop curve regression using production components; check steps, finite progress and cross-track error.
- [x] Remaining package behavioral tests, diff/growth checks and one independent final review.
- [x] Report local native evidence and remaining real-robot validation; no deployment/commit.

## Execution ledger
- No related RBS process running; no stop command needed. Checkout clean at b46e22a.
- Ruling: use this ledger, not commit-based skill scripts, because the user forbids commits/branches/worktrees and has approved execution.
- Ruling: preserve dedicated stopping acceleration/deceleration policy; ordinary-translation interval does not replace it.

- Build 1: existing planner regression target built; new short-turn test failed behaviorally (old cap 0.8 vs <=0.136781 m/s).
- Implemented nominal midpoint/boundary preview and ordinary-translation current-cycle intersection. Added focused reachability and closed-loop regressions before Green verification.

- Build 2: affected package incremental native build succeeded; focused planner, limiter and controller tests passed.
- Independent review: no blocking defect; requested direct inward-raw/outward-final behavior coverage. Added that comparison with swapped axes and both correction signs; no further production changes.
- Build 3: rebuilt only the two changed test targets in the existing workspace. Final focused tests: 111 cases passed (36 planner, 29 limiter, 46 controller).
- Closed-loop ideal command-following, fixed-yaw curve test: maximum path distance 0.026009 m; this is not a real-robot improvement measurement or a baseline comparison.
- Wider behavior checks: 182 active cases total, 181 passed; one unchanged lifecycle parameter-recovery test fails. It declares rotation_max_alpha with invalid max_alpha=0, then restores only max_alpha. Its fixture and corresponding configure/limiter validation logic are unchanged from b46e22a. Not modified as part of TRA-16.
- Initial lifecycle attempt hit a host Cyclone DDS configuration selecting absent 192.168.170.6. Reused the same binary with CYCLONEDDS_URI unset; no rebuild for the environment issue.
- Final diff whitespace check passed. No YAML changes; current max_ax/max_ay/max_alpha remain 1.0/1.0/0.50. No new node/parameter or commit. Production source files remain below 1500 lines.
- Remaining validation: real chassis comparison of raw versus final command normal components, peak/area of path error, ordinary travel speed and stopping/rotation behavior. Hardware lag, slip and dynamic-yaw closed-loop performance are not established by these tests.
