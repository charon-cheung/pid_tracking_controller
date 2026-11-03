# Position-growth entry coverage for translation tracking guard

**Goal:** Cover the Nov 3 P5/P20 missed entry when position error grows but measured outward velocity drops. Preserve the existing P19/P22 guard response, correction and release.

**Authorized scope:** Latest user request to supplement straight protection for these cases. Implement in the existing workspace, preserving pre-existing uncommitted changes. No commits, branches, worktrees or deployment.

**Design:** While not holding, require absolute lateral error >= enter_error and same-side error growth > 0.002 m/s. Accept either measured outward velocity or the same-side finite-difference absolute error growth rate >= min_outward_speed (0.02 m/s). Both qualifying endpoints and sustained confirm_time (0.15 s) remain mandatory. A single jump, opposite-side switch, small error, constant error or recovering error does not confirm entry. Reuse existing state and parameters; document the threshold's dual meaning. Hold only feedforward acceleration and release at <= 0.015 m continuously for 0.4 s as before.

**Responsibility:** TranslationTrackingGuard owns detection. HolonomicSpeedPlanner owns the existing speed cap and correction budget; no changes to those algorithms, omega filtering or drive command interfaces.

**Review focus:** Same-side/signed growth, dt normalization, intermittent velocity disagreement, genuine inward recovery, isolated TF jumps, sample-rate effects, release/reset semantics, straight/curved body-frame symmetry. Sparse log replay cannot prove exact full-rate trigger time or closed-loop error improvement. Drift below 0.02 m/s without outward feedback remains outside this addition.

## Task 1: Tests and minimal implementation

- [x] Add observed P5/P20 sequence tests and sustained-growth/feedback-disagreement tests; preserve legacy outward-speed entry and reject noise/recovery.
- [x] Exercise straight-path speed cap and intact normal correction with inward measured feedback, retaining existing curve, reset and stop checks.
- [x] One representative pre-change failing test/build cycle.
- [x] Add the alternative growth-rate trigger and update threshold comments, with no new parameter or state.
- [x] Incrementally build pid_tracking_controller and run focused suites.

## Task 2: Evidence and final verification

- [x] Replay recorded observations for all 23 cases through old/new production guard; report sampled-data limitations explicitly.
- [x] Inspect task-only and complete diffs, sizes and whitespace.
- [x] One independent final review; address substantive findings and verify.

**Verification:** Existing build/install under /home/CNF2026987648/s100_docker; source ROS Jazzy in each build/test shell, MAKEFLAGS=-j12. Check navigation processes first. Expected pre-change failures: P5/P20 and inward-feedback straight hold. Expected post-change: all selected suites pass. No synthetic closed-loop accuracy claim from replay.

## Verification result

142 tests passed after one representative failing cycle. Two incremental package builds plus one disposable C++ replay compilation. 822 recorded observations from all 23 cases were replayed; P5/P20 candidate entry works on the recorded sequences, P19/P22 sampled transitions are preserved. Conditional/throttled logging prevents exact reconstruction of controller state; runtime/replay mismatches are recorded in log/pid_guard_growth_1103/replay_summary.json. Independent read-only review found no Critical/Important/Minor issue. No deployment or physical accuracy claim.
