# Straight tracking guard implementation plan

> Execution: apply the approved design in this workspace using executing-plans; no commits, branches or worktrees.

**Goal:** Extend outward-drift protection to straight translation and release after 0.4 s of small error.

**Architecture:** Keep detection in TranslationTrackingGuard and speed limiting in HolonomicSpeedPlanner. Remove curvature from the guard interface. Separate 0.15 s entry and 0.4 s release confirmation.

**Tech Stack:** C++17, ROS 2 Jazzy, ament/gtest, existing colcon build/install.

**Spec:** User-approved conversation design: enter at >=0.03 m with sustained growing error and outward speed >=0.02 m/s; release at <=0.015 m continuously for 0.4 s. Freeze further reference acceleration only. Keep max_vy=0.5.

## Constraints and review focus

- Preserve curve triggering, left/right and body-frame symmetry.
- Reject inward motion, non-growing errors and isolated outward samples.
- Reset release confirmation outside the release band.
- Keep entry timing independent of release timing; validate new duration.
- Preserve correction, stopping constraints, mode/path resets and acceleration on release.
- Include the parameter in existing logs/snapshots for Track Insight.

## Task 1: Guard and integration

- [x] Adapt guard tests for straight triggering and 0.4 s release; run one pre-change failing cycle.
- [x] In src/translation_tracking_guard.hpp/.cpp remove local_tangent_rate, add and validate release_confirm_time=0.4, choose duration by holding state.
- [x] Adjust the call in src/holonomic_speed_planner.cpp without changing the speed limiting action.
- [x] Load/log tracking_guard_release_confirm_time in src/pid_tracking_controller.cpp; set 0.40 in param/jazzy/pid_tracking_controller.yaml.
- [x] Extend test/test_holonomic_speed_planner.cpp for straight/curve, frame/sign symmetry and delayed release; retain reset/stopping checks.
- [x] Incrementally build only pid_tracking_controller with MAKEFLAGS=-j12; run focused guard/planner and relevant integration tests.
- [x] Review full diff, whitespace, growth and scope; no commits or deployment.

## Execution record

- Initial worktree clean; no navigation process running at preflight.
- User authorization covers execution; no further design approval needed.

- Verification: two incremental build invocations, both successful. Representative pre-change guard test failed for straight non-triggering and early release. Post-change focused suites: 143 tests passed (guard 4, planner 43, law 9, limiter 29, path/control 54, lifecycle 4).
- Lifecycle initially failed from inherited DDS interface 192.168.170.6, not code behavior. Existing binary passed with a per-process loopback CycloneDDS configuration; no rebuild or global environment changes.
- One comment indentation adjustment after compilation was whitespace-only; no extra build required.
- Final independent static review: no findings. No active deceleration, correction law or final limiter changes. All requested behavior implemented; physical tracking improvement remains to be validated on the robot.
