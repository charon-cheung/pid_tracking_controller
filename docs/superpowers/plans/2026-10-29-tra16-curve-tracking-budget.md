# TRA-16 curve tracking budget implementation plan

> Execute inline using executing-plans; user approved implementation in conversation. No commits, branches, worktrees or README changes.

**Goal:** Preserve 0.8 m/s on feasible straights, reduce excessive turn feedforward and prevent reacceleration while tracking error grows outward.

**Architecture:** Replace the uncommitted 20% preview reserve with one normal-acceleration budget in the existing curvature speed preview. Retain the physical axis envelope at full max_ax/max_ay. A small TranslationTrackingGuard owns confirmation/hysteresis; HolonomicSpeedPlanner uses its hold before reference acceleration/reachability. The controller supplies signed lateral error and measured body velocity, and logs the resulting hold.

**Tech Stack:** C++17, existing ROS 2 Jazzy package and gtest.

**Spec:** Approved conversation design; details below are implementation choices.

## Design and constraints
- max_translation_curve_accel = 0.35 m/s^2 initially; curvature budget min(configured value, max_ax, max_ay). Combine caps by minimum, never multiply curvature speed by another reserve ratio.
- Remove translation_accel_reserve_ratio from runtime config and migrate its tests to meaningful curvature/correction invariants.
- Tracking guard: signed robot-minus-path normal error; entry abs(error) >= 0.03 m, outward measured normal speed >= 0.02 m/s, error growing, local forward tangent rate >= 0.1 rad/m. Require 0.15 s of consecutive qualifying intervals. Entry uses only locally upcoming curvature, not maximum over the remaining path.
- While held, reference speed may decrease but may not increase. Keep full position feedback, axis acceleration limits and braking caps. Release after abs(error) <= 0.015 m for 0.15 s. A single sign/noise excursion cannot release. Clear on setPath, seed/control gap, rotation/terminal or absent observation.
- No fixed-delay estimator, I/D terms, smoother changes, safety changes or new dependencies. No claims of counterfactual real trajectory improvement.
- Preserve prior nominal 0.8 and diagnostic edits. New state lives outside the controller. Keep small logs by extending existing statements.

## Alternatives
- Restore HEAD and restart: discards useful parameters/diagnostics; rejected.
- Stack 20% reserve and new curve limiter: duplicates tuning; rejected.
- Selected: one curve budget plus response-informed acceleration hold using existing planner constraints.

## Review focus
- Hold must not be overridden by the reachable-speed lower bound.
- Normal startup lag, inward correction and straight motion must not cause entry.
- Both signs/yaws must project normal feedback consistently.
- New path, control gaps and rotation must clear state.
- Low external limits and stopping/braking remain authoritative while held.

## Tasks
- [x] 1. Add a representative curved-vs-straight regression; verify behavioral failure on current code in one incremental build.
- [x] 2. Replace preview reserve with curvature budget; implement guard, wire optional observation and diagnostics. Test hold, confirmation/release, reset, finite inputs, coordinate signs, correction budget and stop/external constraints.
- [x] 3. Incrementally build the package and run relevant/all package tests. Review full diff against saved pre-task state and HEAD, inspect file growth, perform one fresh review, resolve substantive findings.

## Execution ledger
- Baseline: HEAD ff69b97 with 6 modified files (+182/-9) and prior untracked reserve plan. Saved patch and sources under log/tra16_curve_guard_1029.
- No RBS/nav2 runtime processes found before verification.

- Representative regression RED: unit-radius curve target remained 0.8 instead of below 0.62. One incremental build, failure was behavioral.
- Review ruling: apply the new tracking budget only in the existing spatial-curvature preview. Keep the acceleration envelope at its original full physical axis budget, including discrete direction jumps. Applying the lower tracking budget at every vertex introduced unintended sampling-dependent slowdown; a radius-2 / 0.1 m sampling regression confirmed target 0.546 instead of >0.74. Continuous-preview and physical-envelope caps are intersected, not multiplied. The reserve guarantee remains planning headroom, not a fixed per-cycle correction guarantee.

- Build 4: sampling slowdown removed; guard/most regressions passed. Two test assertions needed geometry-aware expectations: windowed piecewise curvature permits ~0.748 (not >0.75) on moderate samples; inward projection uses the incoming chord, so controller correction must be compared in that accepted normal. No production change needed for these fixture corrections.

- Build 5: controller integration and sampling regression passed. Additional stop-under-hold assertion used a fresh tracker update at the far endpoint, which remained bounded by initial progress; it did not exercise stopping. Fixed via existing initializePathHandoff with an explicit last-segment assertion. Build 6 is required solely to verify this stopping semantic; normal 5-build budget exceeded by one, reported to user. No control change from build 4.
- Package-wide lint exposes existing formatting divergence; only new component/test formatting is corrected. cppcheck tool reports skipped execution for installed 2.13.0; do not report it as an executed static analysis pass.

- Final verification: build 6 succeeded; 13 active C++ gtest targets, 206 tests, zero failures/errors, including lifecycle and controller integration. Full-package lint is not clean: baseline replay confirms the same 11 uncrustify failing files; flake8/pep257 failures are in untouched tools/latency_spike/analyze.py. New guard and its tests pass formatting. cppcheck was skipped by the installed-version policy. git diff --check passed. Task-only patch, statistics and verification summary saved beside the baseline. No commits or vehicle deployment performed.
