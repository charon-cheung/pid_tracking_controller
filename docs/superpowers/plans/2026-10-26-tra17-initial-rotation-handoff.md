# TRA-17 Initial Rotation Handoff Implementation Plan

> Execute inline with superpowers:executing-plans; no commits or worktrees per project instructions.

**Goal:** Preserve initial rotations for handoff within 0.02 m of both the rotation anchor and initial path progress.
**Architecture:** Keep nearest-branch selection and ambiguity rejection unchanged. Add a private initial-rotation guard in PathProgressTracker; reuse existing rotation admission/completion and acknowledgement.
**Tech Stack:** C++17, ROS 2 Jazzy, ament/gtest.
**Spec:** User-approved conversation design: 2 cm position and arc constraints, initial rotations only, preserve TRA-8 and normal intermediate rotations.

## Constraints and review focus
- No new node, ROS parameter, public API, gain change, or global geometry tolerance change.
- Preserve all pre-existing TRA-16 edits; no git history changes.
- Initial consecutive rotations stay pending; completed ones use existing acknowledgement.
- Nearby return branches with large arc and ambiguous branches must not reset to the initial rotation.
- Near-anchor but outside distance threshold, and short curved paths with arc exceeding threshold, must not trigger protection.
- Build incrementally in workspace build/install with MAKEFLAGS=-j12; stop RBS first only if running.

## Task 1: Guard initial rotation at handoff
Files: src/path_progress_tracker.cpp, test/test_path_progress_tracker.cpp, test/test_path_and_control.cpp.
Interfaces: initializePathHandoff(Point, double) unchanged.
- [x] Add measured P2/P6/P9 and P7 fixtures; assert pending rotation before acknowledgement and translation after.
- [x] Add distance/arc boundary, small perturbation, return-branch and consecutive rotation cases; retain existing TRA-8 tests.
- [x] Add controller regression: zero translation while initial yaw remains wrong despite positive millimetre projection; translation resumes after existing completion conditions.
- [x] Build affected package once; run new tests against old production logic and verify meaningful Red.
- [x] Add implementation-private 0.02 m guard after branch acceptance for leading rotation only; retain strict existing checks for other rotations.
- [x] Incrementally build again; run relevant tracker/reference/rotation/controller suites.
- [x] Review task-only diff and verify no unrelated file edits; independent final review.

## Execution ledger
- Pre-flight: one cohesive task, no new interfaces. RBS process check found no matching navigation processes.
- Approved design already authorizes execution. User constraints override skill defaults for commits/worktrees; use existing workspace and retained task evidence in log/tra17_20261026.

- Red evidence: old production code failed recorded P2/P6/P9 and near-start controller regression for the intended skipped-rotation behavior; ambiguity rejection remained passing. Logs: log/tra17_20261026/test_*_red.log.
- Green evidence: 2 total incremental package build invocations, both successful. Six targeted suites / 108 test cases passed (tracker, prepared path, reference, rotation completion, rotation entry, controller); tests_green.log and build_green.log retained.
- Scope check: production path_progress_tracker.cpp 647 -> 655 lines; tracker tests +103 lines, controller tests +23 lines. Pre-existing TRA-16 edits outside these files preserved; task.diff records exact task-only changes. No oversized production file grew; no parameters or public interfaces changed.
- Final independent review: no substantive correctness or regression findings. git diff --check passed; native targeted verification is complete. Deployment and real-robot confirmation remain pending.
