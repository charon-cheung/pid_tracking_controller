# TRA-19 fixed-heading yaw hysteresis implementation plan

> Execute inline with superpowers:executing-plans. User authorizes this implementation in the existing workspace; no commits, branches or worktrees.

**Goal:** suppress unnecessary yaw corrections during fixed-heading translation, starting correction at 2 degrees and stopping at 1 degree.

**Architecture:** add an internal FixedHeadingYawGate next to the existing rotation gates. Keep the stateless control law unchanged. Filter its angular terms before the final command limiter; linear output and rate limits remain unchanged.

**Tech Stack:** C++17, ROS 2 Jazzy, existing ament/gtest tests.

**Spec:** user-approved discussion on 2026-10-27: fixed-heading translation only; 2/1 degree hysteresis; rotation, terminal and intentional changing-heading translation bypass; controlled angular deceleration remains.

## Constraints and review focus
- Identify eligibility from translation mode and near-zero geometric target yaw rate (numerical tolerance only), never commanded omega.
- Reset on new path, control reseed and leaving the eligible phase; a different fixed target starts a fresh hold state. Error above entry threshold starts correction immediately.
- Parameterize enable and entry/exit radians; defaults enabled, 0.03490658503988659 / 0.017453292519943295. Validate finite 0 < exit < enter <= pi.
- Preserve both signs and wrapped yaw, full linear correction, changing-yaw feedforward, rotation/terminal control, and final acceleration limits.
- Default navigation is not running (checked before build). Reuse workspace build/install, MAKEFLAGS=-j12; at most 3 builds expected.

## Task 1: observable regression
- [x] Add controller-output regression for 1.5-degree error, preserved xy command, 2.5-degree correction, continued correction at 1.5 degrees, exit below 1 degree.
- [x] Incrementally build affected target and observe the intended missing-suppression failure.

## Task 2: component and wiring
- [x] Add src/fixed_heading_yaw_gate.hpp/.cpp with config, apply(reference, command), suppression state and reset.
- [x] Wire constructor/configuration, new-plan/reseed reset, angular filtering and compact state-change diagnostics into controller. Add defaults to Jazzy YAML.
- [x] Add tests for symmetric thresholds, wrap, sign change, phase bypass, tiny intentional yaw rate, reset, disabled configuration, invalid thresholds and angular limiter tails.
- [x] Incrementally build pid_tracking_controller. Run component, active controller, control-law, limiter, rotation gates/completion, reference and lifecycle tests.
- [x] Review diff, source sizes and independent review; report native build vs RK3588 deployment and need for real chassis validation.

## Progress
- Initial working tree clean; current controller 722 lines, public header 127 lines.

- Red build #1 passed; regression failed on the expected 1.5-degree and below-exit nonzero omega. No navigation processes were running.

- Build #2 passed; component/control/reference/rotation/limiter tests passed. Lifecycle first failed due a stale CYCLONEDDS_URI interface; reran with that variable unset and localhost discovery.
- Existing lifecycle invalid-parameter test retained rotation_max_alpha=0 from its deliberately invalid max_alpha=0 configure call. Corrected only the fixture recovery to restore both parameters; added actual configured-threshold/disable and invalid-order output checks. Production limiter defaults are unchanged.
- Independent read-only review found no blocking issues in the implementation snapshot.

- Build #3 compiled lifecycle additions; their first run identified repeated simulated time and a test clock replaced by configure. Build #4 compiled only the corrected lifecycle target. No production changes after build #2. Four incremental builds total, within the medium-change budget of five.
- Final verification: all eight targeted CTest targets passed with localhost DDS configuration. Controller grew from 722 to 746 lines; no oversized source file grew. Native x86_64 build only, no RK3588 deployment or Git commit. Real-chassis validation remains required.
