# Channel entry 2-B / 2-C repair plan

Goal: remove the demonstrated false stop/restart lock while preserving occupied-cell, unknown-input and continuous-motion protection.
Architecture: keep geometry in execution_safety.cpp and release policy in execution_safety_gate.cpp. No new adapter, planner or controller responsibility.
Spec: user-authorized repair based on 17:07 and latest revisions 4/5/6; recorded evidence under log/channel_entry_20261112_1721.

- [x] Pin regressions: narrow parallel passage with real clearance; thin obstacle and rotating corner remain blocked; rejected fast motion must not permanently prohibit a newly checked low-speed/current-direction motion.
- [x] 2-B: on conservative interval-envelope contact, halve only that interval (bounded to 1/16 of min(step, map resolution)); retain complete-interval coverage and all work/time limits. Affine translation at zero angular velocity integrates exactly; do not accumulate a fictitious acceleration integration error in that case.
- [x] 2-C: discard stored rejected target. Recompute release probe from current PID candidate, including ramp to that target, at least 0.2 seconds hold and full braking. Require three consecutive verified moving probes; invalid inputs reset confirmation. Continue rolling checks as PID updates its direction. Do not pretend this is full closed-loop PID prediction.
- [x] Extend existing event/throttled log with current-probe policy, candidate/resume hit pose, cell, time and distance. No per-cycle extra logger or configuration.
- [x] Incrementally build affected package in existing workspace, run safety/gate/model/limiter tests; replay recorded maps against production checker. Preserve pre-existing offline-tool/CMake work, no commits or branches.

Alternatives: reducing footprint/lethal threshold would alter collision semantics and is unsupported by evidence. Full future PID rollout introduces controller-model duplication beyond this bounded correction. Current-direction rolling revalidation addresses the stale target without those costs.
Review focus: stale inputs, zero candidate, path/epoch reset, contact between samples, remaining budget on subdivision. Existing tests plus targeted regressions cover these boundaries.

Verification: 82 safety/gate/model/limiter tests and plugin loading passed; 14 offline parser/model tests passed. Recorded revisions 4/5/6 passed 10 restart trials each under the 10 ms gate budget. These are stopped-state reconstructions, not completed navigation reruns.
