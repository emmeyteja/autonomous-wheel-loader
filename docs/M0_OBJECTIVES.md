# M0 — Simulation foundation objectives

Status: M0 acceptance criteria. Track verified progress and remaining gates in the [M0 milestone](https://github.com/emmeyteja/autonomous-wheel-loader/milestone/1).

## Objective

Create a reproducible, headless Isaac Lab wheel-loader task that can reset and step with bounded commands and finite observations. Establish a credible path for material acquisition and release so later loading and dumping work share the same simulator foundation.

M0 delivers an environment and evidence of stability. It does not claim learned loading, autonomous travel, dump-pose planning, or a completed work cycle.

## Deliverables and acceptance gates

1. **Reproducible runtime.** Inspect the available GPU, driver, OS, Python, Isaac Sim/Lab, and supported RL integration. Select and record a compatible version set and exact setup/launch commands. Report missing prerequisites as blockers rather than replacing the simulator with an unlabelled mock.
2. **Simulator-independent contracts.** Define only the types and configuration needed for state, loading actions, physical actuator limits, step results, and reset. Document SI units, frames, batch shapes, devices, and timestamps. Reserve separate contracts for full-cycle steering and attachment control; do not expand the loading policy action vector implicitly.
3. **Loader articulation.** Integrate a redistributable loader asset, or an explicitly documented articulated approximation with base/wheels, lift, bucket, and future steering support. Record provenance, geometry, mass/inertia assumptions, joint limits, and actuator limits. Demonstrate bounded longitudinal/lift/curl commands at minimum; mark unimplemented steering explicitly.
4. **Pile and material feasibility.** Create a procedural pile and select a documented interaction backend that can eventually model acquisition, retention, release, and deposition. A heightfield alone is not proof of transferable material. Run a small acquisition/release probe and report actual material accounting, solver limitations, throughput, and whether the backend is suitable. If credible interaction is unavailable, mark this gate blocked; a fixed collision pile may support infrastructure tests but cannot pass the material gate.
5. **Registered task and configuration.** Use Isaac Lab's standard task registration without changing its core. Expose seed, environment count, episode duration, simulation/control frequencies, scenario geometry, and action limits. Separate simulator code from pure task logic and optional middleware.
6. **Reset and smoke runner.** Provide headless zero/random-action runs with explicit seeds, finite checks, reset handling, and reproducible output metadata. Keep these runners separate from PPO training.
7. **Cycle extensibility.** Document source-pile, bucket, and future dump-bed/zone state ownership. Validate a bed geometry/zone configuration example at the schema level if introduced; do not build dump planning in M0. Document the extension points for supervisor, motion, attack selection, and dump skills.
8. **Evidence and reporting.** Save an M0 report with commands, versions, asset/material assumptions, pass/fail outcomes, limitations, and next steps. Save a short visual demonstration locally; do not commit bulky assets or videos.

## Required verification

- Pure configuration/contract and action-scaling tests run without Isaac Sim or ROS.
- Headless task registration, instantiation, reset, and step succeed.
- Zero-action and seeded random-action runs complete a total of at least 100 episodes with no NaN/Inf in states, observations, commands, or rewards. Record episode termination reasons; this stability check is not loading success.
- Commands at normalized -1, 0, and +1, and out-of-range inputs, remain within configured physical limits under the documented validation/saturation policy.
- Same seed/configuration reproduces initial conditions within stated tolerances; GPU physics nondeterminism is documented.
- Measure throughput and GPU memory at 1, 16, and 64 environments where feasible. Failed sizes and resource limits must be reported; do not claim scale from a single environment.
- Verify per-environment reset isolation when batched simulation is supported.
- Record the material probe's initial mass, acquired/retained/released material, destination/spillage where represented, and conservation error. M0 requires a credible probe, not calibrated soil physics or a productive loading policy.

All mandatory gates must pass before calling M0 complete. Hardware-limited batch sizes may be documented, but unresolved simulator or material feasibility prevents a complete M0 claim. Revisit scope with evidence if the available runtime or material backend cannot meet these objectives.

## Deferred work

PPO training, reward tuning, scripted productivity comparisons, world models, perception, navigation, autonomous zone selection, dumping policies, and full-cycle orchestration belong to later milestones. Autoware message reuse is designed at the boundary; a ROS dependency is not required to run M0.

## Implementation sequence

1. Audit the runtime and available loader/material assets.
2. Run the smallest loader articulation and material-transfer probes.
3. Implement and test contracts, task registration, scene/reset, and the headless smoke runner using the verified backend.

Choose the material approach early: loading and future zone-based dumping both depend on it.
