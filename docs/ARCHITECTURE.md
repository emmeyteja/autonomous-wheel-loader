# Modular architecture and Autoware interoperability

Status: accepted development requirements; the modules and adapters below are planned, not implemented.

## Goals

Reuse Autoware interfaces where their semantics fit. Keep the software modular so vehicle models, simulators, loading controllers, learning algorithms, and future planners can evolve independently. Scale training through batched simulation and scale integrations through explicit boundaries.

This supplements [DESIGN.md](DESIGN.md). The first milestone remains simulation infrastructure; it does not require running a full Autoware stack or implementing a ROS bridge.

## Proposed full-cycle extension

[PROJECT_GOALS.md](PROJECT_GOALS.md) proposes the end-to-end mission and flexible dumping scope, pending confirmation. Keep mission supervision, attack selection, travel, loading, dump-zone/pose planning, dumping, and material accounting as separate responsibilities. Represent dump-bed geometry, per-zone occupancy/capacity, and source/bucket/destination material state explicitly. A loading episode reset must not accidentally reset mission-wide bed occupancy. Keep the three-action loading skill distinct from steering-capable travel commands.

## Dependency boundaries

Start with a modular Python package. Separate ROS packages only when a concrete ROS integration is implemented. These are logical boundaries, not a requirement to create empty packages now.

- **Domain contracts:** typed loader state, bounded commands, pile state, task results, and configuration schemas. No ROS or Isaac imports; use lightweight schemas and explicit tensor contracts where batching is needed.
- **Task logic:** observation construction, rewards, termination, randomization, and scenario definitions. Operates on domain state and batched tensors. No ROS messages or PPO implementation details.
- **Simulation integration:** Isaac Lab task registration, assets, actuator application, physics stepping, contact extraction, and pile interaction. Converts simulator data into domain state.
- **Controllers:** random, scripted, and learned loading policies behind a common observation-to-action contract. Own controller state explicitly, including reset masks for batched episodes.
- **Training:** RSL-RL/PPO integration, rollout orchestration, checkpointing, and experiment metadata. Task physics stays outside this layer.
- **Evaluation:** reproducible scenarios and controller comparisons, metrics, and artifact generation. Shares the controller and task contracts without requiring the training runner.
- **Optional ROS/Autoware adapters:** translate public messages to and from domain contracts. Own ROS subscriptions, publications, clocks, frame conversions, QoS, and compatibility checks.
- **Application entry points:** compose modules and select implementations from configuration. Dependencies point inward toward contracts; domain logic never imports an adapter.

Introduce explicit protocols for simulator reset/step, controller reset/act, and experiment logging when their first implementations are built. Keep contracts narrow and prefer composition over inheritance. Do not build a generic plugin framework before interchangeable implementations are needed.

## Autoware interface reuse

Interpret “Autoware interfaces” as the public Autoware/ROS 2 message contracts. Import the selected upstream packages rather than copying message definitions into this repository. Inspect their definitions and licenses and record exact dependency versions before implementing adapters.

Candidate reuse points, subject to semantic review:

- `autoware_control_msgs/msg/Control` for compatible lateral and longitudinal motion commands.
- `autoware_vehicle_msgs` for gear commands and relevant vehicle status reports.
- Standard ROS 2 geometry, odometry, transform, and joint-state messages for matching pose, velocity, frame, and linkage reporting needs.
- `autoware_planning_msgs` for future compatible trajectories after the loading skill is established.

These are candidates, not a claim that an adapter exists or that every Autoware control assumption fits an articulated loader. `Control` contains lateral and longitudinal components; it does not define the lift/curl command contract. Do not repurpose unrelated fields to carry attachment commands or payload values.

Keep loader-specific lift/curl commands, payload estimates, pile geometry, and excavation results in explicit domain contracts. Create a minimal separate ROS interface package only for concepts not adequately represented by existing public messages. Document why each custom message is necessary.

Ackermann tire steering and articulated chassis steering have different meanings. An adapter must document and validate any conversion against the chosen loader model, including limits and reverse behavior. Reject unsupported conversions instead of silently equating angles. Gear/sign conventions, frames, and timestamps require the same explicit treatment.

## Interface contracts

Before implementing a boundary, document:

- Inputs and outputs, shapes, batch dimensions, dtype, device, and validity rules.
- SI units, coordinate frames, sign conventions, joint naming, and angle conventions.
- Timestamp source, simulation versus wall clock, sample frequency, and stale-data handling.
- Command bounds, saturation rules, ownership, and reset/error behavior.
- Which observations are available to the policy and which are privileged critic inputs.

Keep the normalized three-action policy interface separate from physical actuator commands and ROS motion commands. Physical bounds come from one validated configuration. Conversion belongs in a dedicated module and must not be duplicated across controllers.

## Scaling requirements

The training path stays in process and uses batched tensors on the simulator device where possible. Do not serialize every parallel environment through ROS messages, introduce per-environment Python loops in hot paths, or copy tensors to CPU for routine logging. Measure throughput and device memory before introducing complexity.

Keep simulator state, controller state, seeds, resets, and output paths isolated per environment or experiment. Use configurable environment counts and bounded telemetry. ROS visualization may sample a selected evaluation environment rather than mirror all training environments.

When the ROS adapter is built, support namespaces and configurable frame/topic names, bounded queues, suitable QoS, simulation time, and explicit message rates. Record the supported ROS distribution, Autoware message revision, Isaac Lab/Sim versions, and conversion assumptions. Keep incompatible dependencies in separate processes only when needed and measure bridge latency.

Future world models and attack-pose planners consume recorded state/action/pile contracts through separate modules. They must not require changes to reward internals or the simulator's actuator implementation.

## Verification as implementation progresses

- Pure task logic and domain contract tests run without ROS or Isaac Sim installed.
- Simulator integration tests verify headless launch, finite observations/rewards, limits, and deterministic reset behavior within documented physics limitations.
- Adapter tests verify units, frames, timestamps, bounds, invalid/stale inputs, and information loss during conversion; test round trips where the mapping is reversible.
- Controller contract tests exercise random, scripted, and learned implementations through the same evaluation entry point.
- Batch tests verify reset isolation and profile throughput/memory at increasing environment counts.
- Import/dependency checks prevent ROS and simulator dependencies leaking into pure modules.

No runtime tests for these planned components have passed yet. Add the checks with each implementation rather than claiming architectural requirements as working features.

## Upstream references

Reviewed on 2026-11-13; implementation must pin a compatible revision rather than rely on a moving branch.

- [Public Autoware messages](https://github.com/autowarefoundation/autoware_msgs)
- [Control message definition](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg)
- [Autoware vehicle interface guide](https://docs.autoware.org/main/tutorials/integrating-autoware/creating-vehicle-interface-package/creating-vehicle-interface/)
