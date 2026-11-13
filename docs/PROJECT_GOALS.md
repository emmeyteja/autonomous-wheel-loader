# Repository goal and scope

Status: development specification; implementation progress is tracked in GitHub.

## End goal

Build a modular, simulation-only autonomous wheel-loader system that repeatedly performs:

**Choose attack pose → approach → load bucket → extract → travel → select dump zone and dump pose → align → dump → retreat → update pile and bed state → choose the next attack and repeat.**

A preliminary dump destination may be reserved before travel; refine and validate the exact dump pose before alignment. Decisions must use the updated source pile and destination state, so the system can change its next attack, route, or dump zone after every cycle.

A mission ends when the configured transfer target is met, the dump destination is full, the source is depleted, or no feasible cycle remains. Report the reason and actual delivered material.

The initial destination is a stationary dump bed with known geometry and pose. Moving trucks, perception-driven bed detection, and multiple destinations are later extensions, not assumed requirements for the first full-cycle demonstration.

## Zone-based dumping

Make dump zones configurable and geometry-driven rather than a fixed set of three dumping points.

- Describe bed length, width, floor height, wall geometry, usable fill height, pose, payload limit, and existing material occupancy with explicit units and frames.
- Support user-defined zones and generated longitudinal segments, with optional subdivision across bed width. Zone count and boundaries must adapt to usable geometry and loading objectives.
- Maintain per-zone current volume, target volume, remaining usable volume, and available dump poses. Track mass separately and use explicit material density when converting between mass and volume.
- Select a reachable zone with sufficient capacity for the predicted deposited material and permitted fill profile. Account for the bucket footprint, material spread, and uncertainty; do not assume all released material stays inside the selected zone.
- Check machine/bucket reach, articulation and linkage limits, wall clearance, approach/retreat feasibility, global bed payload, and configured material distribution constraints.
- Permit configurable sequencing strategies, such as user-defined order or balancing fill deficits. Do not hard-code one front-to-back order for every bed.
- Update actual deposited volume, bed profile, remaining bucket material, and spillage after each dump before choosing the next zone or attack. A commanded dump does not count as successful delivery.
- If no zone can accept the next bucket, choose a smaller load target when feasible or end the mission with an explicit reason. Never silently exceed capacity.

The first baseline uses known simulator geometry and measured simulated material state. Estimated state and uncertain perception can be added through the same contracts later.

## Modular responsibilities

- A **mission supervisor** owns cycle transitions, completion conditions, and bounded recovery decisions.
- An **attack selector** chooses the source pose and target bucket load.
- A **motion planner/controller** moves between approach, extraction, travel, and dump poses.
- A **loading skill** controls penetration, curl, and lift.
- A **dump planner** selects a zone, desired deposit, feasible dump pose, and sequence.
- A **dumping skill** aligns, empties, and withdraws the bucket.
- A **material-state module** tracks source pile, bucket contents, bed occupancy, and spillage.

Modules exchange typed domain contracts. Keep their logic separate from simulator and optional Autoware adapters. The three-action loading policy remains local to bucket loading; full-cycle motion requires a separate steering-capable contract. A learned world model may improve decisions later but is not required to demonstrate a scripted full cycle.

## Delivery sequence

- **M0 — Foundation:** verified simulator setup, loader articulation, material-interaction feasibility, task registration, smoke tests, and extensible scene/contracts. See [M0 objectives](M0_OBJECTIVES.md).
- **M1 — Basic loading RL:** state observations, bounded actions, decomposed rewards, PPO/checkpoints, and independent evaluation against random actions.
- **M2 — Useful loading:** credible material interaction, scripted loading baseline, and measured productivity/force/slip comparisons.
- **M3 — Loading generalization:** domain randomization and held-out scenarios.
- **M4/M5 — Research extensions:** world modeling and learned attack-pose evaluation as described in the original design. These do not block baseline cycle integration.
- **M6 — Travel and zone dumping:** articulated motion, configurable bed/zones, dump-pose feasibility, scripted dumping, material accounting, and scenario evaluation.
- **M7 — Repeated autonomous cycle:** supervisor integrates attack, loading, travel, dumping, state updates, replanning, recovery, and mission completion. Establish a scripted baseline before replacing individual skills with learned policies.

After M2, travel/dumping baseline development can proceed independently of M3–M5 through stable contracts. Do not require one end-to-end RL policy to learn the entire mission.

## Full-cycle acceptance criteria

Freeze numeric thresholds and scenario configurations before evaluating. Thresholds are not selected yet.

- Complete repeated cycles without manually selecting every attack or dump pose.
- Demonstrate configurable zone generation and selection across different bed lengths, usable volumes, and initial occupancy profiles.
- Measure delivered volume/mass, transfer rate, cycle time, work estimate, fill distribution, spillage, maximum interaction force, failures, and recovery outcomes.
- Reconcile material across source pile, bucket, bed, and spilled material within documented simulation tolerances; do not infer delivery from reward alone.
- Compare a scripted mission baseline and the proposed controller on the same scenarios and seeds, including full-bed, unreachable-zone, and interrupted-cycle cases.
- Save configurations, software revisions, results, and videos for reproducibility. Report model simplifications and limitations explicitly.

## Relationship to the original design

[DESIGN.md](DESIGN.md) remains the detailed bucket-loading research specification. Its instruction to defer the full work cycle applies to the early loading milestones, not the repository's eventual goal. This proposal extends the original roadmap with travel, flexible zone dumping, and repeated mission execution. This document governs the overall scope, and the architecture requirements apply to every milestone.
