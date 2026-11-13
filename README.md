# Autonomous Wheel Loader

A simulation-only research project targeting repeated autonomous wheel-loader attack, loading, travel, zone-based dumping, and replanning. Bucket-filling reinforcement learning is the first skill.

## Status

M0 implementation has started: validated configuration, batched bounded actions, deterministic pile geometry, runtime setup, and unit tests are implemented. Isaac Lab 3.0 runs on the GPU with Newton. The wheel-loader task, loader asset, material interaction, PPO training, and evaluation are **not implemented yet**.

Track implementation and verified outcomes in the [M0 milestone](https://github.com/emmeyteja/autonomous-wheel-loader/milestone/1). The upstream reset issue is tracked in [issue #3](https://github.com/emmeyteja/autonomous-wheel-loader/issues/3); M0 remains incomplete.

## End goal

Choose an attack pose, load the bucket, extract and travel, choose a feasible dump zone and pose, dump, update pile/bed state, and repeat. Dump zones should adapt to bed geometry, usable volume, existing fill, and configurable sequencing.

See [the confirmed overall goal](docs/PROJECT_GOALS.md) and [M0 objectives](docs/M0_OBJECTIVES.md). The initial dump destination is a stationary bed with known geometry.

## First objective

Build a reproducible NVIDIA Isaac Lab environment for one bucket-loading attempt from a predefined approach pose. Use state observations and three bounded continuous commands: longitudinal motion, lift-arm velocity, and bucket-curl velocity. Compare PPO with random and scripted controllers.

The intended training stack is Isaac Lab with RSL-RL PPO. Compatible versions and machine requirements will be recorded when integration is implemented.

## Architecture requirements

Modularity and scalability are project requirements. Reuse public Autoware and standard ROS 2 interfaces when their semantics fit, through optional adapters. Keep the simulation and batched training independent of ROS; use explicit loader-specific contracts for attachment control and pile interaction.

See [the architecture requirements](docs/ARCHITECTURE.md) for module boundaries, interface reuse, and scaling expectations. These requirements guide future implementation; no ROS adapter is implemented yet.

## Development roadmap

- [ ] M0: Reproducible runtime, task registration, loader articulation, material-transfer feasibility, and headless smoke tests.
- [ ] M1: Observations, actions, reward components, PPO training, checkpointing, and evaluation.
- [ ] M2: Improved pile interaction, actuator limits, force/slip penalties, and scripted baseline.
- [ ] M3: Domain randomization and evaluation on unseen conditions.
- [ ] M4: Learned world model.
- [ ] M5: Attack-pose planning.
- [ ] M6: Travel, flexible dump-bed zones, dump-pose selection, and dumping baseline.
- [ ] M7: Repeated full-cycle orchestration, state updates, recovery, and replanning.

World-model research is optional for baseline cycle integration; travel/dumping can proceed after the loading foundation through separate modules.

See [the design specification](docs/DESIGN.md) for requirements and acceptance criteria. This document is the proposed research plan, not evidence of implemented functionality.

## Development tracking and public information

Use GitHub milestones and issues for tasks, acceptance checklists, defects, and progress. Keep repository documentation focused on architecture, reproducible setup, and research methods. Issues are public: post only sanitized technical summaries. Keep personal information, workstation inventories, raw logs, and local paths out of commits and issue attachments.

## Repository layout

- `configs/`: planned environment, training, and evaluation configuration.
- `src/wheel_loader_rl/`: future environment, asset integration, evaluation, and utilities.
- `scripts/`: future training, evaluation, smoke-test, and benchmark entry points.
- `tests/`: future unit and simulator integration tests.
- `docs/`: design specification and experiment results.
- `outputs/`: local experiment artifacts, excluded from version control.

## Development setup

Follow [SETUP.md](docs/SETUP.md) for the verified Python 3.12 / Isaac Lab 3.0 / CUDA environment, dependency snapshot, and test commands.

```bash
./scripts/test_core.sh
.venv/bin/python scripts/audit_runtime.py
```

No wheel-loader simulation or training command exists yet. `runtime_smoke.sh` probes the upstream cartpole runtime and currently reports a failed reset check; it does not prove a loading environment works.

## Research scope

This project is for simulation research. It does not provide a real-machine control interface or demonstrate sim-to-real validity. Do not commit proprietary machine assets, credentials, datasets, training checkpoints, logs, or videos.

## License

A software license has not been selected yet.
