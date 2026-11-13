# Autonomous Wheel Loader

A simulation-only research project for learning wheel-loader bucket filling with reinforcement learning.

## Status

Initial project scaffold and design specification. The Isaac Lab environment, loader asset, pile interaction, PPO training, and evaluation are **not implemented yet**. No training or simulation results are available.

## First objective

Build a reproducible NVIDIA Isaac Lab environment for one bucket-loading attempt from a predefined approach pose. Use state observations and three bounded continuous commands: longitudinal motion, lift-arm velocity, and bucket-curl velocity. Compare PPO with random and scripted controllers.

The intended training stack is Isaac Lab with RSL-RL PPO. Compatible versions and machine requirements will be recorded when integration is implemented.

## Architecture requirements

Modularity and scalability are project requirements. Reuse public Autoware and standard ROS 2 interfaces when their semantics fit, through optional adapters. Keep the simulation and batched training independent of ROS; use explicit loader-specific contracts for attachment control and pile interaction.

See [the architecture requirements](docs/ARCHITECTURE.md) for module boundaries, interface reuse, and scaling expectations. These requirements guide future implementation; no ROS adapter is implemented yet.

## Development roadmap

- [ ] M0: Isaac Lab integration, task registration, loader asset, procedural pile, and smoke tests.
- [ ] M1: Observations, actions, reward components, PPO training, checkpointing, and evaluation.
- [ ] M2: Improved pile interaction, actuator limits, force/slip penalties, and scripted baseline.
- [ ] M3: Domain randomization and evaluation on unseen conditions.
- [ ] M4: Learned world model.
- [ ] M5: Attack-pose planning.

See [the design specification](docs/DESIGN.md) for requirements and acceptance criteria. This document is the proposed research plan, not evidence of implemented functionality.

## Repository layout

- `configs/`: planned environment, training, and evaluation configuration.
- `src/wheel_loader_rl/`: future environment, asset integration, evaluation, and utilities.
- `scripts/`: future training, evaluation, smoke-test, and benchmark entry points.
- `tests/`: future unit and simulator integration tests.
- `docs/`: design specification and experiment results.
- `outputs/`: local experiment artifacts, excluded from version control.

## Local package setup

```bash
python -m pip install -e .
```

This installs the namespace scaffold only. It does not install Isaac Lab or provide a runnable simulation or training command.

## Research scope

This project is for simulation research. It does not provide a real-machine control interface or demonstrate sim-to-real validity. Do not commit proprietary machine assets, credentials, datasets, training checkpoints, logs, or videos.

## License

A software license has not been selected yet.
