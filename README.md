# Autonomous Wheel Loader

- Isaac Lab simulation for wheel-loader research.
- Modular components with optional Autoware interface adapters.
- Available: configuration, bounded loading commands, pile generation, and a pile-motion demo.
- Experimental: scripted loader approximation and particle-transfer checks; dry-sand calibration remains open.
- [Setup](docs/SETUP.md) · [Architecture](docs/ARCHITECTURE.md) · [Issues](https://github.com/emmeyteja/autonomous-wheel-loader/issues)

Run the pile demo:

```bash
./scripts/demo_pile.sh
```

- Default: 966-ball trapezoidal pile; saves `outputs/pile_demo/pile_settling.mp4`.
- Faster preset: `./scripts/demo_pile.sh --config configs/pile_trapezoid.toml`.
- Desktop viewer: `./scripts/demo_pile.sh --interactive --no-video`.
- Tests: `./scripts/test_core.sh`.

- Loader probe: `./scripts/demo_pile.sh --config configs/loader_probe.toml --loader-probe --duration-s 30 --output outputs/loader_probe`.
- [Material checks and limitations](docs/MATERIAL_MODEL.md).
