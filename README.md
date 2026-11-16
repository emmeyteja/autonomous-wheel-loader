# Autonomous Wheel Loader

- Isaac Lab simulation for wheel-loader research.
- Modular components with optional Autoware interface adapters.
- Available: configuration, bounded loading commands, pile generation, and a pile-motion demo.
- Loader motion and bucket loading are not implemented yet.
- [Setup](docs/SETUP.md) · [Architecture](docs/ARCHITECTURE.md) · [Issues](https://github.com/emmeyteja/autonomous-wheel-loader/issues)

Run the pile demo:

```bash
./scripts/demo_pile.sh
```

- Saves a video to `outputs/pile_demo/pile_settling.mp4`.
- Desktop viewer: `./scripts/demo_pile.sh --interactive --no-video`.
- Tests: `./scripts/test_core.sh`.
