# Autonomous Wheel Loader

- Simulation-only, modular development of an articulated WA320-like loader.
- Active: parameterized chassis, four wheels, boom/bucket and closed Z-bar geometry.
- [Complete unladen V-cycle scenarios](docs/experiments/V_CYCLE.md): travel, loading/dump motions and return to origin.
- [Reconstructed model and limitations](docs/calibration/WA320_RECONSTRUCTION_STATUS.md). Physical parameters are provisional; no calibrated soil or trained policy yet.
- [Scripted material-scoop experiment](docs/experiments/M2_SCRIPTED_SCOOP.md): physical coarse grains, CPU diagnostics; M2 acceptance remains incomplete.
- Komodo and earlier particle demos are preserved under `extras/` as references.
- [Setup](docs/SETUP.md) · [Kinematics](docs/models/LOADER_KINEMATICS.md) · [Architecture](docs/architecture/ACTIVE_SYSTEM.md) · [Issues](https://github.com/emmeyteja/autonomous-wheel-loader/issues)

```bash
pip install -e ".[dev]"
python train.py --vehicle wa320  # geometry checks; no learning
python scripts/build_wa320.py
./scripts/test_core.sh
```
