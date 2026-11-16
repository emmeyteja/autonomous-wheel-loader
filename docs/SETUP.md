# M0 development setup

Target runtime: Linux x86_64, Python 3.12, and a compatible CUDA-capable NVIDIA GPU/driver. Track current task integration in the [M0 milestone](https://github.com/emmeyteja/autonomous-wheel-loader/milestone/1).

## Runtime choice

Use Isaac Lab 3.0.0 with its default Newton-based stack for initial headless probes. The GPU runtime has been exercised in a bounded upstream probe. Isaac Sim/Kit is not installed; this is an Isaac Lab setup, not a verified Isaac Sim setup. This choice follows the [versioned installation guide](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html), which supports a Newton installation without Isaac Sim. Loader/material suitability still needs experimental verification.

No separate system CUDA toolkit is required for these wheel-based probes. No ROS stack is required. Do not change the host's ROS or Python installation to satisfy project dependencies.

## Recreate the observed environment

From the repository root, using Python 3.12 and a compatible NVIDIA driver:

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install uv==0.13.8
.venv/bin/uv pip sync --python .venv/bin/python --index-strategy unsafe-best-match requirements/m0-linux-py312-cu130.txt
.venv/bin/uv pip install --python .venv/bin/python --no-deps -e .
```

The requirements file is an exact package-version snapshot of the observed Linux environment, not a portable multi-platform lockfile. Fresh recreation from this snapshot has not yet been tested. The project excludes the environment and large outputs from Git.

For a smaller simulator-free development install, install PyTorch using the appropriate official CPU/CUDA index, then install `.[dev]`. Core configuration and pile modules can be imported without PyTorch; action execution requires it. The minimum simulator-independent Python requirement for this project is currently 3.12.

## Checks

```bash
./scripts/test_core.sh
.venv/bin/python scripts/audit_runtime.py
./scripts/runtime_smoke.sh --seed 42 --steps 100 --num-envs 1
```

The unit-test wrapper disables automatic third-party pytest plugin discovery so the host's unrelated ROS plugins cannot break simulator-independent tests. It does not install or modify ROS dependencies.

The runtime smoke command exercises the upstream cartpole task, not a wheel-loader task. It writes `outputs/m0/runtime_probe.json`, checks finite observations/rewards, and compares seeded reset observations. **Currently the reset comparison fails on the verified runtime; the command intentionally returns nonzero.** See [issue #3](https://github.com/emmeyteja/autonomous-wheel-loader/issues/3). Do not bypass this failed result when evaluating M0 readiness.

Runtime inventory and diagnostic outputs stay local under `outputs/m0/`; do not upload them wholesale to this public repository or its issues. These commands do not train a policy or command hardware.

## Configuration and material assumption

`configs/env.toml` defines timing, seed, environment count, physical action bounds, and a procedural coarse-sphere pile layout. Parameters are initial uncalibrated simulation assumptions. `generate_pile` produces geometry only; it does not simulate contacts, acquisition, release, or calibrated soil behavior.

The three loading action columns are longitudinal velocity [m/s], lift rate [rad/s], and curl rate [rad/s]. State and command frames use +X forward, +Y left, +Z up. Steering remains a separate future motion contract.

## Pile demo

- Run `./scripts/demo_pile.sh` for a headless rigid-sphere simulation and local video.
- Run `./scripts/demo_pile.sh --interactive --no-video` for the Newton desktop viewer; requires a display.
- Default: 966 coarse spheres in a trapezoidal stockpile, five simulated seconds, 25 video frames per second.
- Presets: `configs/pile_trapezoid.toml` (382 spheres), `configs/pile_trapezoid_dense.toml` (966), and `configs/env.toml` (original small mound).
- Synthetic benchmark geometry: 6 m along X × 8 m along Y at the base, 1.5 m high, initially 30° side slopes, with a flat top. These are project assumptions, not dimensions copied from a paper.
- Use `--duration-s 15` for a longer settling preview. Geometry is an initial condition; no walls or particle locking maintain the slopes.
- Resolutions share the intended geometry; represented particle mass/packing can differ. They are not a validated physical convergence study.
- Large site-scale piles require an active-region or multiscale approach; particle/candidate limits reject oversized full-pile requests.
- Video: `outputs/pile_demo/pile_settling.mp4`; raw positions and summary stay in the same ignored directory.
- Video export requires `ffmpeg` on PATH; `--no-video` skips export.
- This shows gravity and contacts, not calibrated soil, bucket loading, or loader motion.

- The rigid-sphere demo starts at ground level; use `--drop-height-m 0.8` only for an intentional drop experiment.
- `pile.contact_dimensions = 6` enables rolling resistance in MuJoCo contacts; mode 3 omits it even when a rolling-friction coefficient is set. The exported effective model is checked at startup.
- These coarse particles are a contact demo, not a calibrated dry-sand model. The first target material is dry sand; its repose angle, bulk density, and excavation response still require validation.

## Stockpile geometry references

- [Aoshima et al., §6.2](https://www.mdpi.com/2673-4052/5/3/16): triangular, conical, and wedge seed piles at 20° and 30°; the 3.6 m and 5.2 m heightmaps are local observation windows, not full-pile dimensions.
- [Aoshima–Servin, §5.3](https://link.springer.com/article/10.1007/s11044-024-10005-5): the DEM validation uses piles in a 6 m-wide container.
- [JICA thermal-power coal-terminal study, printed p.147, Fig.5.3.25](https://openjicareport.jica.go.jp/pdf/12229118_01.pdf#page=171): an elongated trapezoidal stockpile, 16 m high, 6 m top width, and 47 m base width. This is a coal-site example, not a dry-sand calibration target.
