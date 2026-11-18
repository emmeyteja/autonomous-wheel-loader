# Run the active foundation

- Python 3.12. Install the core and test tools with `pip install -e ".[dev]"`.
- No Komodo checkout, ROS, TensorFlow, Isaac or GPU is required for geometry validation.

```bash
python train.py --vehicle wa320
python scripts/build_wa320.py
python -m pytest tests/unit -q
```

`train.py` currently defaults to foundation geometry validation. It does not train a policy; `--mode ppo` fails with the prerequisite gate. `--vehicle komodo --allow-reference` reports archive metadata only.

## Use the committed model

`assets/wheel_loader/wa320/usd/` contains portable floating and fixed PhysX assets. In the full Isaac Sim environment, you can run the probes below with those asset paths directly, without installing the standalone converter.

## Optional supported USD conversion

Use a separate Python 3.12 environment with the supported `isaacsim-asset-isolated==6.2.0.0` importer and this project installed. The tested dependency combination includes `usd-exchange==3.0.1` and `usd-optimize==1.3.1`; keep standalone importer dependencies separate from Isaac Lab/Newton and full Isaac Sim.

```bash
python scripts/convert_wa320.py --output outputs/wa320/usd
python scripts/convert_wa320.py --fixed --output outputs/wa320/usd_fixed
```

The converter adds the mandatory passive loop closure and exports a portable single-layer PhysX asset. Bare URDF import alone leaves the mechanism open. Output paths printed by conversion are authoritative. See [kinematics](models/LOADER_KINEMATICS.md).

## Optional physical checks

In a separate full Isaac Sim 6.2 Python environment, install this project, accept NVIDIA's license on first use, then run:

```bash
python scripts/check_wa320_physx.py --usd outputs/wa320/usd_fixed/wa320_fixed.usda --fixture --output outputs/wa320/fixture
python scripts/check_wa320_physx.py --usd outputs/wa320/usd/wa320.usda --output outputs/wa320/ground
python scripts/check_wa320_physx.py --usd outputs/wa320/usd/wa320.usda --pile --output outputs/wa320/rigid_pile
```

`--capture` optionally saves a verification image; it adds no camera observations or perception pipeline. Fixed-base tests must use the fixed asset; driving tests must use the floating asset. Logs/media/raw motion remain ignored under `outputs/`.

The PhysX and Isaac Lab/Newton probes are diagnostics, not a registered training environment. In the separate Isaac Lab 3 / Newton environment, run the committed fixed asset:

```bash
python scripts/check_wa320_isaaclab.py --usd assets/wheel_loader/wa320/usd/wa320_fixed.usda
python scripts/check_wa320_isaaclab.py --usd assets/wheel_loader/wa320/usd/wa320_fixed.usda --batch-reset --output outputs/wa320/batch_reset
```

The default numerical profile is `configs/simulation/newton.yaml`; override it with `--physics-config`. It keeps 240 Hz control with eight solver substeps. A derived Newton asset and effective solver model are saved under the output directory before scene import. Source assets remain unchanged.

For floating rigid-ground driving, use the floating asset:

```bash
python scripts/check_wa320_isaaclab.py --usd assets/wheel_loader/wa320/usd/wa320.usda --ground --output outputs/wa320/newton_ground
```

This creates an analytic plane locally with configured estimated friction and records rear-frame poses, wheel contact forces, joint states and collision geometry at 240 Hz. The passing ground approximation is selected by `configs/simulation/newton_ground.yaml` (`--ground-config` to override): analytical cylindrical tire contact, Newton solver, elliptic friction cone and a provisional 40 ms contact reference. Body mass/inertia and the committed PhysX assets are unchanged. An explicit consistent joint pose is written and checked before stepping; floating reset/replay is also checked.

Create a playable replay from the recorded poses in the same Newton environment, with system `ffmpeg` installed:

```bash
MUJOCO_GL=egl python scripts/render_wa320_replay.py --input outputs/wa320/newton_ground
```

The command writes `replay.mp4` locally. It displays collision geometry with simple colors and replays recorded motion without advancing physics. Appearance does not establish physical accuracy.

The Newton fixture passes closure, six driven-joint stop challenges, sampled self-collision checks and repeatable reset. The separate two-environment test checks reset isolation. These checks do not validate physical accuracy, calibrated driving, material loading or training readiness. Keep simulator-specific behavior inside `environment/simulation/`.

- [Parameter/calibration status](calibration/WA320.md).
- [Validation record](experiments/WA320_FOUNDATION.md).
- Previous particle demos and Komodo tools: `extras/legacy_loading` and `extras/komodo`.

## Full V-cycle motion rehearsal

- [Scenario definitions, commands and acceptance checks](experiments/V_CYCLE.md).
- Run the V-cycle script with --case all for nominal, slow and wide cases. Each case tests the entire maneuver; these are unladen motion tests, without sand acquisition or payload transfer.
