# Sensor replay diagnostic

- Offline CPU sensor synthesis over an accepted recorded Newton rigid-pile V-cycle. Physics is not advanced; the active approximate vehicle is unchanged. This is not the production batched training sensor backend.
- Core contracts and estimation live in `sensing/`; native raycasts and state extraction in `environment/simulation/`; reporting in `evaluation/`. No ROS or hardware connection.
- Configuration: [diagnostic.yaml](../../configs/sensors/diagnostic.yaml). Mount, noise, dropout and latency are trial estimates, not calibrated sensor specifications. See [measurement provenance](../calibration/MEASUREMENT_PROVENANCE.md).

## Paper reference and available channels

- [Aoshima and Servin, §§4.1–4.3, Table 1](https://doi.org/10.1007/s11044-024-10005-5): lift/tilt cylinder pressures and piston positions, boom/bucket angular measurements, wheel-revolution speed and hydraulic-drive-derived tractive force. Field logging is 100 Hz. Lift/tilt forces are pressure-derived and reported in arbitrary units; this does not supply our SI hydraulic calibration.
- That setup uses laser distance to a wall for position, a 2D pile scan before each loading, and built-in weighing after loading. Our multi-beam roof scanner is a project extension, not a reproduction of that instrumentation.
- Range scanner: 16 × 81 geometric rays at 5 Hz, 0.3–25 m, trial 20 mm Gaussian range noise and 1% dropout. Native MuJoCo raycasts retain vehicle occlusion over recorded geometry. Appearance textures are not sensed.
- Wheel odometry: ideal rear-wheel angle increments plus articulation at 60 Hz. Equal axle offsets, planar rolling and no slip are assumed. It never consumes root truth. Pose is rear axle XY/yaw relative to its initial pose; articulation is not an Ackermann wheel angle.
- Joint states: ideal recorded relative angles in radians at 60 Hz. IMU: rear-frame specific force and angular rate from finite differences; endpoint samples invalid. No calibrated encoder quantization or IMU bias/noise model.
- Bucket contact force: original recorded Newton net contact vector in world axes, newtons. It is neither a load cell nor payload weight.
- Hydraulic pressure, absolute cylinder stroke and payload mass: **unavailable**, with a reason and unit in the report; no fabricated numeric zeros. Rotary drive torque cannot substitute for chamber pressure, and contact force divided by gravity cannot weigh a dynamically contacting bucket.
- `gross_cylinder_force_N` supports known cap/rod pressures and bore/rod diameters: `p_cap A_bore − p_rod (A_bore − A_rod)`. The [shop manual](../models/WA320_SHOP_MANUAL.md) supplies reference cylinder dimensions and a lift-bottom sensor transfer curve, subject to configuration confirmation. This helper is not activated for this vehicle and excludes friction, leverage and power sharing.

## Frames, timing and map limits

- X forward, Y left, Z up. SI units; rear-frame mount translation in metres and configuration RPY in degrees; joint and odometry output angles in radians. RangeScan has batch/ray dimensions `[B,R]`, unit directions `[R,3]`, owned read-only arrays, NaN missing ranges and an explicit validity mask.
- Capture and arrival times are separate simulation timestamps. Arrival = capture + configured 20 ms; this offline mapper uses capture poses and does not demonstrate delayed real-time fusion. Sample rates divide the 240 Hz source; no interpolation to paper logging rates is claimed.
- Sensor points are transformed using estimated extrinsics and wheel odometry. Flat ground and fixed root height are assumed. Grid stores maximum observed surface height and counts; unseen cells remain NaN. It includes ground and vehicle returns: no pile segmentation, unseen-volume inference or payload accounting.
- Truth-aligned poses and ray geometry IDs are stored separately in `privileged_evaluation.npz`, only for diagnostic error/coverage checks. They are not estimator or map inputs. Simulator truth used to synthesize an instrument is not evidence of physical sensor accuracy.
- Batch/reset isolation is tested in estimator/noise contracts. Replay orchestration and CPU raycasts are deliberately outside the training path. A device-resident backend, episode reset integration, temporal filtering and pile-only mapping remain pending.
- Future ROS adapter candidates are standard point-cloud, odometry, IMU, joint-state, pressure and stamped-wrench messages where semantics match. Verify and pin message versions before implementation; no adapter exists here.

## Reproduce

```bash
PYTHONPATH=src .venv/bin/python scripts/check_sensor_replay.py --input outputs/wa320/rigid_pile_release/nominal
MUJOCO_GL=egl PYTHONPATH=src .venv/bin/python scripts/render_sensor_replay.py --input outputs/wa320/rigid_pile_release/nominal
```

- Requires a locally generated accepted motion recording, native MuJoCo/Pillow optional dependencies and ffmpeg for video. Outputs stay under ignored `outputs/wa320/sensors/`.
- Reports numerical validity, return coverage and wheel-odometry error against privileged simulator truth. These are not real-machine accuracy benchmarks or completed hydraulic/granular validation gates.
