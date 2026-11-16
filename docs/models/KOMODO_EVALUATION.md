# Komodo model evaluation

## Recommendation

- Keep Komodo as an **optional, small skid-steer research baseline**. It is not a replacement for a full-size loader with articulated chassis steering.
- Faithful conversion is blocked by missing arm/bucket meshes and physically inconsistent wheel inertias. A supported importer can convert the **explicit diagnostic variant**, which has different attachment geometry and wheel inertias.
- Preserve the current domain contracts and simulator boundaries. No legacy ROS, Gazebo, Python 2, TensorFlow, hardware plugin or actuation interface is integrated.
- Before the next loading milestone: obtain dimensioned/licensed attachment geometry, resolve inertias and limits, establish frame/sign conventions, and select the intended steering mechanism. Validate material contacts separately.

## Source and complete mapping

- Audited revision: [`d53759100ced7439dd501620f955f347087e4f63`](https://github.com/osheraz/komodo/tree/d53759100ced7439dd501620f955f347087e4f63).
- Inputs: [`komodo2.xacro`](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/komodo2_description/urdf/komodo2.xacro), [`komodo2_gazebo.xacro`](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/komodo2_description/urdf/komodo2_gazebo.xacro), and [`komodo2_control.yaml`](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/komodo2_control/config/komodo2_control.yaml).
- [Complete source mapping](komodo_mapping.json) and [source-to-runtime link map](komodo_link_mapping.csv): all 19 links and 18 joints, origins, axes, limits, visual/collision geometry, masses, COMs, full inertia tensors, principal moments and mesh hashes. [Kinematics equations](LOADER_KINEMATICS.md).
- Seven principal rigid components: chassis/base, four wheels, arm and bucket. `base_arm` is fixed to the chassis; sensor/optical frames account for the other fixed links. No central articulation or wheel steering joints exist.
- Original moving-link mass total is approximately 51.477 kg including sensor inertials. Diagnostic sensor removal gives 51.464 kg. This is a miniature platform, not an industrial wheel loader.

## Links, geometry and physical parameters

- `base_link`: 40.0058 kg, COM (0,0,.0749) m; diagonal inertia (.385,1.072,1.1919) kg m². Collision box .531×.275×.2 m at COM height. Visual `base.STL`: 25,000 triangles, about .569×.327×.219 m bounds.
- Four wheels: 1.8524 kg each, local +Y rotation, continuous angles, effort .05 Nm, velocity 32 rad/s. Collision cylinders have radius .127 m and width .05 m. Visual wheel mesh has 6,780 triangles and about .084 m width; collision width is narrower.
- Each wheel inertia has principal moments near (.006905,.006905,.17) kg m². The largest exceeds the sum of the other two, so it cannot describe a rigid body's mass distribution. Positive definiteness alone does not make it physically valid.
- `base_arm`: 1.6502 kg, COM approximately (0,.080906,.065805) m; isotropic inertia .05 kg m². Its mesh is missing.
- `arm`: 1.6541 kg, COM (0,.38607,-.022642) m; diagonal (.020029,.0233,.041275) kg m², with nonzero `iyz=.0034648`. Local -X hinge, limits [-.1,.32] rad, effort 100 Nm, speed .2 rad/s, damping 40 and friction .1 in the URDF. Mesh is missing.
- `bucket`: .74403 kg, COM (0,.081094,-.0092968) m; diagonal (.00060709,.0060723,.0066375) kg m², `iyz=-4.2694e-6`. Local -X hinge, limits [-.5,.9] rad, effort 30 Nm, speed .01 rad/s, damping 2 and friction .9. Mesh is missing. A full 1.4-rad sweep at that speed would take 140 s.
- No cylinder stroke, linkage ratios, hydraulic dynamics, wheel/tire compliance, payload-rated limits or manufacturer-calibrated parameters are provided. The two independent attachment hinges do not model a Z-bar/four-bar linkage.

## Actuator and contact interfaces inspected

- Gazebo transmissions declare `VelocityJointInterface` for all wheels and `EffortJointInterface` with reduction 1 for arm/bucket. They are source metadata, not imported hardware interfaces.
- Legacy control uses differential/skid steering; configured radius .1248 m and separation .396 m differ from URDF collision radius .127 m and wheel-centre track .3791 m. Its slip factor is .630. Do not silently copy these into a kinematic adapter.
- Legacy attachment controllers are effort-based position PID: arm P=1000, I=.1, D=10; bucket P=70, I=.1, D=.01. Diagnostic PD removes integral action and does not reproduce those controllers exactly.
- Gazebo tire contact specifies `mu1=mu2=1.5`, `slip1=.5`, `slip2=0`, `kp=1e6`, `kd=100`. These are engine-specific contact settings, not transferable calibrated Newton/PhysX coefficients. Plugins, force sensor and optional sensors are excluded.

## Mesh provenance and redistribution

- Repository root has an [MIT license](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/LICENSE), with a 2020 upstream copyright notice. The description package manifest still says `TODO` for its license.
- Only `base.STL` and `wheel.STL` are present for this robot. Both have generic `VCG` STL headers, without CAD-author or asset-specific attribution. Root license evidence exists; original CAD provenance and mesh-specific permissions are not independently established.
- Missing: `arm2/meshes/base_arm.STL`, `arm2/meshes/arm.STL`, `arm2/meshes/bucket.STL`, and `lms1xx/meshes/sick-lms1xx.stl`. Unrelated perception-object meshes are not substitutes.
- Converted mesh assets remain local under ignored outputs. Keep the upstream license alongside local derived files; do not publish the source meshes or derived USD until provenance is resolved. No author contact has been sent.

## Comparison with the existing approximation

- Existing `loader_approximation.xml`: three actuated coordinates (rail travel, lift, curl), fixed wheel geometry, approximately 7770 kg, invented boom/bucket geometry and gains. It tests coarse material contact, not realistic loader kinematics or chassis articulation.
- Komodo: six actuated joints (four rotating wheels, arm, bucket), approximately 51 kg, CAD chassis/wheel visuals, two missing attachment meshes plus missing fixed mount mesh. Steering is wheel-ground skid steering; there is still no hinged chassis.
- Both use open two-hinge attachment chains, without a mechanical linkage model. Their axes, origins, angle limits, speed/effort limits and scale are different. Existing bucket containment/accounting geometry cannot be reused for Komodo.
- The old three-action loading contract stays unchanged. Model-specific actuator mapping and a separate steering-capable motion contract are prerequisites for task integration.

## Supported conversion and explicit diagnostic changes

- Conversion uses NVIDIA's `isaacsim.asset.importer.urdf.URDFImporter` 6.2.0.0, through its supported standalone distribution. See the [URDF importer documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html) and [Isaac Lab 3 installation/importer instructions](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html).
- Expand only the description with modern standalone `xacro==2.1.1`. No ROS package loading or legacy stack is needed. Strict preparation refuses missing meshes and invalid inertias.
- `--diagnostic` explicitly removes sensor links (including missing lidar visual), substitutes original primitive shapes for the missing mount/arm/bucket, and repairs wheel tensors using uniform cylinders of radius .127 m, width .05 m and mass 1.8524 kg, with COM at the cylinder centre.
- The bucket proxy is .32 m wide, .24 m deep and .15 m high with compound box walls; it is not recovered CAD and must not be called an accurate Komodo bucket.
- Preserve the six moving joint names, axes, origins and limits. Fixed-joint merging combines `base_link` and `base_arm` into `base_footprint`, including their inertial transforms; wheel, arm and bucket bodies remain separate. Sensor frames are removed in the diagnostic variant, not renamed to physical bodies.
- Import with visual-derived collisions disabled, source primitive collision shapes retained, self-collision enabled, and force drives. Fixed-base output is a suspended joint-isolation fixture; free-base output is a separate conversion.
- The importer emits layered USD with `Physics` variants `none`, `physics`, `mujoco` and `physx`. The conversion wrapper saves `physx` as the default for opening in Isaac Sim. Isaac Lab/Newton explicitly selects `mujoco`; Isaac Sim capture selects `physx`. A successful conversion does not establish physical accuracy.
- Every local conversion writes preparation and conversion manifests. Asset publication is held; scripts, mapping and documentation are public.

## Reproduction and validation

- Obtain the pinned upstream checkout outside the repository. Install the optional audit extra with `.venv/bin/uv pip install --python .venv/bin/python -e '.[model-audit]'`.
- `scripts/inspect_komodo.py --source SOURCE` produces the full local audit. `scripts/prepare_komodo.py --source SOURCE` refuses the incomplete original. Add `--diagnostic` only for the documented substitute model.
- Run `scripts/convert_komodo.py --urdf INPUT --output OUTPUT --fix-base` using the separate standalone-importer environment; omit `--fix-base` for free-base output. The returned asset path is authoritative: the importer may append a suffix on repeated conversions.
- Run `.venv/bin/python scripts/validate_komodo.py --usd USD_PATH` for Isaac Lab/Newton checks. Raw poses, effective solver model and results stay local.
- The isolated Isaac Sim environment runs `scripts/capture_komodo_isaacsim.py --usd USD_PATH`; its license must be accepted by the user. It uses PhysX and RTX, not the Newton preview renderer.

## Verified results and limitations

- **Pure checks:** 70 tests pass, including source topology/limits, URDF frame composition, impossible wheel-inertia detection, strict conversion refusal and diagnostic inertia consistency. Source-data checks use a curated mapping; actual conversion/runtime checks are separate.
- **Isaac Lab/Newton, fixed-base diagnostic:** all six moving joints load; wheels rotate and reverse differentially; arm and bucket move; nominal motion remains inside the source angle ranges. Relative link transforms match the URDF FK within 1.94e-7 m position and 4.75e-6 rotation-matrix norm.
- **Joint-stop challenges FAIL:** targets .15 rad beyond each stop are applied only in the diagnostic test. Maximum excess: arm upper .01191 rad, arm lower .00969 rad, bucket upper .08931 rad, bucket lower .08254 rad. These exceed the declared .005-rad tolerance. Do not treat source limits as hard guarantees under these drives/solver settings. Production command clamping, stop/contact tuning and full rate-limit checks remain necessary.
- **Self-collision:** 25 static joint configurations checked against the exported Newton model; maximum sampled self-penetration was zero. An artificial overlap between a chassis collider and the nonadjacent bucket is detected in a CPU-only model copy, confirming that the filter allows self-contact. This does not validate collision geometry that is missing from upstream.
- **Free-base ground check partly FAILS:** with source wheel effort .05 Nm and target 8 rad/s, five seconds of straight command gives .3549 m displacement; differential commands change yaw by only -.000712 rad. Straight screen (> .1 m) passes; turn screen (> .1 rad) fails. Contact assumptions are uncalibrated; this does not prove a real Komodo cannot turn.
- **Isaac Sim 6.2/PhysX/RTX diagnostic smoke:** six DOFs load and stay finite. Each wheel rotates about 3.83 rad, the arm reaches about .161 rad and the bucket about .040 rad. A local RTX screenshot verifies appearance. No claim of full PhysX stop/traction validation is made.
- **Drive conversion correction:** the importer applies target-type selection before gain overrides. Explicitly use zero wheel position stiffness and .2 Nms/rad velocity damping; assigning .2 to stiffness would recreate a position drive. USD angular gains are stored per degree, so .2 Nms/rad becomes .00349066 Nms/degree. Isaac Lab independently configures its actuator gains in SI units.
- Faithful original USD, accurate attachment collision, industrial scale, hinged chassis steering, productive scooping, hydraulic dynamics and M0 completion remain **unverified or unavailable**. The diagnostic conversion and screenshot do not clear mesh redistribution.

## Isolated optional environments

```bash
# Leave the existing .venv Newton runtime unchanged.
.venv/bin/uv venv .local/komodo-importer --python .venv/bin/python
.venv/bin/uv pip install --python .local/komodo-importer/bin/python \
  isaacsim-asset-isolated==6.2.0.0 \
  --overrides https://raw.githubusercontent.com/isaac-sim/IsaacLab/v3.0.0/tools/wheel_builder/uv-overrides.txt \
  --index https://pypi.nvidia.com --index-strategy unsafe-best-match
.venv/bin/uv pip install --python .local/komodo-importer/bin/python --no-deps -e .
```

- The full optional rendering environment uses the same pattern in `.local/komodo-isaacsim`, installing `isaacsim[all,extscache]==6.2.0.0` instead. Do not install it over the Newton environment. Its first launch requests the NVIDIA license acceptance; acceptance must come from the user.
- Conversion outputs and captures are intentionally ignored. Keep the entire layered USD folder together when opening the local interface asset.
