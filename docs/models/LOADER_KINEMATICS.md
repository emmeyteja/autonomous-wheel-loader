# WA320-like kinematics

Original primitive geometry; no Komodo geometry and no manufacturer CAD. The architecture follows Aoshima's WA320-7 description. All unmeasured pivots and limits are explicitly estimated in `configs/vehicle/wa320.yaml`.

## Chassis and wheels

- Rear frame → central revolute joint about +Z → front frame. Positive articulation yaws the front left relative to the rear.
- Rear and front axle centres sit at −wheelbase/2 and +wheelbase/2 along their respective frames. Four wheels rotate about local +Y; positive rotation drives forward.
- No axle oscillation, tire compliance, differential/driveline dynamics or power sharing yet.
- Boom and linkage elevation rotations use local −Y: positive planar X/Z angle raises a forward-pointing member.

For equal hinge-to-axle distances `L = wheelbase/2`, rear speed `v`, articulation `a` and articulation rate `a_dot`, the no-slip reference is:

```text
rear yaw rate  w_r = (v sin(a) − L a_dot) / (L cos(a) + L)
front yaw rate w_f = w_r + a_dot
front speed   v_f = v cos(a) + L w_r sin(a)
wheel rates       = (axle speed ∓ track_width/2 × axle yaw rate) / wheel_radius
```

The minus sign applies to the left wheel. This is a reference controller; contact slip may violate these kinematic assumptions.

## Closed Z-bar geometry

In the boom's planar X/Z frame:

```text
B = boom pivot; P = B + [boom_length, 0]             (bucket pin)
C = B + [boom_length × pivot_fraction, 0]           (bellcrank pivot)
E = C + R(phi) [upper_length, 0]                    (tilt-cylinder pin)
D = C − R(phi) [lower_length, 0]                    (rod pin)
F = P + R(beta) bucket_anchor                      (bucket attachment)
|F − D| = rod_length                               (loop closure)
```

`phi` is the driven bellcrank angle relative to the boom; `beta` is the passive bucket angle relative to the boom. Solve `F` from the intersection of the circle about `P` with radius `|bucket_anchor|` and the circle about `D` with radius `rod_length`. Keep one assembly branch; reject disjoint/tangent circles rather than switching branches or silently stretching a bar. Rod orientation is `atan2(F_z−D_z, F_x−D_x)`.

The front-frame pose adds boom elevation and pivot translation. Central yaw then maps the entire front assembly into the rear frame. Nominal offsets are baked into URDF joint origins; joint positions are displacements from those offsets.

- URDF contains the spanning tree. `closure.json` specifies the missing rod-to-bucket revolute joint.
- Isaac conversion adds that passive USD loop joint and marks it excluded from the reduced-coordinate articulation, following [NVIDIA's closed-loop workflow](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html#closed-loops).
- Rod and bucket have no drives. Physics must enforce the loop; independent servo targets or visual pose updates do not count.
- The tilt-cylinder length is computed from the fixed front-frame anchor to `E`. It is geometry metadata only. Current boom/bellcrank rotary drives do not model the paper's two lift cylinders and one tilt cylinder.
- This is an estimated Z-bar topology, not a verified reproduction of Komatsu's proprietary parallel/PZ linkage or hydraulic ratios.
- A rectangular joint envelope is not a collision-free workspace. Ground/bucket clearance and self-collision need simulator checks over actual operating paths.

The former rail-constrained model and its kinematics are archived in `extras/legacy_loading`.

## Newton loop representation

- The simulation integration prepares a local derived asset before scene import; the committed PhysX assets are unchanged.
- Two spherical connect constraints, separated by 0.1 m along the common hinge axis, enforce the same five constrained relative degrees of freedom as the rod/bucket revolute closure. The original loop joint is disabled only in this derived asset.
- Both connects remain outside the reduced-coordinate tree. Bucket and rod remain unactuated.
- `configs/simulation/newton.yaml` sets solver subdivision and loop/stop compliance. These are numerical settings, not measured mechanical or hydraulic compliance.
- The exported MuJoCo model is checked to ensure those settings reached the solver. See the [validation record](../experiments/WA320_FOUNDATION.md) for tested paths and remaining gaps.

- Newton ground contact optionally uses analytical tire cylinders with the original nominal radius/width, selected by the ground profile. Tire body inertia and wheel hinges remain unchanged; this is rigid contact geometry, not a deformable tire model.

- Recorded replay now displays estimated twin lift cylinders and a tilt cylinder, with pin geometry and world-frame state. They are visual overlays, not physical actuators; see [replay state](../experiments/REPLAY_STATE.md).

## Brochure-derived candidate

- [WA320-7 drawing reconstruction](WA320_BROCHURE_GEOMETRY.md) identifies a nonzero crank-pivot offset and a provisional triangular bellcrank. The opt-in reconstruction records explicit picks, residuals and ambiguity; it does not replace the active straight-bar closed loop or validate its forces.
