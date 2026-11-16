# Loader kinematics

## Current `loader_approximation.xml`

- This is a rail-mounted, open serial chain with one prismatic and two revolute joints. It is **not** an articulated steering chassis, a Z-bar linkage, or a hydraulic loader model. Its wheels are fixed shapes.
- World frame: +X forward, +Y left, +Z up; metres and radians. Joint coordinates are travel `s`, lift `l`, curl `c`. Positive lift/curl rotate about **-Y**, raising a point ahead of the hinge.
- Chassis origin: `(-3+s, 0, 0.7)`; boom pivot: `(-2.5+s, 0, 1.0)`; pivot-to-bucket offset at zero lift: `a=(2,0,-0.96)`.
- For a point `u=(ux,uy,uz)` fixed in the bucket, with column-vector rotations:

```text
p_world = (-2.5+s, 0, 1) + Ry(-l) a + Ry(-(l+c)) u
R_world_bucket = Ry(-(l+c))

x = -2.5+s + 2 cos(l) + .96 sin(l) + ux cos(l+c) - uz sin(l+c)
y = uy
z = 1 + 2 sin(l) - .96 cos(l) + ux sin(l+c) + uz cos(l+c)
```

- The bucket origin uses `u=(0,0,0)`; the front floor lip uses `u=(1.4,0,0.02)`. Curl changes bucket orientation about its back hinge; it does not move that hinge. Bucket pitch measured as nose-up angle is `l+c`.
- Joint ranges: `s∈[-1,4] m`, `l∈[0,.8] rad`, `c∈[-1.3,.9] rad`. Script target-rate limits: 1 m/s, .4 rad/s, .6 rad/s. These are assumptions, not machine specifications.
- The boom starts with a geometric slope `atan2(-.96,2)≈-25.64°`; lift is an increment from that pose. Scripted joint targets feed position PD drives directly. There is no inverse-kinematics solver, linkage solver or end-effector trajectory controller.
- These equations describe the reference frames, not cylinder strokes, forces, traction, linkage mechanical advantage, or physical accuracy. The existing simulation probe remains a material-contact experiment.

## Komodo source chain

- Source: [pinned `komodo2.xacro`](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/komodo2_description/urdf/komodo2.xacro).
- Komodo has four continuous wheel joints about local +Y, plus `arm_joint` and `bucket_joint` about local -X. It has **no steering hinge, steering knuckles, four-bar linkage, or hydraulic cylinders**.
- Wheelbase is .33 m and wheel-centre track is .3791 m. URDF collision radius is .127 m. Skid-steering turns the base by differential wheel-ground forces, not by changing a chassis angle.
- `base_footprint → base_link` includes a -π yaw. The mount lies at -X of base_link, but the arm extends toward **+X of base_link**: the bucket hinge is at approximately x=+.3985 m at zero joint angles. The -π base yaw puts this hinge toward **-X of base_footprint**. This frame/sign mismatch needs an explicit validated adapter convention. Do not equate the local joint axes with world axes.
- For homogeneous transforms `T(x,y,z)`, fixed-axis URDF `R(r,p,y)=Rz(y) Ry(p) Rx(r)`, and `Raxis(a,q)`:

```text
H_footprint_bucket =
  T(0,0,.1275) Rz(-3.14159265358)
  T(-.3,0,.17) Rz(-1.57079632679)
  T(0,.068765,.18244) R(-.0065548, 8.1987e-19, 1.4009e-16)
  Raxis((-1,0,0), q_arm)
  T(0,.63123,-.22318) R(.0093956,-2.6528e-16,1.0777e-16)
  Raxis((-1,0,0), q_bucket)
```

- Direction check from the source FK: increasing arm angle from 0 to .2 rad lowers the bucket hinge from z≈.2526 m to z≈.1320 m in `base_footprint`. A raising command therefore needs decreasing arm angle near zero. This differs from the old approximation’s positive-up lift convention. For the diagnostic proxy’s +Y-facing opening, increasing bucket angle tips the lip downward; original cutting-edge semantics remain unverified without its mesh.
- Arm range [-.1,.32] rad; bucket range [-.5,.9] rad. Bucket orientation depends on both coordinates. Its hinge position depends on the arm coordinate alone.
- A cutting-edge pose cannot be recovered accurately without the missing bucket mesh or a dimensioned drawing. The diagnostic primitive bucket supplies a separate, explicitly assumed shape.
- [Complete joint/link mapping](../../komodo/docs/komodo_mapping.json) records each origin, axis, limit, mass, inertia and geometry. [Model evaluation](../../komodo/docs/KOMODO_EVALUATION.md) records conversion differences and limitations.
