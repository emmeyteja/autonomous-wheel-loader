# WA320 reconstructed v1 mass distribution

- Coordinates: metres, X forward / Y left / Z up. World ground is Z=0; rear-frame origin is the central hinge at Z=1.1 m. Report pose: empty bucket, boom displacement 0.15 rad, bellcrank displacement -0.05 rad, zero articulation, flat ground.
- Operating mass is frozen at **15,175 kg**. No runtime residual correction, ballast correction, anti-tip force or mass scaling is used. The estimated installed groups were explicitly budgeted to this total; mass closure alone is not evidence their distribution is accurate.
- `configs/vehicle/wa320.yaml` is canonical. `masses` controls component totals; `bodies` controls mass allocation and inertia primitives; `collision_geometry` controls independently configurable exterior/contact shapes; actuator/controller/contact settings have separate sections. Geometry exports are generated.

## Mass budget

| Group | kg | Source / interpretation |
|---|---:|---|
| rear_frame | 3271.0 | ESTIMATED: 1206 kg bare frame + 1815 kg counterweight + estimated 250 kg guards/steps/structural attachments. COM uses three separate primitives. |
| engine_powertrain | 2400.0 | ESTIMATED: Estimated installed powertrain group: engine, transmission/HST, cooling, mounts, exhaust and omitted housings. Manual dry component subtotal is not a complete installed group. |
| cab | 888.0 | DERIVED: 838 kg cab/floor + 50 kg seat from manual; no duplicated cab mass in rear_frame. |
| hydraulic_system | 1166.0 | ESTIMATED: Explicit estimated rear equipment group: tanks, fluids, fuel, pumps, valves, hoses, electrical equipment and miscellaneous service equipment. Large uncertainty; independent measurements required. |
| rear_axle | 677.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| front_frame | 1226.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| front_axle | 714.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| boom | 1313.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| bellcrank | 300.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| rod | 68.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| bucket | 1328.0 | ESTIMATED: Manual 2.8 m3 bucket with BOC/hinge pin is a proxy for selected 3.0 m3 bucket; this mass is ESTIMATED for that option. |
| lift_cylinders | 222.0 | DERIVED: Two 111 kg manual cylinders; fixed barrel/rod mass proxies split 60/40, not a full cylinder dynamic mechanism. |
| bucket_cylinder | 196.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| steering_cylinders | 42.0 | DERIVED: Two 21 kg manual cylinders; split between rear/front fixed proxies. |
| wheel × 4 | 1364.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| **Total** | **15175.0** | Frozen operating-mass budget |

- Rear-frame group: 1206 kg bare rear frame + 1815 kg counterweight + explicitly estimated 250 kg attachments. Their three distinct locations determine its COM.
- Engine/powertrain is an estimated installed group, not a claim that the engine weighs 2400 kg. The manual's engine is 680 kg; listed dry cooling/HST/transfer/shaft components bring a relevant subtotal to 1260 kg. Remaining installed equipment, housings and mounts are not individually identified. This group and the 1166 kg hydraulic/rear-equipment group have substantial calibration uncertainty.
- Cab: manual cab/floor 838 kg + seat 50 kg. Hydraulic/rear equipment includes estimated tanks, fluids, fuel, electrical equipment and miscellaneous installed mass; their detailed allocation remains unknown.
- Manual 2.8 m³ bucket mass 1328 kg is used as an **estimated proxy** for the selected 3.0 m³ bucket. The primary configuration stays 3.0 m³ / 2.685 m width. Other bucket configurations do not block v1.
- Two lift cylinders total 222 kg; bucket cylinder 196 kg; two steering cylinders total 42 kg. Barrel/rod mass is split 60/40 into fixed mass proxies carried by their attachment bodies. This is not a complete moving cylinder multibody model. Steering mass is split between frames.

## Body COM and estimated inertia

- Every body below is exported separately. Fixed joints join installed components to their frame; the front/rear connection remains the articulation revolute joint. Wheels remain independently revolute, bucket and link passive.
- COM below is in the common ground/world frame for the report pose. Inertia is about the body's COM in its own axes. All COM placements and inertia estimates are engineering approximations, including those for components with documented masses.
- Primitive box/cylinder tensors are rotated into body axes, then summed with parallel-axis shifts. Compound off-diagonal terms are retained in URDF. No identical arbitrary inertia tensor is assigned to unrelated bodies.

| Body | kg | COM world XYZ [m] | Ixx, Iyy, Izz [kg m²] | Ixz [kg m²] |
|---|---:|---|---|---:|
| rear_frame | 3271.0 | -2.312, 0.000, 1.027 | 849.60, 3885.19, 4488.67 | -71.02 |
| engine_powertrain | 2400.0 | -1.703, 0.000, 0.948 | 312.01, 727.12, 869.71 | 69.62 |
| cab | 888.0 | -0.900, 0.000, 2.350 | 348.73, 338.92, 259.93 | 0.00 |
| hydraulic_system | 1166.0 | -1.083, 0.000, 1.175 | 205.59, 386.84, 393.95 | 122.65 |
| rear_axle | 677.0 | -1.515, 0.000, 0.695 | 199.24, 23.36, 189.70 | 0.00 |
| front_frame | 1226.0 | 0.850, 0.000, 0.900 | 277.89, 198.20, 425.01 | 0.00 |
| front_axle | 714.0 | 1.515, 0.000, 0.695 | 210.13, 24.64, 200.07 | 0.00 |
| boom | 1313.0 | 2.008, 0.000, 1.285 | 269.35, 677.73, 889.79 | -15.93 |
| bellcrank | 300.0 | 2.500, 0.000, 1.485 | 3.10, 53.46, 52.57 | 6.23 |
| rod | 68.0 | 2.683, 0.000, 0.842 | 0.26, 4.16, 4.16 | 0.00 |
| bucket | 1328.0 | 3.452, 0.000, 0.780 | 1283.52, 480.60, 1385.38 | 85.95 |
| rear_left_wheel | 341.0 | -1.515, 1.025, 0.695 | 49.46, 82.36, 49.46 | 0.00 |
| rear_right_wheel | 341.0 | -1.515, -1.025, 0.695 | 49.46, 82.36, 49.46 | 0.00 |
| front_left_wheel | 341.0 | 1.515, 1.025, 0.695 | 49.46, 82.36, 49.46 | 0.00 |
| front_right_wheel | 341.0 | 1.515, -1.025, 0.695 | 49.46, 82.36, 49.46 | 0.00 |
| lift_barrel_left | 66.6 | 1.050, 0.680, 1.300 | 5.90, 5.90, 0.69 | 0.00 |
| lift_rod_left | 44.4 | 1.415, 0.570, 1.456 | 0.07, 2.41, 2.41 | 0.00 |
| lift_barrel_right | 66.6 | 1.050, -0.680, 1.300 | 5.90, 5.90, 0.69 | 0.00 |
| lift_rod_right | 44.4 | 1.415, -0.570, 1.456 | 0.07, 2.41, 2.41 | 0.00 |
| bucket_barrel | 117.6 | 1.550, 0.000, 2.100 | 1.32, 10.46, 10.46 | 0.00 |
| bucket_piston | 78.4 | 2.609, 0.000, 1.956 | 0.22, 3.31, 3.31 | 0.00 |
| steering_rear | 21.0 | -0.450, 0.000, 0.750 | 1.15, 1.15, 2.24 | 0.00 |
| steering_front | 21.0 | 0.450, 0.000, 0.750 | 1.15, 1.15, 2.24 | 0.00 |

- Full 3×3 tensors, local COMs and per-body data are reproducibly exported to ignored `outputs/wa320/reconstructed_v1/mechanics.json`.
- `total_vehicle_COM_m`: [-0.2507, 0.0000, 1.0588] m.
- `front_frame_COM_m`: [0.8500, 0.0000, 0.9000] m.
- `rear_frame_COM_m`: [-2.3116, 0.0000, 1.0268] m.
- `rear_body_COM_m`: [-1.7328, 0.0000, 1.1037] m.
- Rear-body aggregate: 9105.0 kg, including rear wheels, axle, cab and installed rear groups; excludes the articulated front assembly.

## Static axle sanity

- Moment balance about the rear axle: front reaction = total weight × (COM_X + wheelbase/2) / wheelbase. Rear reaction is the remaining weight. This assumes straight articulation and two coplanar axle supports.
- Front: **6331.8 kg-equivalent / 62115 N / 41.72%**.
- Rear: **8843.2 kg-equivalent / 86752 N / 58.28%**.
- Both supports are positive and within the documented project sanity screen (front 30–60%, rear 40–70%). These are model predictions, not manufacturer axle-load measurements. Actual load distribution and COM require weighing/telemetry.

## Reproduce

```bash
PYTHONPATH=src python scripts/check_wa320_reconstruction.py
python -m pytest tests/test_wa320_mass_budget.py tests/test_wa320_static_load.py
```

- A known 2000 kg payload independently tests the expected front-load increase and rear-load reduction. Dynamic tests retain a separate rigid ballast through release onto a receiver; they do not simulate granular loading or infer payload from contact force.
