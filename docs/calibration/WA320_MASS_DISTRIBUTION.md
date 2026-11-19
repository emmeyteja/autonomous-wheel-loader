# WA320 reconstructed v1 mass distribution

- SI units; X forward, Y left, Z up. Report pose: flat ground, zero articulation, empty bucket, boom 0.15 rad, crank -0.05 rad. Root is the rear-frame hinge at Z=1.1 m.
- Operating mass is frozen at **15175 kg**. Existing frame masses are subdivided into structural/housing bodies; no residual correction, mass scaling, anti-tip force or extra ballast is used to close the budget.
- `configs/vehicle/wa320.yaml` is canonical: component totals in `masses`, allocation/estimated inertia primitives in `bodies`, independent contact geometry in `collision_geometry`, and cylinder geometry/limits/actuation in their own sections. Exported URDF/USD is generated.

## Component budget

| Component group | kg | Provenance / interpretation |
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
| lift_cylinders | 222.0 | DERIVED: Two 111 kg service-manual cylinder masses; explicit moving barrel/rod bodies with estimated 60/40 mass allocation. |
| bucket_cylinder | 196.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| steering_cylinders | 42.0 | DERIVED: Two 21 kg service-manual cylinder masses; explicit moving barrel/rod bodies with estimated 60/40 mass allocation. |
| wheel ×4 | 1364.0 | SERVICE_MANUAL: SEN06202-03 component reference; inertia/COM remain estimated. |
| **Total** | **15175.0** | Frozen operating-mass reference |

- Rear group: manual bare frame 1206 kg + counterweight 1815 kg + estimated guards/attachments 250 kg. Rear hinge housing/pins receive an estimated 160 kg of this existing group; the remaining rear-frame body is 3111 kg. Counterweight contribution remains 1815 kg.
- Front group: manual 1226 kg, divided into 1116 kg frame structure and estimated 110 kg hinge sleeve/neck. These allocations are assumptions, not separately weighed parts.
- Cab: manual cab/floor 838 kg + seat 50 kg. Installed powertrain 2400 kg and rear hydraulic/equipment 1166 kg remain high-uncertainty estimates. The manual 680 kg dry engine and relevant 1260 kg dry component subtotal do not establish installed-group masses.
- Bucket: manual 1328 kg for 2.8 m³ is an **estimated proxy** for the selected 3.0 m³ bucket. The manual also lists 1326 kg for 2.9 m³; neither establishes the 3.0 m³ mass.
- Component-table link/lift-cylinder/bucket-cylinder references are 68/111/196 kg. Disassembly references include 45/110/200 kg for differently described assemblies. The selected table values are retained; exact installed assembly applicability remains unresolved.
- Cylinder groups now have moving pinned barrels and prismatic rods, with estimated 60/40 mass splits. They are no longer fixed mass proxies. Structural clearance changes move estimated equipment rearward and alter frame COM/inertia; total masses remain unchanged.

## Every body COM and estimated inertia

- COM values below are in the common ground/world frame at the report pose. Tensors are about the body COM in body axes. Box, solid-cylinder and hollow-ring primitives use rotated analytic tensors and parallel-axis shifts. All placements, shape density assumptions and tensors are estimates, even when component mass is documented.
- Every body has physical geometry. The two frame housings are fixed subassemblies; the front/rear connection remains one Z-axis revolute joint. Cylinder hinges and prismatic rods move physically; bucket and link remain passive.

| Body | kg | World COM X,Y,Z [m] | Ixx,Iyy,Izz [kg m²] | Ixy,Ixz,Iyz [kg m²] |
|---|---:|---|---|---|
| rear_frame | 3111.0 | -2.3643, 0.0000, 1.0046 | 1110.27, 3879.44, 4761.00 | 0.00, -68.46, 0.00 |
| engine_powertrain | 2400.0 | -1.7025, 0.0000, 0.9475 | 312.01, 727.12, 869.71 | 0.00, 69.62, 0.00 |
| cab | 888.0 | -0.9000, 0.0000, 2.3500 | 348.73, 338.92, 259.93 | 0.00, 0.00, 0.00 |
| hydraulic_system | 1166.0 | -1.3300, 0.0000, 1.1750 | 205.59, 224.51, 231.62 | 0.00, 57.72, 0.00 |
| rear_axle | 677.0 | -1.5150, 0.0000, 0.6950 | 199.24, 23.36, 189.70 | 0.00, 0.00, 0.00 |
| front_frame | 1116.0 | 0.9419, 0.0000, 1.0511 | 494.28, 320.50, 460.49 | 0.00, 32.41, 0.00 |
| front_axle | 714.0 | 1.5150, 0.0000, 0.6950 | 210.13, 24.64, 200.07 | 0.00, 0.00, 0.00 |
| boom | 1313.0 | 1.9968, 0.0000, 1.2846 | 263.46, 634.28, 837.28 | 0.00, -18.05, 0.00 |
| bellcrank | 300.0 | 2.5004, 0.0000, 1.4848 | 3.10, 53.46, 52.57 | 0.00, 6.23, 0.00 |
| rod | 68.0 | 2.6340, 0.0000, 0.8328 | 0.26, 3.26, 3.26 | 0.00, 0.00, 0.00 |
| bucket | 1328.0 | 3.4524, 0.0000, 0.7797 | 1283.52, 480.60, 1385.38 | 0.00, 85.95, 0.00 |
| rear_left_wheel | 341.0 | -1.5150, 1.0250, 0.6950 | 49.46, 82.36, 49.46 | 0.00, 0.00, -0.00 |
| rear_right_wheel | 341.0 | -1.5150, -1.0250, 0.6950 | 49.46, 82.36, 49.46 | 0.00, 0.00, -0.00 |
| front_left_wheel | 341.0 | 1.5150, 1.0250, 0.6950 | 49.46, 82.36, 49.46 | 0.00, 0.00, -0.00 |
| front_right_wheel | 341.0 | 1.5150, -1.0250, 0.6950 | 49.46, 82.36, 49.46 | 0.00, 0.00, -0.00 |
| rear_hinge_housing | 160.0 | -0.1640, 0.0000, 1.0200 | 19.03, 26.90, 14.68 | 0.00, -0.00, 0.00 |
| front_hinge_housing | 110.0 | 0.1155, 0.0000, 1.0200 | 3.62, 6.59, 5.25 | 0.00, 0.00, 0.00 |
| lift_left_barrel | 66.6 | 0.9117, 0.2400, 1.2690 | 7.78, 7.78, 0.19 | 0.00, 0.00, 0.00 |
| lift_left_rod | 44.4 | 1.2973, 0.2400, 1.2627 | 3.21, 3.21, 0.03 | 0.00, 0.00, 0.00 |
| lift_right_barrel | 66.6 | 0.9117, -0.2400, 1.2690 | 7.78, 7.78, 0.19 | 0.00, 0.00, 0.00 |
| lift_right_rod | 44.4 | 1.2973, -0.2400, 1.2627 | 3.21, 3.21, 0.03 | 0.00, 0.00, 0.00 |
| bucket_barrel | 117.6 | 1.6185, 0.0000, 2.1150 | 14.44, 14.44, 0.48 | 0.00, 0.00, 0.00 |
| bucket_rod | 78.4 | 2.2763, 0.0000, 2.2146 | 3.81, 3.81, 0.12 | 0.00, 0.00, 0.00 |
| steering_left_barrel | 12.6 | 0.2281, 0.3976, 1.0200 | 0.02, 0.48, 0.48 | 0.00, -0.00, 0.00 |
| steering_left_rod | 8.4 | 0.6162, 0.4535, 1.0200 | 0.00, 0.30, 0.30 | 0.00, -0.00, 0.00 |
| steering_right_barrel | 12.6 | 0.2281, -0.3976, 1.0200 | 0.02, 0.48, 0.48 | 0.00, -0.00, 0.00 |
| steering_right_rod | 8.4 | 0.6162, -0.4535, 1.0200 | 0.00, 0.30, 0.30 | 0.00, -0.00, 0.00 |

- `total_vehicle_COM_m`: [-0.2597, 0.0000, 1.0669] m.
- `front_frame_COM_m`: [0.9419, 0.0000, 1.0511] m.
- `rear_frame_COM_m`: [-2.3643, 0.0000, 1.0046] m.
- `rear_body_COM_m`: [-1.7379, 0.0000, 1.0965] m.
- Rear-body aggregate: 9126.0 kg. Frame-body COM excludes the separately listed hinge housing; the rear aggregate includes all rear descendants, including steering-cylinder mass.

## Static axle checks

- Empty transport moment balance: front **61676.1 N / 6287.1 kg-equivalent / 41.43%**; rear **87190.7 N / 8887.9 kg-equivalent / 58.57%**.
- These are model predictions, not Komatsu axle-load measurements. Straight, flat, two-axle support is assumed; rear-axle oscillation is locked in this model.
- Dynamic solver checks also hold lowered, transport, raised, curled and 2000 kg rigid-payload poses. They require total measured normal force within 3% of weight, positive mean load at every wheel, speed below 0.01 m/s in the final 2 s, and front load within 3% of total weight from the independent moment prediction.
- A rigid retained/released ballast checks load transfer; it is not acquired material. Contact/pump/tire calibration remains unknown.

## Reproduce

```bash
python scripts/check_wa320_reconstruction.py --output outputs/wa320/mechanical_correction/mechanics.json
python scripts/check_wa320_dynamics.py --source <Newton-exported-effective_model.xml> --output outputs/wa320/mechanical_correction/release_dynamics
python -m pytest tests/test_wa320_mass_budget.py tests/test_wa320_static_load.py tests/test_wa320_mechanical_structure.py
```

- The local report contains full tensors, per-body COMs, parameter fingerprints and solver results. Raw source PDFs, traces and videos remain outside Git.
