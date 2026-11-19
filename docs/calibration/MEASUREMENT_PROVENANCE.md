# WA320-7 measurement provenance

- Machine-measured values: **none supplied** for this project. A plausible picture or successful simulation is not a measurement.
- Published nominal values: manufacturer dimensions, operating mass, capacity and performance figures. These depend on the brochure configuration; rounding is not a tolerance. The published innermost-wheel turning radius is **3.060 m**, not 3060 m.
- Derived published values: wheel radius from nominal diameter and articulation limits from published steering angle. Tire deformation and actual steering stops are unmeasured.
- Diagram estimates: candidate boom, crank and bucket geometry reconstructed from brochure views. These remain separate from the active approximate vehicle.
- Engineering estimates: active unverified link geometry, mass distribution, inertia, contact and drive settings. Numerical solver settings are classified separately.
- Every active numeric parameter has units, source classification and an unknown real-vehicle error in [the ledger](../../configs/measurements/wa320_7.yaml). The generated report includes its current value. Coverage fails if parameters are added without classification.

## What the error numbers mean

- `true_vehicle_error: null`: unknown; never zero. Computing this requires an independently measured reference for the same configuration and pose.
- Diagram residual: difference from a picked feature or published dimension in the same drawing. It is an internal cross-check, not physical accuracy.
- Pick sensitivity: variation under assumed pixel errors. The boom-length 5th–95th percentile is 2.669–2.756 m around the 2.710 m estimate, using assumed ±3 pixel visible picks and ±1 pixel axle/ground picks. This is not a real-machine confidence interval.
- The candidate visible boom-base discrepancy is 31 mm. Bucket lip fit and held-out same-drawing residuals are 54 and 43 mm. Dump reach differs by +96 mm and clearance by −16 mm.
- The provisional crank-D hypothesis requires rod lengths differing by 121 mm across registered poses. It is rejected for promotion; neither hydraulic leverage nor forces should be calibrated against it.
- Detailed frames, picks, assumptions and reproduction commands: [boom reconstruction](../models/WA320_BROCHURE_GEOMETRY.md) and [bucket registration](../models/WA320_BUCKET_GEOMETRY.md).

## Reproduce

```bash
PYTHONPATH=src .venv/bin/python scripts/audit_wa320_measurements.py
```

- Output: ignored `outputs/wa320/measurements.json`; 72 active parameters at introduction; v1 extends coverage to component, geometry and actuator records.
- The [shop-manual reference](../models/WA320_SHOP_MANUAL.md) now supplies cylinder bore/rod/stroke/length envelopes and component weights for serial 80001+, with configuration differences. These are separate reference values, not active overrides.
- Missing evidence: matching full linkage and cylinder mounts, component COM/inertia, configuration confirmation, hydraulic chamber/valve data, tire and material calibration, sensor mount survey and synchronized real measurements.
- Preserve published, estimated and measured values separately when new evidence arrives. Re-run geometry, dynamics and maneuver checks before promoting candidate dimensions.
