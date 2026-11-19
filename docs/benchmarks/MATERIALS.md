# Material and vehicle benchmark applicability

- First full-loader reference: Aoshima/Servin gravel. Keep dry sand as a separate candidate; changing a material selector does not transfer calibration. Coal and cohesive soils need their own contact models and measurements.
- **Material-level:** reproduce the test apparatus, particle-size distribution, density/packing, moisture, gravity and preparation. Compare density, wall friction, repose, collapse/runout and shear response. No loader is required.
- **Tool-level:** reproduce bucket/blade geometry, cutting edge, penetration depth, orientation, speed/path, boundaries and material state. Compare force/torque histories, work, moved/retained mass and disturbed surface. A matched instrumented tool test can isolate soil physics before the whole vehicle is accurate.
- **Vehicle-level:** additionally match linkage geometry, cylinder ratios/limits, mass/COM/inertia, tires/traction, driveline, controls and initial conditions. Different vehicles cannot share an unqualified force, payload, cycle-time or sim-to-real error benchmark.
- Normalize quantities only for a justified similarity study. Mass/fill ratio, force divided by a declared characteristic force, and dimensionless speed/depth/grain-size ratios can help compare trends; normalization does not remove differences in bucket geometry, cohesion, packing or power limits. Record every normalization and its units.
- Fit material parameters on calibration experiments, then validate against held-out shapes, depths, speeds and loading cycles. Numerical stability, timestep agreement and matching one repose angle are separate from physical accuracy.

- [Selected gravel heap protocol, open scans and geometric measurement check](GRAVEL_HEAP_PROTOCOL.md).
- [Zhang gravel calibration: stationary slope, rotating drum and independent cylinder test](ZHANG_GRAVEL.md).
- [Obermayr blade-force protocol, assumptions and reproduction gaps](OBERMAYR_BLADE.md).

## References and suitable uses

- **Gravel/full loader:** [Aoshima and Servin, DOI 10.1007/s11044-024-10005-5](https://doi.org/10.1007/s11044-024-10005-5), especially §§4–6. The field gravel includes fines and some moisture; reported bulk density is 1727 kg/m³. Their DEM uses 2590 kg/m³ solid density and targets 32° repose. Natural grains and simulation elements are different sizes. Compare motion/forces, loaded mass and work only with matched tests. The paper states normalized time series are available on request; this repository does not contain those field signals. Its reported accuracy is not our result.
- **Dry quartz sand/material tests:** [Dastgerdi and Malinowska (2024), DOI 10.3390/ma17194833](https://doi.org/10.3390/ma17194833). Bulk-density, surface-friction and repose calibration, followed by independent column-collapse tests. Useful without a vehicle; copy the apparatus and sample preparation rather than treating all dry sand as identical. Non-spherical grains and their LS-DYNA contact law differ from our trial spheres.
- **Cohesionless soil/tool force:** [Obermayr et al. (2011), DOI 10.1016/j.jterra.2011.08.003](https://doi.org/10.1016/j.jterra.2011.08.003). Triaxial calibration and laboratory straight-blade draft-force comparison. Useful as a vehicle-independent force benchmark after matching tool geometry and cutting conditions; not direct validation of our bucket.
- **Cohesive soil:** [Obermayr et al. (2014), DOI 10.1016/j.jterra.2014.04.003](https://doi.org/10.1016/j.jterra.2014.04.003). Triaxial calibration plus small laboratory and excavator/field comparisons, using a stress-history-dependent cohesive model. A friction-only gravel backend cannot reproduce that physics merely by changing a coefficient.
- **Coal/coal rock:** [*Research on coal rock parameter calibration based on discrete element method* (2024), DOI 10.1038/s41598-024-77538-9](https://doi.org/10.1038/s41598-024-77538-9). Repose and compression-based contact/bond calibration. Applicable to its coal samples and apparatus; bonded coal-wall fracture is different from loose stockpiled coal loading. It is a material-test reference, not a wheel-loader force benchmark.

## Pile relaxation after excavation

- “Pile relaxation,” “slumping,” “local collapse” and “surface redistribution” describe material moving after support is removed. The stable slope is an emergent bulk response; do not freeze particles or impose a fixed angle during loading.
- Compare before/after surface height in a common surveyed frame, excavated/redistributed volume, runout and settling time. Define field of view, grid spacing and measurement uncertainty. Separate material retained by the bucket, dumped material and spillage in mass accounting.
- Bucket reaction forces require dynamic coupling to moving material. Our earlier rigid-pile contact force is not an excavation-force benchmark, and a released-column test cannot validate bucket forces or loaded stability.
