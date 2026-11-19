# What the current simulator evidence supports

- Current use: an estimated rigid multibody model for scripted unladen driving and attachment-motion development. A real-machine error percentage is not yet available.
- Three complete V-cycle scenarios pass the same motion, route, limits, closure, contact and reset criteria. Final origin error is 0.129–0.245 m; passive-loop gap stays below 0.13 mm on those runs. These are internal numerical checks, not errors against a real loader.
- See [V-cycle results and acceptance thresholds](V_CYCLE.md), [foundation checks](WA320_FOUNDATION.md), and [physical parameter status](../calibration/WA320.md). Passing paths do not establish full-workspace safety or physical accuracy.
- Manufacturer/paper dimensions anchor overall scale. Pivots, mass distribution, inertia, contact compliance and rotary drives remain estimates. Hydraulic pressures/flows, driveline power sharing, tire deformation, soil excavation and loaded performance are not validated.

## Reference and comparison method

- [Aoshima and Servin, *Examining the simulation-to-reality gap of a wheel loader digging in deformable terrain*](https://doi.org/10.1007/s11044-024-10005-5) compares field-test motion and actuation-force time series, loaded mass and total work. Its approximately 10% aggregate gap is a result for that study's calibrated vehicle/gravel setup, not this repository and not a per-signal error bound.
- [Komatsu's WA320-7 product reference](https://www.komatsu.eu/en/product-archive/wheel-loaders/wa320-7) and the [research lineage](../references/aoshima.md) provide dimensional and mechanism references. Regional specifications and bucket options differ; retain one explicitly identified configuration for comparisons.
- A defensible fidelity assessment needs measured machine trajectories and actuator/load data under matched commands and initial conditions. Separate fitting runs from held-out validation runs, and report signal errors rather than a single unsupported accuracy score.
- On rigid ground, compare speed, stopping distance, yaw/articulation response, wheel slip, boom/bucket trajectories and cylinder lengths/speeds. Use position/orientation RMSE, peak deviation, settling time and repeatability, with measurement uncertainty.
- For hydraulics, compare pressures, forces, strokes, delays and load-dependent velocities only after those states and the relevant physical limits are implemented and measured.
- For sand, independently match bulk density, repose/settling and excavation response before comparing bucket fill, retained mass, spillage, dump-zone mass and mechanical work. Gravel parameters and coarse display geometry do not establish dry-sand fidelity.
- Numerical convergence is a separate check: repeat matched cases at finer timestep/solver settings and compare trajectories, contacts and loop residuals. Similar results at two resolutions do not, by themselves, show agreement with reality.
