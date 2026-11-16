# Aoshima research lineage

These sources motivate the architecture. Their simulation accuracy and trained-model performance are not results of this repository. User-supplied PDFs/videos are read locally, not redistributed.

- [Aoshima, *High-performance autonomous wheel loading – a computational approach*, doctoral thesis, 2025](https://www.diva-portal.org/smash/get/diva2%3A1923171/FULLTEXT01.pdf): §3.1 describes repeated pile-to-receiver short cycles; §3.2 uses front/rear frames, articulated steering, four wheels, parallel Z-bar, driveline/differentials and compliant tires. Our current primitive geometry and rotary drives omit much of that dynamics.
- [Aoshima, Fälldin, Wadbro and Servin, *World Modeling for Autonomous Wheel Loaders*, 2024](https://doi.org/10.3390/automation5030016): learns loading outcomes and pile changes from AGX Dynamics data. Its 95%/97% average prediction figures measure learned models against simulated datasets, not machine fidelity. The terrain is dry gravel; do not reuse those material values as sand or coal.
- [Aoshima and Servin, *Examining the simulation-to-reality gap of a wheel loader digging in deformable terrain*](https://doi.org/10.1007/s11044-024-10005-5): published online 2024; journal volume 64 (2025), pp. 121–148. §4.1 describes WA320-7, 15,175 kg, 3.03 m wheelbase, 2.05 m track, 1.39 m wheel diameter, 3 m³ bucket and 2.685 m bucket width, with two lift cylinders and one tilt cylinder.
- [Komatsu Japanese WA320-7 brochure](https://kcsj.komatsu/-/media/kcsj/contact/download/pdf/construction_machine/wa320-7.pdf?hash=407C4B433729A39F32EFA45E16ADBE5C&rev=b7bbb016972c43218296faf183be8782): dimensional/steering reference. Regional bucket/weight variants differ; do not mix them silently.

## Terrain and numerical resolution

The sim-to-real paper compares full non-smooth DEM (D) with a reduced multiscale terrain model (G), coupling vehicle dynamics to an active aggregate and local particle flow while resting terrain is represented separately. This is not equivalent to a fixed rigid obstacle, a generic heightmap or freely rolling coarse spheres.

Table 3 uses characteristic element/grid sizes of 50, 100, 200 and 400 mm. D50 contains about 450,000 particles at 2 ms; D400 about 800 at 20 ms. G50–G400 use fewer active particles and 5–50 ms timesteps. These are simulation resolution choices, not measurements of natural sand-grain size. Thesis §3.2.2 also explains coarse particles, contact friction and rolling resistance as approximations requiring calibration.

## Measured errors and limits

For the paper's gravel loading tests, §6.6 reports approximately 10% overall mean sim-to-real error with 3% standard deviation. §6.5 reports mean loaded-mass errors of 15% (D) and 12% (G), and work errors of 17% and 21%. §6.3 uses dynamic-time-warping trajectory error; scalar signals use mean absolute error. These metrics are not interchangeable, and an overall average is not a bound on every signal or cycle.

The study is specific to homogeneous gravel and simplified driveline/hydraulics. Tire/ground assumptions, breakout transients and shared engine power remain limitations. Neither paper supplies a complete redistributable WA320 CAD model or all the pivot, inertia and actuator data needed here. Our YAML labels those missing values as estimates. Validate independent held-out motion/contact/material tests before drawing sim-to-real conclusions.
