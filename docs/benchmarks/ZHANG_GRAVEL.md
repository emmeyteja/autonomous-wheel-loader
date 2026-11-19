# Zhang gravel calibration reference

- Source: [Zhang et al. (2023)](https://doi.org/10.3390/pr11010268). Reported errors belong to that study.
- Material: 10–25 mm dry gravel, particle density 2661 kg/m³; irregular grains represented by 20–25-sphere clumps (§2.3).
- Calibration (§2.1, Table 1): 300 × 200 × 400 mm baffle box, lifting at 0.1 m/s; static target 32.534°. Drum: diameter 300 mm, length 150 mm, 40% maximum fill mass, 30 rpm; dynamic target 49.0579°.
- Independent validation (§4.3, Table 10): cylinder diameter 200 mm, height 500 mm, lifting at 0.01 m/s. Experimental mean 32.53°, simulated 31.47°; reported error 3.26%.
- Source discrepancy: §4.2 prints 32.054°; Table 1 and Equation 15 give 32.534°. Preserve this distinction when reproducing targets.
- Contact model: Hertz–Mindlin/EDEM, with separate grain/grain and grain/wall friction. Rolling coefficients cannot transfer directly to MuJoCo's length-valued convention.

## Applicability here

- Calibrate stationary and moving material separately; retain an independent validation apparatus. The drum's dynamic slope is not a stationary-pile target.
- Our rapid release, spherical grains, ordered preparation and shared wall/grain friction require changes before reproduction. Similar slopes do not establish accuracy.
- Resolve fill mass/preparation, clump coordinates and raw measurements before claiming a matched experiment. The article directs code requests to its authors.
- Useful for material-flow calibration. Its pump application does not validate wheel-loader bucket forces. Keep Aoshima for the loader; use an instrumented cutting-tool reference for force validation.
