# Active wheel-loader system

- Active package: `src/wheel_loader`; original parametric WA320-7-like articulated vehicle.
- Archive: `extras/komodo` and `extras/legacy_loading`; neither is required by active installation, model generation, validation or the training entry point.
- Vehicle geometry and parameters live in `vehicle/`; SI commands and wheel references in `control/`; terrain geometry/selection in `terrain/`.
- Simulator-specific conversion, scene setup, stepping and state extraction live in `environment/simulation/`. No Isaac/ROS imports in domain logic.
- `evaluation/` owns repeatable checks. `training/` selects the vehicle and enforces prerequisites; foundation mode currently performs geometry validation, not learning.
- Diagnostic wheel/bucket contact sensors and full V-cycle motion rehearsals, with an optional immovable rigid-pile contact test, are implemented. Policy observations, rewards, material scooping tasks and learned policies remain deferred until their milestone. Do not create substitute implementations that imply these work already.
- Default: WA320 plus rigid pile. Registry exposes Komodo reference metadata only with explicit opt-in, never importing its implementation.
- Optional Autoware adapters remain outside the core. Match units, frames and semantics; an articulation angle is not an Ackermann tire angle. Lift/curl need loader-specific contracts.

## Canonical gates

- M0: kinematic frames, four wheels, central articulation, boom, bucket and closed Z-bar geometry; joint ranges and geometry validation.
- M1: identified masses, COMs, inertias, contact parameters and actuator limits. Estimated test values are provisional, not calibrated dynamics.
- M2: rigid-ground driving, steering, approach and linkage motion; rigid-pile contact and repeatable resets in the simulator.
- M3: validated granular terrain, starting with dry sand; DEM and multiscale are separate interchangeable backends.
- M4: scripted approach → penetrate → lift → curl → breakout → reverse, with material accounting.
- M5: PPO only after M4 works.
- M6: world model only after stable physics and PPO baseline.

The first integrated physical foundation combines M0–M2 in Isaac Sim/Isaac Lab. Successful USD conversion does not complete these gates. Full-cycle travel, configurable dump zones/poses and repeated attack selection remain the mission goal; see [mission scope](../PROJECT_GOALS.md). No hardware actuation.

## Contracts and scaling

- X forward, Y left, Z up; metres, seconds, kilograms, radians, newtons and newton-metres. Root at the central hinge, rigidly attached to the rear frame.
- `LoaderCommand`: rear-axle speed [m/s], front-relative-to-rear yaw [rad], boom displacement [rad], bellcrank displacement [rad]. These are diagnostic simulator commands, not a learned action space or hydraulic interface.
- Bucket and rod angles are passive outputs. Do not add independent position drives to close the loop artificially.
- Future batched task state/action tensors stay on the simulator device. ROS transport must not sit in the training loop. Isolate reset state, seeds and persistent dump-bed occupancy by environment.
- Independent CPU material-column diagnostics exist for contact-law investigation; they do not enable the production granular backend or establish Newton vehicle/material coupling.
- Reserved terrain backends fail explicitly until implemented and calibrated. Missing material values are unknown, not zero.
