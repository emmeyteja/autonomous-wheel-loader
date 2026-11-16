# Kinematic acceptance contract

- M0 now means the WA320-like kinematic model, as defined in [active architecture](architecture/ACTIVE_SYSTEM.md).
- Require rear/front frames, central steering, four rotating wheels, boom, bucket and an explicit closed Z-bar mechanism.
- Validate configured joint ranges, assembly branch, pin coincidence and geometry. Record estimated dimensions and collision limitations.
- Passing algebraic loop closure or converting USD alone does not validate dynamics, contact or self-collision.
- First integrated foundation additionally requires M1/M2 physics and rigid-terrain checks. Training remains gated until M4.
- Historical simulator/material setup checks remain archived; they do not establish completion of these new gates.
