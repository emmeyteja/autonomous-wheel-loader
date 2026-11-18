# Development guidance

Read `docs/ARCHITECTURE.md` before changing boundaries or dependencies. `docs/architecture/ACTIVE_SYSTEM.md` is the canonical direction: original WA320-like articulated loader, rigid terrain before granular, scripted scoop before PPO, stable PPO before world models. `extras/komodo` and `extras/legacy_loading` are optional references; active code must not import them. Treat estimates as estimates. Report incomplete physics gates accurately; conversion or algebraic closure is not physical validation.

Track work and defects in GitHub issues assigned to the appropriate milestone. Keep durable design and reproduction instructions in documentation. Publish only project-relevant, sanitized technical information.

# Architecture requirements

- Follow `docs/ARCHITECTURE.md` for module boundaries and interface decisions.
- Reuse public Autoware and standard ROS 2 interfaces where units and semantics match; verify and pin the upstream version before implementation.
- Keep domain contracts, task logic, controllers, simulation, ROS adapters, training, and evaluation separate.
- Keep ROS message imports and middleware dependencies inside the optional ROS adapter packages.
- Keep Isaac Lab imports and simulator details inside the simulator integration; pure logic tests must run without Isaac Sim or ROS.
- Preserve batched tensor execution on the training path; do not route each environment step through ROS.
- Define narrow typed contracts with documented SI units, frames, timestamps, dimensions, and ownership.
- Never treat an articulated chassis angle as an Ackermann tire angle without an explicit validated model.
- Use loader-specific contracts for lift/curl, payload, and pile state where upstream semantics do not fit.
- Select implementations through configuration at application entry points; avoid global state and circular imports.
- Add abstractions at actual integration boundaries; avoid speculative services and deep class hierarchies.
- Test contract conversions, limits, reset behavior, and batch isolation when implementing each boundary.
- Keep this repository simulation-only; no real-machine actuation interface is authorized.

- The first target material is dry sand. Do not describe the coarse-sphere demo or gravel-paper parameters as calibrated sand; validate bulk behavior and angle of repose before relying on excavation results.

# Public repository requirements

- This repository and its GitHub issues are public.
- Never commit private emails, real names not required for attribution, personal paths, hostnames, private network addresses, credentials, or private third-party material.
- Use the account's GitHub noreply email for commit author and committer identity. Do not change global Git identity.
- Keep workstation inventory, raw diagnostics, unredacted logs, and local approval/chat notes local under ignored `outputs/` or `.local/` directories.
- Use GitHub milestones and issues for work items, progress, defects, and acceptance checklists. Issues need the same privacy review as source commits.
- Share minimal technical reproductions and relevant software versions; review attachments and screenshots before publishing.
- Keep documentation focused on stable architecture, setup, configuration semantics, and research methods.
- Preserve dependency versions and technical reproducibility without publishing personal machine identity.
- Review staged files and author/committer identity before pushing. Check history when removing previously published information.
- When an authorized history correction is necessary, make a local recovery backup, verify the remote head, and use an explicit force-with-lease; never overwrite others' new commits.

- Keep the README short and use bullet points. Omit sections on first objectives, development roadmaps, research scope, and public-information policy; use issues for planning and tracking.

- At the end of each simulation development step, include a playable video of the current recorded run. Label replay versus live simulation and state any failed acceptance checks. Keep video files under ignored outputs; do not imply that appearance proves physical accuracy.
