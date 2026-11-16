# Komodo reference archive

> Komodo is retained as a research/reference implementation only. It is not part of the active wheel-loader simulation architecture.

- Source: [osheraz/komodo](https://github.com/osheraz/komodo), audited revision `d53759100ced7439dd501620f955f347087e4f63`.
- License: upstream root MIT; package metadata contains `TODO`, and independent mesh/CAD provenance remains unresolved. Original meshes and converted diagnostic assets are not redistributed here.
- Simulator: legacy ROS/Gazebo, Python 2 and TensorFlow 1.x. None are active-project dependencies.
- Vehicle: small four-wheel skid-steer base, lift hinge and bucket hinge; no central chassis articulation or Z-bar.
- Algorithms: DDPG and A2C implementations; the checked-in learning launcher selects A2C. This is not a PPO baseline.
- Observations (`komodo_env.py`): nominal 12 entries—particle-derived payload estimate, four tip/bucket X/Z values, chassis X, three joint/control states and three state differences. `komodo_env_new.py` differs; do not assume interchangeable semantics.
- Actions: three continuous channels for forward/reverse drive, arm and bucket commands; no articulated steering channel.
- Rewards: several variants. The default environment calls `reward_V2`: approach/target-distance shaping, arm/height shaping when empty, a particle bonus near the target and failure penalties. Reward is clipped to ±20 in the step function. This is not a physical productivity measurement.
- Useful: episode/reset concepts, bounded loading actions, scripted comparison ideas and the model audit tools below.
- Limits: missing attachment meshes; invalid upstream wheel inertias; diagnostic primitive substitutions; 2D particle-in-bucket approximation; unresolved mesh provenance; joint-stop and ground-turning checks failed in the prior audit.

The archive contains this repository's inspection/conversion/validation work and source mappings. It does not copy or migrate the upstream ROS/RL stack. See [audit](docs/KOMODO_EVALUATION.md), [original-source record](original/README.md) and [RL reference](rl_reference/README.md).

Optional tools, from the repository root:

```bash
pip install -e extras/komodo
python extras/komodo/scripts/inspect_komodo.py --source /path/to/komodo
PYTHONPATH=extras/komodo/src pytest extras/komodo/tests -q
```

Conversion and runtime tools additionally require their previously documented isolated Isaac environments. Active WA320 runs never import `komodo_reference`.
