# Scooping research reference

- [Learning launcher](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/komodo2_rl/src/komodo_learn.py): DDPG/A2C selection, rollout and checkpoint ideas.
- [Environment](https://github.com/osheraz/komodo/blob/d53759100ced7439dd501620f955f347087e4f63/komodo2_rl/src/environments/komodo_env.py): observations, three action channels, reset and reward variants.
- Consult the code to distinguish paper results from launcher defaults. Existing callbacks, payload estimates and rewards are not validated for WA320.
- The active repository must establish rigid-body dynamics, terrain and a scripted scooping baseline before adding PPO. No Python 2, ROS Kinetic or TensorFlow 1.x port is intended.
