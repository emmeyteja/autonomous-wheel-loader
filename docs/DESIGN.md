# Autonomous Wheel Loader — Isaac Lab RL Research Prototype
## Agent-Execution Design Document

**Purpose:** Give an autonomous coding agent a concrete specification to build, train, test, and report a first wheel-loader bucket-filling RL prototype in NVIDIA Isaac Lab.

**Primary research direction:** PPO policy learning for wheel-loader excavation, with a later learned world model and attack-pose planner.

**Important:** This is a research prototype, not a production autonomous-machine controller. Do not claim sim-to-real validity.

---

# 1. Goal

Build a reproducible Isaac Lab environment in which a simulated wheel loader learns a simplified bucket-filling maneuver using PPO.

The first milestone is intentionally narrow:

> Starting from a predefined approach pose in front of a granular pile, learn a control policy that drives the loader into the pile and fills the bucket while avoiding excessive force, slip, bad trajectories, and unnecessary time.

Do NOT implement the full autonomous work cycle yet.

Do NOT implement VLA, camera perception, Hybrid A*, or a world model in the first milestone.

The architecture should be designed so those can be added later without rewriting the environment.

---

# 2. Target Architecture

```text
                 CURRENT MILESTONE

          Isaac Lab / Isaac Sim
                  │
                  ▼
        Wheel-loader RL Environment
                  │
        ┌─────────┴─────────┐
        │                   │
   Observation           Action
        │                   │
        ▼                   ▼
   Loader state          PPO policy
   Bucket state             │
   Pile state               │
   Interaction state        │
        │                   │
        └─────────┬─────────┘
                  ▼
             Simulation
                  │
                  ▼
              Reward
                  │
                  └──────► PPO


                 FUTURE

Pile perception
      │
      ▼
Candidate attack poses
      │
      ▼
World-model evaluation
      │
      ▼
Attack-pose planner
      │
      ▼
Hybrid A*
      │
      ▼
PPO loading policy
      │
      ▼
Loader
      │
      ▼
Updated pile
      └──────────────► replan
```

Isaac Lab currently supports PPO through RSL-RL and other libraries; RSL-RL is the preferred starting point for GPU-parallel robotics RL according to the current Isaac Lab documentation.

---

# 3. Repository Structure

Create a clean standalone repository:

```text
wheel_loader_rl/
├── README.md
├── LICENSE
├── pyproject.toml
├── .gitignore
├── configs/
│   ├── env.yaml
│   ├── train.yaml
│   └── evaluation.yaml
├── src/
│   └── wheel_loader_rl/
│       ├── __init__.py
│       ├── env/
│       │   ├── __init__.py
│       │   ├── wheel_loader_env.py
│       │   ├── wheel_loader_env_cfg.py
│       │   ├── observations.py
│       │   ├── actions.py
│       │   ├── rewards.py
│       │   ├── terminations.py
│       │   └── randomization.py
│       ├── assets/
│       │   └── README.md
│       ├── policies/
│       │   └── README.md
│       ├── evaluation/
│       │   ├── evaluate_policy.py
│       │   ├── metrics.py
│       │   └── plots.py
│       └── utils/
│           ├── logging.py
│           └── reproducibility.py
├── scripts/
│   ├── train.sh
│   ├── evaluate.sh
│   ├── smoke_test.sh
│   └── benchmark.sh
├── tests/
│   ├── test_env.py
│   ├── test_observations.py
│   ├── test_rewards.py
│   └── test_determinism.py
├── docs/
│   ├── DESIGN.md
│   └── RESULTS.md
└── outputs/
    └── .gitkeep
```

Use the standard Isaac Lab project/task registration mechanism. Do not fork or modify Isaac Lab core code.

---

# 4. Simulation Task

## 4.1 Episode

One episode represents one bucket-loading attempt.

Initial state:

```text
loader
   ↓
   ↓ approach direction

████████████████
      PILE
████████████████
```

The loader starts at a controlled distance from the pile with a fixed initial orientation.

The bucket starts empty.

The episode terminates when one of these occurs:

1. Bucket reaches target fill/load.
2. Loader exits the valid loading region.
3. Excessive collision/force occurs.
4. Loader becomes stuck for a timeout.
5. Maximum episode duration is reached.
6. Numerical instability occurs.

---

# 5. Observation Space

Use STATE-BASED observations first.

Do not use RGB cameras in Milestone 1.

Suggested normalized observation vector:

```text
vehicle:
    longitudinal velocity
    lateral velocity
    yaw rate
    steering state
    vehicle pose relative to pile

loader linkage:
    lift-arm joint angle
    lift-arm joint velocity
    bucket joint angle
    bucket joint velocity

bucket:
    bucket position relative to pile
    bucket orientation
    bucket velocity

interaction:
    estimated bucket load
    contact force
    longitudinal force
    vertical force
    wheel slip estimate

pile:
    local height/geometry representation
    distance from bucket cutting edge to pile surface
    local pile slope

task:
    remaining target distance
    episode progress
```

Do not expose privileged quantities in the final deployment-style observation group unless explicitly marked as `privileged_obs`.

Maintain two observation groups:

```text
policy_obs
critic_obs / privileged_obs
```

If the selected PPO implementation does not support asymmetric observations cleanly, keep the first experiment fully observable and document the limitation.

---

# 6. Action Space

Use continuous normalized actions in [-1, 1].

First version:

```text
a[0] = vehicle longitudinal command
a[1] = lift-arm velocity command
a[2] = bucket-curl velocity command
```

The environment converts normalized actions into physically bounded actuator commands.

Example:

```text
[-1, 1]
   │
   ▼
scale + clamp
   │
   ▼
physical actuator command
```

Do NOT directly expose unbounded torques.

Prefer actuator-level velocity/position targets with realistic limits.

All action limits must be defined in one configuration file.

---

# 7. Control Frequency

Use separate simulation and policy frequencies.

Example starting point:

```text
Physics:          100–200 Hz
Actuator update:   50 Hz
RL policy:         10–20 Hz
```

Do not assume these values are optimal.

Make them configurable.

The agent must run a small frequency sweep after the basic environment works:

```text
policy_hz = 10
policy_hz = 20
policy_hz = 50
```

Compare stability and training speed.

---

# 8. Reward Function

The reward must explicitly reflect the engineering objective.

Start with:

```text
R =
    + w_mass      * Δbucket_mass
    - w_time      * Δt
    - w_energy    * estimated_work
    - w_slip      * wheel_slip
    - w_force     * excessive_contact_force
    - w_collision * collision_penalty
    - w_bad       * bad_state_penalty
```

Do NOT initially over-engineer the reward.

Recommended first experiment:

```text
positive:
    increase in bucket load

negative:
    time
    excessive contact force
    excessive wheel slip
    collision
    getting stuck
```

The reward implementation must expose each component separately in logging.

Example:

```text
reward_total
reward_mass
reward_time
reward_force
reward_slip
reward_collision
```

This is essential for debugging reward hacking.

---

# 9. Reward Hacking Tests

The agent must explicitly test for these behaviors:

### Failure mode A — ramming

Loader accelerates aggressively into pile.

Expected:
- high force penalty
- lower final reward

### Failure mode B — spinning wheels

Loader spins wheels without increasing bucket load.

Expected:
- slip penalty
- no meaningful mass reward

### Failure mode C — sitting in pile

Loader stops moving but receives small incremental reward.

Expected:
- time penalty prevents this

### Failure mode D — lifting without loading

Loader manipulates bucket without meaningful material acquisition.

Expected:
- no significant mass reward

### Failure mode E — exploiting simulator

Policy discovers physically unrealistic behavior.

Expected:
- terminate or penalize invalid states.

---

# 10. Domain Randomization

Do NOT start with aggressive randomization.

Phase A:

```text
fixed loader
fixed pile
fixed friction
fixed actuator response
```

First prove learning.

Phase B:

Randomize:

```text
pile height
pile slope
soil density
friction coefficient
material mass
loader initial position
loader initial speed
actuator delay
actuator strength
sensor noise
```

Phase C:

Randomize combinations and distributions.

Keep evaluation environments deterministic and separate from training environments.

---

# 11. Pile Representation

For Milestone 1, use a procedurally generated pile.

Start with a simple pile:

```text
                 /\
                /  \
_______________/    \_______________
```

Then support:

- Gaussian/random mound
- asymmetric mound
- multiple local peaks
- variable slope
- randomized material properties

Represent the pile internally using a heightfield or equivalent terrain representation where practical.

The observation should contain a local representation rather than the entire global environment.

Future world-model work will use:

```text
heightmap_t + action_t
        │
        ▼
world model
        │
        ▼
heightmap_{t+1}
```

Do not implement the learned world model in Milestone 1.

---

# 12. PPO Configuration

Use Isaac Lab + RSL-RL PPO as the primary implementation unless the installed Isaac Lab version makes this impractical.

Keep all hyperparameters configurable.

Initial baseline:

```text
algorithm: PPO

policy:
    MLP
    2–3 hidden layers
    256–512 units/layer
    activation: ELU or equivalent

rollout:
    configurable number of steps

learning_rate:
    configurable

gamma:
    ~0.99

GAE lambda:
    ~0.95

clip range:
    ~0.2

entropy coefficient:
    small positive value initially

value loss coefficient:
    configurable

gradient clipping:
    enabled
```

These are starting points, not scientifically validated final values.

The agent must report the actual configuration used.

Do not silently tune hyperparameters until the result looks good.

---

# 13. Baselines

The RL policy must be compared against at least two baselines.

## Baseline 1 — Random policy

Useful only as a sanity check.

## Baseline 2 — Hand-designed controller

Implement a simple scripted loading controller:

```text
approach
→ penetrate
→ curl
→ lift
→ reverse
→ stop
```

This is important.

A trained RL policy that beats random but loses to a simple heuristic is not a useful result.

## Optional Baseline 3

A fixed parameterized loading controller with manually optimized parameters.

---

# 14. Evaluation Metrics

Every evaluation run must report:

```text
success rate
mean bucket mass
median bucket mass
loading time
energy/work estimate
maximum contact force
mean contact force
wheel slip
distance traveled
number of failures
```

Report:

```text
mean
standard deviation
median
P10
P90
```

Use at least 100 evaluation episodes for final comparison if computationally feasible.

Never evaluate only the best episode.

---

# 15. Generalization Tests

Train on one distribution.

Evaluate on unseen distributions.

Example:

```text
TRAIN
pile slope:     20–30°
friction:       fixed range
density:        fixed range

TEST
pile slope:     15–35°
friction:       wider range
density:        wider range
initial pose:   unseen
```

The evaluation script must clearly distinguish:

```text
in-distribution
out-of-distribution
```

This is more scientifically meaningful than simply reporting training reward.

---

# 16. Required Tests

Before training:

### Test 1 — Environment import

```bash
python -m pytest tests/test_env.py
```

Environment must instantiate without Isaac Sim GUI.

### Test 2 — Zero/random action smoke test

Run a short episode with zero actions and random actions.

Verify:

- no crash
- no NaN
- no Inf
- observations have expected dimensions
- actions are within limits
- rewards are finite

### Test 3 — Reset determinism

Same seed + same configuration should produce the same initial state.

### Test 4 — Action bounds

Send:

```text
-1
0
+1
```

and verify actuator commands remain inside configured physical limits.

### Test 5 — Reward decomposition

Verify total reward approximately equals the sum of configured reward terms.

### Test 6 — Batch environments

Run:

```text
num_envs = 1
num_envs = 16
num_envs = 64
```

and report:

```text
steps/sec
GPU memory
simulation throughput
```

Do not assume 64 is optimal; find a stable maximum for the available GPU.

---

# 17. Training Tests

Run a deliberately small training experiment first.

Example:

```text
num_envs = 64
short training = 10k–50k steps
```

Goal:

- verify learning curve exists
- verify no NaNs
- verify checkpoint creation
- verify evaluation works

Then run the real experiment.

Training must support:

```bash
./scripts/train.sh
```

with optional overrides:

```bash
./scripts/train.sh --num-envs 1024
./scripts/train.sh --seed 42
./scripts/train.sh --max-iterations N
```

Use the exact command syntax supported by the installed Isaac Lab version.

---

# 18. Checkpointing

Every experiment must save:

```text
policy checkpoint
environment configuration
PPO configuration
random seed
git commit hash
Isaac Lab version
Python version
GPU information
training command
evaluation results
```

Never create an experiment whose configuration cannot be reconstructed.

Directory:

```text
outputs/
└── YYYYMMDD_HHMMSS_<experiment_name>/
    ├── checkpoint/
    ├── config/
    ├── logs/
    ├── videos/
    ├── metrics.json
    └── README.md
```

---

# 19. Reproducibility

Every experiment must accept:

```bash
--seed
```

Set seeds for:

- Python
- NumPy
- PyTorch
- environment randomization
- procedural pile generation

Document any remaining nondeterminism caused by GPU physics/simulation.

Run at least 3 seeds for a meaningful comparison.

Do not report one lucky seed as the result.

---

# 20. Visualization

Evaluation must produce videos showing:

1. loader approaching pile
2. bucket penetration
3. bucket filling
4. bucket extraction
5. final bucket state

Overlay optional debug information:

```text
episode
reward
bucket mass
velocity
force
slip
```

Also generate plots:

```text
episode reward
success rate
bucket mass
loading time
force
slip
```

---

# 21. Coding Best Practices

## 21.1 Separation of concerns

Never put everything into one environment file.

Separate:

```text
observations
actions
rewards
termination
randomization
metrics
configuration
```

The RL algorithm must not contain task-specific physics logic.

---

## 21.2 Configuration over hardcoding

Bad:

```python
max_speed = 2.5
```

inside arbitrary code.

Good:

```yaml
vehicle:
  max_speed_mps: 2.5
```

and load it through the environment configuration.

---

## 21.3 Units

Use SI units internally:

```text
meters
seconds
kilograms
newtons
radians
m/s
N·m
joules
```

Document units in variable names where ambiguity exists.

Examples:

```python
vehicle_speed_mps
bucket_mass_kg
contact_force_n
```

Avoid:

```python
speed
force
mass
```

when the unit is not obvious.

---

## 21.4 Tensor-first implementation

For RL training:

- avoid Python loops over environments
- use batched PyTorch tensors
- keep data on GPU where possible
- vectorize reward calculations
- vectorize observation construction

A loop over 4096 environments in Python is unacceptable unless profiling proves it irrelevant.

---

## 21.5 No magic numbers

Bad:

```python
if force > 120000:
```

Good:

```python
if force > cfg.termination.max_contact_force_n:
```

---

## 21.6 Assertions

Use assertions for configuration invariants:

```python
assert action_dim == 3
assert observation_dim > 0
assert max_speed > min_speed
```

Do not use assertions as runtime safety mechanisms for the physical controller.

---

## 21.7 Logging

Use structured logging.

Every training run should expose:

```text
episode_return
success_rate
bucket_mass
loading_time
contact_force
slip
```

Do not log thousands of individual tensor values every step.

---

## 21.8 Testing

Unit-test pure functions separately from Isaac Sim.

Examples:

```text
reward calculation
normalization
action scaling
observation construction
pile generation
termination conditions
metrics
```

Use integration tests for Isaac Lab interaction.

---

## 21.9 Git discipline

Small commits.

Recommended sequence:

```text
feat: create Isaac Lab project skeleton
feat: add wheel loader asset
feat: add pile terrain
feat: add observation space
feat: add action interface
feat: add reward terms
test: add environment smoke tests
feat: add PPO training configuration
feat: add evaluation pipeline
```

Never commit:

```text
checkpoints
large simulation assets
training logs
videos
datasets
```

Use Git LFS only when there is a clear reason and document it.

---

# 22. Safety of the Research Software

This is simulation-only research.

Do not create code that can directly command a real construction machine.

Keep the real-machine interface separate from the research environment.

The RL policy output should terminate at:

```text
simulation actuator interface
```

Future real-machine integration must have an independent safety layer.

---

# 23. Milestone Plan

## M0 — Infrastructure

Deliver:

- repository
- Isaac Lab integration
- task registration
- loader asset placeholder
- procedural pile
- smoke test

PASS condition:

```text
environment launches
random policy runs 100 episodes
no NaN/Inf
```

---

## M1 — Basic RL

Implement:

- observation
- action
- reward
- PPO
- checkpointing
- evaluation

PASS condition:

> PPO demonstrably improves over random policy.

---

## M2 — Useful Loading

Implement:

- better pile interaction
- realistic actuator limits
- force/slip penalties
- scripted baseline

PASS condition:

> PPO beats scripted baseline on at least one meaningful productivity metric while maintaining acceptable failure rate.

---

## M3 — Generalization

Add domain randomization.

PASS condition:

> Policy retains useful performance on unseen pile conditions.

---

## M4 — World Model

Collect trajectories:

```text
s_t
a_t
s_{t+1}
reward
pile_t
pile_{t+1}
```

Train:

```text
f_theta(s_t, a_t) → s_{t+1}
```

and eventually:

```text
f_theta(H_t, s_t, a_t)
    →
H_{t+1}, s_{t+1}, performance
```

Compare predicted vs actual simulator outcomes.

---

## M5 — Attack Pose Planning

Add:

```text
pile heightmap
    ↓
candidate attack poses
    ↓
world model
    ↓
predicted loading outcome
    ↓
pose selection
    ↓
Hybrid A*
    ↓
PPO loading skill
```

This is where the larger research contribution begins.

---

# 24. Research Experiment Matrix

The agent should eventually produce this table:

| Experiment | Controller | World Model | Randomization | Result |
|---|---|---|---|---|
| E0 | Random | No | No | baseline |
| E1 | Scripted | No | No | baseline |
| E2 | PPO | No | No | baseline RL |
| E3 | PPO | No | Yes | robustness |
| E4 | PPO + WM | Yes | Yes | model-based RL |
| E5 | WM planner + PPO | Yes | Yes | sequential planning |

Do not skip E0/E1.

---

# 25. World Model Design — Future

Do NOT implement until M3 is stable.

Initial model:

```text
input:
    state_t
    action_t
    local pile heightmap_t

        ↓

CNN / encoder
        +
MLP dynamics model

        ↓

predicted:
    state_{t+1}
    pile representation_{t+1}
    bucket mass
    loading time
    work
```

Aoshima's work provides a useful reference architecture: a learned model predicts loading performance and resulting pile state from local pile geometry and loading actions.

For temporal dependence, investigate an LSTM world model as used in recent wheel-loader research.

---

# 26. Attack Pose Planning — Future

Candidate pose:

```text
g = [x, y, yaw]
```

For each candidate:

```text
H_t + g + loading parameters
        ↓
world model
        ↓
M, T, E, H_{t+1}
```

Select:

```text
g* = argmax objective(g)
```

Eventually use multi-step planning:

```text
g1 → predicted H1
       ↓
g2 → predicted H2
       ↓
g3 → predicted H3
```

This is more interesting than simply selecting the highest immediate bucket-fill pose.

---

# 27. Agent Reporting Requirement

At the end of every implementation phase, the coding agent MUST report:

## Environment

```text
Isaac Lab version:
Isaac Sim version:
Python:
PyTorch:
GPU:
OS:
```

## Implementation

List:

```text
files created
files modified
major architectural decisions
```

## Tests

For every test:

```text
test name
command
PASS/FAIL
result
```

## Training

```text
seed
num_envs
iterations
wall-clock time
steps/sec
final reward
success rate
```

## Evaluation

Provide a table:

| Metric | Random | Scripted | PPO |
|---|---:|---:|---:|
| Success | | | |
| Bucket mass | | | |
| Loading time | | | |
| Force | | | |
| Slip | | | |

## Problems

Explicitly report:

```text
known limitations
unstable behavior
failed experiments
assumptions
```

Do NOT hide failed experiments.

## Next recommendation

Give exactly:

```text
1. highest-priority issue
2. next experiment
3. expected reason
```

---

# 28. Definition of Done for First Prototype

The first prototype is complete only when:

- [ ] Isaac Lab environment launches reproducibly.
- [ ] Wheel-loader asset is simulated.
- [ ] Procedural pile exists.
- [ ] State observations work.
- [ ] Continuous actions work.
- [ ] Reward components are separately logged.
- [ ] Random policy runs.
- [ ] Scripted baseline runs.
- [ ] PPO trains.
- [ ] PPO checkpoint can be loaded.
- [ ] Evaluation runs independently from training.
- [ ] At least 100 evaluation episodes can run.
- [ ] PPO results are compared against both baselines.
- [ ] No NaN/Inf occurs.
- [ ] Results are saved with configuration and git commit.
- [ ] Videos/plots are generated.
- [ ] README explains how to reproduce the experiment.
- [ ] Agent provides a final test report.

---

# 29. Important Research Discipline

Do not optimize for:

> "make the reward curve look good."

Optimize for:

> "demonstrate a reproducible physical behavior that generalizes to unseen conditions."

Do not change the reward, environment, or evaluation distribution after seeing the results without recording the change.

Every experimental change must have:

```text
hypothesis
change
expected effect
result
conclusion
```

This repository should eventually be usable as the experimental foundation for a research paper.

---

# 30. References

Use the current Isaac Lab documentation for environment/task structure and PPO workflows.

Relevant references:

- Isaac Lab documentation: https://isaac-sim.github.io/IsaacLab/
- Isaac Lab reinforcement learning: https://isaac-sim.github.io/IsaacLab/develop/source/concepts/reinforcement_learning.html
- Isaac Lab Direct/Manager MDP: https://isaac-sim.github.io/IsaacLab/main/source/concepts/task_workflows.html
- Isaac Lab RL agent configuration: https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/training_guide.html

Research references to reproduce/compare against:

- Aoshima et al., 2024, "World Modeling for Autonomous Wheel Loaders", Automation 5(3), 259–281.
- Morais et al., 2025, "Reinforcement Learning with World Models for Autonomous Excavation Optimization in Wheel Loaders", IFAC 2025.
- Eriksson et al., 2024, "Optimizing a Bucket Filling Strategy for Wheel Loaders Inside a Dream Environment."

---

# 31. Final Instruction to Coding Agent

Implement the project incrementally.

Do not attempt to build the entire architecture in one pass.

Start with M0.

After M0:
1. run the smoke tests,
2. report exact results,
3. fix failures,
4. proceed to M1 only after M0 passes.

At every milestone, preserve reproducibility and report both successes and failures.

The first objective is NOT state-of-the-art performance.

The first objective is a **clean, reproducible, testable wheel-loader RL environment on which serious research can be built.**
