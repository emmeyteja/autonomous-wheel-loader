from dataclasses import replace
import math
from pathlib import Path
import subprocess
import sys
import pytest
from wheel_loader_rl.config import ActionLimits, EnvironmentConfig, PileConfig, load_config


def test_checked_in_config_has_coherent_timing():
    cfg = load_config(Path(__file__).parents[2] / 'configs/env.toml')
    assert cfg.decimation * cfg.policy_hz == cfg.physics_hz
    assert cfg.episode_steps == 100


@pytest.mark.parametrize('updates', [
    {'num_envs': 0}, {'num_envs': True}, {'num_envs': 1.5},
    {'seed': -1}, {'seed': 2**32}, {'seed': 4.5},
    {'physics_hz': 0}, {'policy_hz': 0}, {'physics_hz': 200, 'policy_hz': 30},
    {'episode_length_s': math.nan}, {'episode_length_s': -1}, {'episode_length_s': 0.123},
])
def test_invalid_runtime_configuration_fails_before_launch(updates):
    with pytest.raises(ValueError):
        EnvironmentConfig(**updates)


@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf, True])
def test_invalid_action_limits_rejected(value):
    with pytest.raises(ValueError):
        ActionLimits(lift_radps=value)


def test_unknown_config_key_rejected(tmp_path):
    path = tmp_path / 'invalid.toml'
    path.write_text('num_env = 16\n')
    with pytest.raises(TypeError):
        load_config(path)


def test_overlapping_particle_configuration_rejected():
    with pytest.raises(ValueError):
        replace(PileConfig(), spacing_m=0.1)


def test_pure_module_imports_do_not_load_simulator_or_ros():
    code = """
import sys
import wheel_loader_rl.config, wheel_loader_rl.pile, wheel_loader_rl.env.actions
assert not any(name.split('.')[0] in {'isaaclab', 'isaacsim', 'rclpy', 'torch'} for name in sys.modules)
"""
    subprocess.run([sys.executable, '-c', code], check=True)


@pytest.mark.parametrize('updates', [
    {'static_friction': -1}, {'dynamic_friction': math.nan},
    {'rolling_friction': math.inf}, {'rolling_friction': -0.1},
    {'static_friction': 0.2, 'dynamic_friction': 0.3},
])
def test_invalid_material_contact_parameters_rejected(updates):
    with pytest.raises(ValueError):
        replace(PileConfig(), **updates)


@pytest.mark.parametrize('value', [1, 4, 7, True])
def test_unsupported_pile_contact_dimension_rejected(value):
    with pytest.raises(ValueError):
        replace(PileConfig(), contact_dimensions=value)
