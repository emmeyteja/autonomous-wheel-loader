"""Validated SI-unit configuration, independent of ROS and simulation imports."""
from dataclasses import dataclass, field
import math
from pathlib import Path
import tomllib


def _positive(name: str, value: float) -> None:
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class ActionLimits:
    """Uncalibrated initial limits; not real-machine operating parameters."""
    longitudinal_mps: float = 1.0
    lift_radps: float = 0.4
    curl_radps: float = 0.6

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _positive(name, getattr(self, name))

    @property
    def scales(self) -> tuple[float, float, float]:
        return (self.longitudinal_mps, self.lift_radps, self.curl_radps)


@dataclass(frozen=True)
class PileConfig:
    """Coarse particle geometry and material-contact configuration; not calibrated soil."""
    shape: str = 'mound'
    slope_deg: float = 30.0
    max_particles: int = 4096
    radius_m: float = 0.08
    spacing_m: float = 0.18
    half_length_m: float = 0.9
    half_width_m: float = 0.6
    height_m: float = 0.6
    particle_density_kgpm3: float = 1600.0
    contact_dimensions: int = 6
    static_friction: float = 0.65
    dynamic_friction: float = 0.5
    rolling_friction: float = 0.02
    center_x_m: float = 2.0
    jitter_m: float = 0.005

    def __post_init__(self):
        for name in ('radius_m', 'spacing_m', 'half_length_m', 'half_width_m', 'height_m', 'particle_density_kgpm3'):
            _positive(name, getattr(self, name))
        if self.shape not in ('mound', 'trapezoid', 'column'):
            raise ValueError('shape must be mound, trapezoid or column')
        if not math.isfinite(self.slope_deg) or not 0 < self.slope_deg < 90:
            raise ValueError('slope_deg must be finite and between 0 and 90 degrees')
        if type(self.max_particles) is not int or not 1 <= self.max_particles <= 16384:
            raise ValueError('max_particles must be an integer in [1, 16384]')
        if self.shape == 'column' and min(self.half_length_m, self.half_width_m) <= self.radius_m + self.jitter_m:
            raise ValueError('column footprint must accommodate radius and jitter')
        if self.shape == 'trapezoid':
            run = self.height_m / math.tan(math.radians(self.slope_deg))
            if run >= min(self.half_length_m, self.half_width_m):
                raise ValueError('trapezoid height/slope must leave a positive top width in both axes')
        if type(self.contact_dimensions) is not int or self.contact_dimensions not in (3, 6):
            raise ValueError('contact_dimensions must be 3 (sliding) or 6 (sliding and rolling)')
        for name in ('static_friction', 'dynamic_friction', 'rolling_friction'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.dynamic_friction > self.static_friction:
            raise ValueError('dynamic_friction must not exceed static_friction')
        if not math.isfinite(self.center_x_m):
            raise ValueError('center_x_m must be finite')
        if not math.isfinite(self.jitter_m) or self.jitter_m < 0:
            raise ValueError('jitter_m must be finite and nonnegative')
        if self.spacing_m < 2 * self.radius_m + 2 * self.jitter_m:
            raise ValueError('spacing must keep jittered particle centers non-overlapping')
        if min(self.half_length_m, self.half_width_m) < self.radius_m or self.height_m < 2 * self.radius_m:
            raise ValueError('pile dimensions must accommodate a sphere')

    @property
    def sphere_mass_kg(self) -> float:
        return self.particle_density_kgpm3 * (4.0 / 3.0) * math.pi * self.radius_m ** 3


@dataclass(frozen=True)
class EnvironmentConfig:
    seed: int = 42
    num_envs: int = 1
    physics_hz: int = 200
    policy_hz: int = 20
    episode_length_s: float = 5.0
    action_limits: ActionLimits = field(default_factory=ActionLimits)
    pile: PileConfig = field(default_factory=PileConfig)

    def __post_init__(self):
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError('seed must be an integer in [0, 2**32)')
        for name in ('num_envs', 'physics_hz', 'policy_hz'):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be a positive integer')
        if self.physics_hz % self.policy_hz:
            raise ValueError('physics_hz must be an integer multiple of policy_hz')
        _positive('episode_length_s', self.episode_length_s)
        steps = self.episode_length_s * self.policy_hz
        if not math.isclose(steps, round(steps), rel_tol=0, abs_tol=1e-9):
            raise ValueError('episode duration must contain an integer number of policy steps')

    @property
    def decimation(self) -> int:
        return self.physics_hz // self.policy_hz

    @property
    def episode_steps(self) -> int:
        return round(self.episode_length_s * self.policy_hz)


def load_config(path: str | Path) -> EnvironmentConfig:
    """Reject unknown keys and invalid configuration before starting a runtime."""
    with Path(path).open('rb') as stream:
        data = tomllib.load(stream)
    actions = ActionLimits(**data.pop('action_limits', {}))
    pile = PileConfig(**data.pop('pile', {}))
    return EnvironmentConfig(**data, action_limits=actions, pile=pile)
