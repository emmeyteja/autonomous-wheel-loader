"""Batched normalized loading actions to bounded physical velocity targets.

Columns are longitudinal speed [m/s], lift rate [rad/s], curl rate [rad/s].
This contract excludes steering. It is a loading-skill contract, not a vehicle
interface, ROS Control message, or unconstrained actuator-effort command.
"""
from typing import TYPE_CHECKING
from wheel_loader_rl.config import ActionLimits

if TYPE_CHECKING:
    import torch


def scale_loading_actions(actions: 'torch.Tensor', limits: ActionLimits) -> 'torch.Tensor':
    """Clamp finite (N, 3) floating tensors, preserving dtype and device.

    Finite validation is deliberately enabled at this boundary for M0. It may
    synchronize a GPU; profile before moving checks out of a future hot path.
    """
    import torch
    if not isinstance(actions, torch.Tensor):
        raise TypeError('actions must be a torch.Tensor')
    if actions.ndim != 2 or actions.shape[1] != 3 or actions.shape[0] < 1:
        raise ValueError('actions must have shape (num_envs, 3) with num_envs >= 1')
    if not actions.is_floating_point():
        raise TypeError('actions must have floating dtype')
    if not torch.isfinite(actions).all():
        raise ValueError('actions must be finite')
    return actions.clamp(-1.0, 1.0) * actions.new_tensor(limits.scales)
