import pytest
import torch
from wheel_loader_rl.config import ActionLimits
from wheel_loader_rl.env.actions import scale_loading_actions


@pytest.mark.parametrize('device', ['cpu', pytest.param('cuda', marks=pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA unavailable'))])
def test_batch_commands_are_bounded_independent_and_preserve_device(device):
    limits = ActionLimits()
    commands = torch.tensor([[-2., 0., 2.], [1., -1., 0.5]], dtype=torch.float64, device=device)
    original = commands.clone()
    actual = scale_loading_actions(commands, limits)
    expected = commands.new_tensor([[-1., 0., 0.6], [1., -0.4, 0.3]])
    torch.testing.assert_close(actual, expected)
    torch.testing.assert_close(commands, original)
    assert actual.device == commands.device and actual.dtype == commands.dtype
    isolated = scale_loading_actions(commands[1:2], limits)
    torch.testing.assert_close(actual[1:2], isolated)


@pytest.mark.parametrize('value', [-1., 0., 1.])
def test_endpoint_commands(value):
    actual = scale_loading_actions(torch.full((16, 3), value), ActionLimits())
    expected = torch.tensor(ActionLimits().scales).expand(16, -1) * value
    torch.testing.assert_close(actual, expected)


@pytest.mark.parametrize('shape', [(3,), (2, 4), (0, 3), (2, 3, 1)])
def test_invalid_batch_shape(shape):
    with pytest.raises(ValueError):
        scale_loading_actions(torch.zeros(shape), ActionLimits())


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_invalid_actions_do_not_become_valid_commands(value):
    with pytest.raises(ValueError):
        scale_loading_actions(torch.tensor([[0., value, 0.]]), ActionLimits())


def test_integer_commands_rejected():
    with pytest.raises(TypeError):
        scale_loading_actions(torch.zeros((1, 3), dtype=torch.int64), ActionLimits())
