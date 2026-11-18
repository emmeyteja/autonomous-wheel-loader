"""Every scenario exercises the entire V-cycle in a kinematic reference model."""

import math
from pathlib import Path

import numpy as np
import pytest

from wheel_loader.control.v_cycle import PHASES, VCycleController, VCycleScenario
from wheel_loader.vehicle.parameters import load_parameters

P = load_parameters(Path(__file__).resolve().parents[2] / "configs/vehicle/wa320.yaml")


@pytest.mark.parametrize("name", ["nominal", "slow", "wide"])
def test_full_cycle_bounded_commands_and_return(name):
    controller = VCycleController(P, VCycleScenario.named(name))
    xy = np.zeros(2)
    yaw, articulation = 0.0, 0.0
    dt = 0.05
    maximum_speed = 0.0
    previous = None
    for step in range(round(controller.scenario.max_duration / dt)):
        command = controller.command(step * dt, dt, xy, yaw, articulation)
        command.validate(P)
        if previous:
            assert (
                abs(command.articulation_angle - previous.articulation_angle)
                <= 0.15 * dt + 1e-8
            )
            assert abs(command.boom_angle - previous.boom_angle) <= 0.1 * dt + 1e-8
            assert abs(command.tilt_angle - previous.tilt_angle) <= 0.22 * dt + 1e-8
        previous = command
        da = (command.articulation_angle - articulation) / dt
        length = P["vehicle"]["wheelbase"] / 2
        v = command.rear_axle_speed
        omega = (v * math.sin(articulation) - length * da) / (
            length * (1 + math.cos(articulation))
        )
        xy += dt * np.array(
            [
                v * math.cos(yaw) - length * omega * math.sin(yaw),
                v * math.sin(yaw) + length * omega * math.cos(yaw),
            ]
        )
        yaw += dt * omega
        articulation = command.articulation_angle
        maximum_speed = max(maximum_speed, abs(v))
        if controller.done:
            break
    assert controller.done and not controller.timeout
    assert [event["phase"] for event in controller.events] == list(PHASES)
    assert np.linalg.norm(xy) < 0.35
    assert maximum_speed <= controller.scenario.speed
    for phase, goal in [
        (
            "forward_right",
            [controller.scenario.distance, -controller.scenario.half_width],
        ),
        ("reverse_loaded", [0, 0]),
        (
            "forward_left",
            [controller.scenario.distance, controller.scenario.half_width],
        ),
        ("reverse_home", [0, 0]),
    ]:
        event = next(event for event in controller.events if event["phase"] == phase)
        assert np.linalg.norm(np.asarray(event["xy_m"]) - goal) < 0.25


def test_missing_travel_does_not_advance_loading_or_claim_completion():
    cycle = VCycleController(P, VCycleScenario())
    for step in range(120):
        cycle.command(step * 0.1, 0.1, [0, 0], 0)
    assert cycle.phase_name == "forward_right"
    assert not cycle.done
    cycle.command(cycle.scenario.max_duration, 0.1, [0, 0], 0)
    assert cycle.timeout
    assert not any(event["phase"] == "dump" for event in cycle.events)


def evaluation_record():
    from wheel_loader.evaluation.v_cycle import evaluate_v_cycle

    scenario = VCycleScenario()
    names = ["articulation_joint", "boom_joint", "tilt_joint", "bucket_joint"]
    blocks, poses, labels, commands, events = [], [], [], [], []
    xy = np.zeros(2)
    for phase_id, phase in enumerate(PHASES):
        count = 120
        goal = {
            "forward_right": [scenario.distance, -scenario.half_width],
            "reverse_loaded": [0, 0],
            "forward_left": [scenario.distance, scenario.half_width],
            "reverse_home": [0, 0],
            "push": xy + np.array([0.4, -0.2]),
        }.get(phase, xy)
        points = np.linspace(xy, goal, count)
        root = np.zeros((count, 7))
        root[:, :2] = points
        yaw = (
            -0.4636 if phase in ("forward_right", "push", "reverse_loaded") else 0.4636
        )
        root[:, 5], root[:, 6] = math.sin(yaw / 2), math.cos(yaw / 2)
        q = np.zeros((count, 4))
        q[:, 1] = 0.45
        if phase == "curl_lift":
            q[:, 1] = np.linspace(0.02, 0.45, count)
            q[:, 3] = np.linspace(-0.1, 0.3, count)
        elif phase == "dump":
            q[:, 3] = np.linspace(0.3, -0.4, count)
        command = np.zeros((count, 4))
        command[:, 0] = (
            -0.6
            if phase.startswith("reverse")
            else (
                0.15 if phase == "push" else (0.6 if phase.startswith("forward") else 0)
            )
        )
        blocks.append(q)
        poses.append(root)
        labels.extend([phase_id] * count)
        commands.append(command)
        xy = points[-1]
        events.append({"phase": phase, "xy_m": xy.tolist()})
    arguments = (
        P,
        {
            "joint_names": names,
            "distance": scenario.distance,
            "half_width": scenario.half_width,
        },
        np.concatenate(blocks),
        np.concatenate(poses),
        np.asarray(labels),
        np.concatenate(commands),
        events,
        True,
    )
    return evaluate_v_cycle, arguments


def test_full_evaluation_is_serializable():
    import json

    evaluate, arguments = evaluation_record()
    result = evaluate(*arguments)
    assert all(result["checks"].values())
    json.dumps(result)


@pytest.mark.parametrize(
    "fault,check",
    [
        ("skip_dump", "full_sequence"),
        ("path_detour", "v_path_corridor"),
        ("no_bucket_motion", "dump_motion"),
        ("wrong_dump_direction", "dump_motion"),
        ("wrong_reverse", "reverse_with_bucket_raised"),
        ("bucket_down", "reverse_with_bucket_raised"),
        ("no_return", "return_to_origin"),
    ],
)
def test_incomplete_maneuvers_fail_acceptance(fault, check):
    evaluate, arguments = evaluation_record()
    p, scenario, q, roots, phases, commands, events, completed = arguments
    if fault == "skip_dump":
        events = [event for event in events if event["phase"] != "dump"]
    elif fault == "path_detour":
        roots[phases == PHASES.index("forward_left"), 1] += 10
    elif fault == "no_bucket_motion":
        q[phases == PHASES.index("dump"), 3] = 0.3
    elif fault == "wrong_dump_direction":
        q[phases == PHASES.index("dump"), 3] *= -1
    elif fault == "wrong_reverse":
        mask = phases == PHASES.index("reverse_loaded")
        roots[mask, 5] = math.sin((math.pi - 0.4636) / 2)
        roots[mask, 6] = math.cos((math.pi - 0.4636) / 2)
    elif fault == "bucket_down":
        q[phases == PHASES.index("reverse_loaded"), 1] = 0.02
    else:
        roots[-1, :2] = [1.0, 0]
    result = evaluate(p, scenario, q, roots, phases, commands, events, completed)
    assert not result["checks"][check]


def test_phase_events_serialize_float32_simulator_observations():
    import json

    cycle = VCycleController(P, VCycleScenario())
    cycle.command(2.0, 0.01, np.array([0, 0], dtype=np.float32), np.float32(0))
    json.dumps(cycle.events)
