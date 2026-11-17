"""Numerical-profile validation stays usable without Isaac, Torch, USD or ROS."""

from pathlib import Path

import pytest

from wheel_loader.environment.simulation.newton_profile import (
    NewtonProfile,
    load_profile,
)

ROOT = Path(__file__).resolve().parents[2]


def test_control_rate_is_preserved_by_solver_substeps():
    settings = load_profile(ROOT / "configs/simulation/newton.yaml")
    dt = 1 / 240
    assert settings.solver_dt(dt) * settings.num_substeps == pytest.approx(dt)
    assert settings.solver_dt(dt) == pytest.approx(1 / 1920)
    settings.validate_timestep(dt)
    with pytest.raises(ValueError):
        NewtonProfile(num_substeps=1).validate_timestep(dt)


@pytest.mark.parametrize(
    "name,value",
    [
        ("num_substeps", 0),
        ("num_substeps", 1.5),
        ("num_substeps", True),
        ("iterations", -1),
        ("njmax", 0),
        ("nconmax", False),
        ("loop_time_constant", float("nan")),
        ("stop_time_constant", float("inf")),
        ("stop_impedance", 1.0),
        ("stop_impedance", 0.0),
        ("stop_time_constant", True),
    ],
)
def test_reject_invalid_numerical_settings(name, value):
    with pytest.raises(ValueError):
        NewtonProfile(**{name: value})


@pytest.mark.parametrize("dt", [0.0, -1.0, float("nan"), True])
def test_reject_invalid_timestep(dt):
    with pytest.raises(ValueError):
        NewtonProfile().solver_dt(dt)


def test_profile_schema_is_explicit(tmp_path):
    file = tmp_path / "bad.yaml"
    file.write_text("num_substeps: 8\n")
    with pytest.raises(ValueError):
        load_profile(file)
    file.write_text("- not-a-mapping\n")
    with pytest.raises(TypeError):
        load_profile(file)


def test_import_audit_understands_mujoco_shorthand_and_detects_missing_settings(
    tmp_path,
):
    from wheel_loader.environment.simulation.newton_profile import audit_import

    joints = "".join(
        f'<joint name="model_Physics_{name}" solreflimit="0.002" solimplimit="0.9999 0.9999 0.001"/>'
        for name in (
            "articulation_joint",
            "boom_joint",
            "tilt_joint",
            "bucket_joint",
            "rod_joint",
        )
    )
    text = f'<mujoco><worldbody><body>{joints}</body></worldbody><equality><connect solref="0.002"/><connect solref="0.002"/></equality><actuator/></mujoco>'
    path = tmp_path / "effective.xml"
    path.write_text(text)
    assert all(audit_import(path, NewtonProfile()).values())
    path.write_text(text.replace('solreflimit="0.002"', 'solreflimit="0.02"'))
    assert not audit_import(path, NewtonProfile())["stop_settings_consumed"]
    path.write_text(text.replace('solref="0.002"', ""))
    assert not audit_import(path, NewtonProfile())["loop_settings_consumed"]
    path.write_text(
        text.replace(
            "<actuator/>",
            '<actuator><general joint="model_Physics_bucket_joint"/></actuator>',
        )
    )
    assert not audit_import(path, NewtonProfile())["passive_links_unactuated"]
