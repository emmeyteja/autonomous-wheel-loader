"""Prevent apparently stable but unmatched trials from claiming comparison."""

from copy import deepcopy

import pytest

from wheel_loader.evaluation.column_comparison import compare_columns


@pytest.fixture
def trial():
    return {
        "checks": {"finite_states": True, "target_mass": True},
        "preparation": {
            "protocol": "mass_normalized_expanded_lattice",
            "release": "instantaneous_unconfined",
            "initial_height_m": 0.25,
            "initial_outer_radius_m": 0.12,
            "initial_speed_m_s": 0,
        },
        "represented_mass_kg": 13,
        "duration_s": 12,
        "material_candidate": "gravel",
        "contact_trial": {"particle_density_kg_m3": 2590, "sliding_friction": 0.65},
        "dt_s": 0.001,
        "configuration": {"seed": 42, "element_radius": 0.0125},
        "height_m": 0.15,
        "runout_radius_p95_m": 0.3,
    }


def test_matched_timestep_study_measures_change_without_accuracy_claim(trial):
    second = deepcopy(trial)
    second["dt_s"] /= 2
    second["runout_radius_p95_m"] = 0.33
    result = compare_columns(trial, second, same_timestep=False)
    assert result["sensitivity_comparison_eligible"]
    assert result["changes"]["runout_radius_p95_m"][
        "relative_change_percent"
    ] == pytest.approx(10)
    assert not result["physical_validation_passed"]
    assert not result["convergence_established"]


@pytest.mark.parametrize(
    "mismatch",
    [
        "mass",
        "envelope",
        "missing_preparation",
        "missing_contacts",
        "solver_failure",
        "material",
    ],
)
def test_unmatched_trial_does_not_publish_sensitivity_metrics(trial, mismatch):
    second = deepcopy(trial)
    if mismatch == "mass":
        second["represented_mass_kg"] *= 1.063
    elif mismatch == "envelope":
        second["preparation"]["initial_height_m"] = 0.23
    elif mismatch == "missing_preparation":
        second.pop("preparation")
    elif mismatch == "missing_contacts":
        trial.pop("contact_trial")
        second.pop("contact_trial")
    elif mismatch == "solver_failure":
        second["checks"]["finite_states"] = False
    else:
        second["material_candidate"] = "dry_sand"
    result = compare_columns(trial, second, same_timestep=True)
    assert not result["sensitivity_comparison_eligible"]
    assert result["changes"] == {}
