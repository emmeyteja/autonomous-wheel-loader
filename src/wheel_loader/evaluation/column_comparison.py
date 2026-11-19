"""Eligibility and sensitivity metrics for independent column diagnostics."""

import numpy as np


def compare_columns(first, second, *, same_timestep):
    """Compare measured outputs only when mass and preparation controls match.

    These controls permit a diagnostic sensitivity comparison, not a physical
    validation or a convergence claim. Element shape and lattice fabric remain
    approximations; fixed length-valued contact parameters also change relative
    to element size on refinement.
    """
    controls = {}
    for name, result in (("first", first), ("second", second)):
        controls[f"{name}_numerical_checks"] = bool(result.get("checks")) and all(
            result["checks"].values()
        )
    a, b = first.get("preparation", {}), second.get("preparation", {})
    controls["controlled_preparation"] = (
        a.get("protocol") == b.get("protocol") == "mass_normalized_expanded_lattice"
        and a.get("release") == b.get("release") == "instantaneous_unconfined"
    )
    for key, atol in (
        ("represented_mass_kg", 1e-8),
        ("duration_s", 1e-6),
    ):
        controls[key] = _close(first.get(key), second.get(key), atol)
    for key in ("initial_height_m", "initial_outer_radius_m", "initial_speed_m_s"):
        controls[key] = _close(a.get(key), b.get(key), 1e-10)
    controls["same_material_and_contacts"] = (
        bool(first.get("material_candidate"))
        and bool(first.get("contact_trial"))
        and first.get("material_candidate") == second.get("material_candidate")
        and first.get("contact_trial") == second.get("contact_trial")
    )
    controls["valid_timesteps"] = all(
        result.get("dt_s") is not None
        and np.isfinite(result["dt_s"])
        and result["dt_s"] > 0
        for result in (first, second)
    )
    controls["timestep_control"] = (
        _close(first.get("dt_s"), second.get("dt_s"), 1e-12)
        if same_timestep
        else bool(first.get("configuration"))
        and first.get("configuration") == second.get("configuration")
    )
    eligible = all(controls.values())
    changes = {}
    if eligible:
        for key in ("height_m", "runout_radius_p95_m"):
            initial, final = first[key], second[key]
            changes[key] = {
                "first": initial,
                "second": final,
                "relative_change_percent": 100 * (final - initial) / initial,
            }
    return {
        "controls": controls,
        "sensitivity_comparison_eligible": eligible,
        "changes": changes,
        "convergence_established": False,
        "physical_validation_passed": False,
    }


def _close(a, b, atol):
    return bool(
        a is not None
        and b is not None
        and np.isfinite(a)
        and np.isfinite(b)
        and np.isclose(a, b, atol=atol, rtol=0)
    )
