"""Source/uncertainty coverage for active parameters without claiming ground truth."""

import copy


def audit_parameters(parameters, ledger):
    flat = {
        f"{section}.{key}": value
        for section, values in parameters.items()
        for key, value in values.items()
    }
    entries = ledger["active_parameters"]
    if set(flat) != set(entries):
        raise ValueError(
            f"Provenance coverage differs: missing={sorted(set(flat) - set(entries))}, stale={sorted(set(entries) - set(flat))}"
        )
    result = {}
    for path, value in flat.items():
        entry = entries[path]
        if not entry["unit"] or any(
            s not in ledger["sources"] for s in entry["sources"]
        ):
            raise ValueError(f"Missing units or source: {path}")
        if entry["true_vehicle_error"] is not None:
            raise ValueError("A real-machine error needs a matched measurement dataset")
        result[path] = {"value": copy.deepcopy(value), **copy.deepcopy(entry)}
    return {
        **copy.deepcopy(ledger),
        "active": result,
        "real_machine_measurements": ledger["real_machine_measurements"],
        "physical_accuracy_established": False,
    }
