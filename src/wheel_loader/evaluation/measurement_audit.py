"""Source/uncertainty coverage for active parameters without claiming ground truth."""

import copy


def audit_parameters(parameters, ledger):
    def flatten(value, path=""):
        if isinstance(value, dict):
            return {
                key: val
                for name, item in value.items()
                for key, val in flatten(item, f"{path}.{name}".strip(".")).items()
            }
        if isinstance(value, list) and any(isinstance(item, dict) for item in value):
            return {
                key: val
                for i, item in enumerate(value)
                for key, val in flatten(item, f"{path}.{i}").items()
            }
        return {} if isinstance(value, str) else {path: value}

    flat = flatten(parameters)
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
