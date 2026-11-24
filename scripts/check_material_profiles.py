"""Check physical provenance and independent numerical resolution; no simulation."""

import argparse
import json
from pathlib import Path

from wheel_loader.terrain.materials import load_material, load_resolution


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profiles", nargs="*", type=Path)
    parser.add_argument("--resolution", type=Path)
    args = parser.parse_args()
    paths = args.profiles or sorted(Path("configs/terrain/materials/v1").glob("*.yaml"))
    if not paths:
        parser.error("No material profiles selected")
    resolution = load_resolution(args.resolution) if args.resolution else None
    records = []
    for path in paths:
        material = load_material(path)
        records.append(
            {
                "material_id": material.material_id,
                "material_sha256": material.fingerprint,
                "unknown_properties": [
                    k for k, p in material.properties.items() if p.value is None
                ],
                "resolution_sha256": resolution.fingerprint if resolution else None,
                "physical_validation": "not established by configuration validation",
                "simulator_enabled": False,
            }
        )
    print(json.dumps(records, indent=2))


if __name__ == "__main__":
    main()
