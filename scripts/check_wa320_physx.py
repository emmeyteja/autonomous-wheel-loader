"""Run optional isolated PhysX checks; this is not a training environment."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.physx_probe import run_probe
from wheel_loader.terrain.rigid_pile import load_terrain
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/physx"))
    parser.add_argument("--fixture", action="store_true")
    parser.add_argument("--pile", action="store_true")
    parser.add_argument(
        "--terrain-config", type=Path, default=Path("configs/terrain/rigid_pile.yaml")
    )
    parser.add_argument("--capture", action="store_true")
    args = parser.parse_args()
    run_probe(
        load_parameters(args.config),
        args.usd,
        args.output,
        fixture=args.fixture,
        pile=args.pile,
        capture=args.capture,
        terrain=load_terrain(args.terrain_config) if args.pile else None,
    )


if __name__ == "__main__":
    main()
