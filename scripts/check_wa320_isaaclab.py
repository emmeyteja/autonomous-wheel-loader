"""Validate fixed or floating WA320 USD using the existing Isaac Lab/Newton integration."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.isaaclab_probe import (
    run_batch_reset,
    run_probe,
)
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--usd",
        type=Path,
        required=True,
        help="Asset with passive loop; floating for --ground, fixed otherwise",
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/isaaclab"))
    parser.add_argument(
        "--physics-config", type=Path, default=Path("configs/simulation/newton.yaml")
    )
    parser.add_argument(
        "--ground-config",
        type=Path,
        default=Path("configs/simulation/newton_ground.yaml"),
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--ground", action="store_true", help="Validate floating rigid-ground driving"
    )
    mode.add_argument(
        "--batch-reset",
        action="store_true",
        help="Check two-environment reset isolation instead of the single fixture",
    )
    args = parser.parse_args()
    from wheel_loader.environment.simulation.newton_ground import run_ground
    from wheel_loader.environment.simulation.newton_profile import load_profile

    runner = (
        run_ground
        if args.ground
        else (run_batch_reset if args.batch_reset else run_probe)
    )
    kwargs = {}
    if args.ground:
        from wheel_loader.environment.simulation.ground_contact import (
            load_ground_profile,
        )

        kwargs["contact_profile"] = load_ground_profile(args.ground_config)
    runner(
        load_parameters(args.config),
        args.usd,
        args.output,
        profile=load_profile(args.physics_config),
        **kwargs,
    )


if __name__ == "__main__":
    main()
