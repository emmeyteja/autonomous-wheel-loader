"""Small unchanged-collider contact suite; no vehicle or training environment."""

import argparse
import json
from pathlib import Path

from wheel_loader.environment.simulation.edge_entry import EDGE_VARIANTS, run_case

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bundle", type=Path, default=Path("outputs/wa320/backend_benchmark/bundle")
    )
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--variant", choices=list(EDGE_VARIANTS), default="blunt")
    p.add_argument("--attack-height", type=float, default=0.085)
    p.add_argument(
        "--backstop",
        action="store_true",
        help="Straight entry only; max advance 0.70 m",
    )
    a = p.parse_args()
    if not (0.07 <= a.attack_height <= 0.085):
        p.error("Diagnostic attack-height range is 0.070–0.085 m")
    if a.output.exists() and any(a.output.iterdir()):
        p.error("Output directory must be empty")
    results = {}
    for count in (1, 3):
        for label, depth in [
            ("straight", -1.0),
            ("curl_040", 0.40),
            ("curl_095", 0.95),
        ]:
            if a.backstop and label != "straight":
                continue
            name = f"row_{count}_{label}"
            results[name] = run_case(
                a.bundle,
                a.output / name,
                count,
                depth,
                a.variant,
                a.attack_height,
                a.backstop,
            )
            print(name, json.dumps(results[name]), flush=True)
    (a.output / "comparison.json").write_text(json.dumps(results, indent=2) + "\n")
