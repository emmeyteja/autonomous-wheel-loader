"""Inspect a locally obtained, pinned Komodo checkout without running its software."""
import argparse
import json
from pathlib import Path
import subprocess
from wheel_loader_rl.simulation.komodo.inspection import inspect_model, UPSTREAM_REVISION


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('outputs/komodo/inspection.json'))
    args = parser.parse_args()
    revision = subprocess.check_output(['git', '-C', str(args.source), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != UPSTREAM_REVISION:
        raise SystemExit('Source revision differs from the audited revision; review before conversion')
    report = inspect_model(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'links': len(report['links']), 'joints': len(report['joints']),
                      'missing_meshes': report['missing_meshes'], 'invalid_inertia_links': report['invalid_inertia_links'],
                      'faithful_conversion_ready': report['faithful_conversion_ready']}, indent=2))


if __name__ == '__main__':
    main()
