"""Prepare local conversion inputs. Diagnostic mode is not a faithful asset."""
import argparse
from pathlib import Path
import subprocess
from wheel_loader_rl.simulation.komodo.inspection import UPSTREAM_REVISION
from wheel_loader_rl.simulation.komodo.prepare import prepare_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('outputs/komodo/input'))
    parser.add_argument('--diagnostic', action='store_true')
    args = parser.parse_args()
    revision = subprocess.check_output(['git', '-C', str(args.source), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != UPSTREAM_REVISION:
        raise SystemExit('Source revision differs from audited revision')
    print(prepare_model(args.source, args.output, args.diagnostic))


if __name__ == '__main__':
    main()
