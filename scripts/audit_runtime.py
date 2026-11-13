"""Write a reproducible local runtime inventory without importing Isaac/Kit."""
import argparse
from datetime import datetime, timezone
from importlib import metadata
import json
from pathlib import Path
import platform
import subprocess
import sys


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'available': False, 'error': type(exc).__name__}
    return {'returncode': result.returncode, 'stdout': result.stdout.strip(), 'stderr': result.stderr.strip()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/m0/runtime.json'))
    args = parser.parse_args()
    packages = {}
    for name in ('isaaclab', 'isaacsim', 'torch', 'newton', 'warp-lang', 'numpy', 'gymnasium', 'rsl-rl-lib'):
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = None
    result = {
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'python': sys.version, 'platform': platform.platform(), 'packages': packages,
        'gpu': command(['nvidia-smi', '--query-gpu=name,driver_version,memory.total', '--format=csv,noheader']),
        'git_commit': command(['git', 'rev-parse', 'HEAD']),
        'git_status': command(['git', 'status', '--short']),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(args.output)


if __name__ == '__main__':
    main()
