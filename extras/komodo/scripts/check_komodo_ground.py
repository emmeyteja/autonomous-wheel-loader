"""Measure the free-base diagnostic without changing source motor effort limits."""
import argparse
from pathlib import Path
from komodo_reference.ground_probe import probe_ground

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--usd', type=Path, required=True)
parser.add_argument('--output', type=Path, default=Path('outputs/komodo/ground'))
args = parser.parse_args()
result = probe_ground(args.usd, args.output)
if not (result['straight_screen_passed'] and result['turn_screen_passed']):
    raise SystemExit('FAIL: ground-motion screening thresholds not met')
