"""Run with the optional Isaac Sim Python environment after accepting its EULA."""
import argparse
from pathlib import Path
from komodo_reference.isaacsim_capture import capture

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--usd', type=Path, required=True)
parser.add_argument('--output', type=Path, default=Path('outputs/komodo/isaacsim'))
args = parser.parse_args()
capture(args.usd, args.output)
