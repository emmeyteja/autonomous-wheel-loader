"""Run in an environment containing the pinned standalone Isaac Sim URDF importer."""
import argparse
from pathlib import Path
from komodo_reference.convert import convert_urdf

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf', type=Path, required=True)
parser.add_argument('--output', type=Path, default=Path('outputs/komodo/usd'))
parser.add_argument('--fix-base', action='store_true', help='Suspended joint-isolation fixture, not a driving model')
args = parser.parse_args()
print(convert_urdf(args.urdf, args.output, args.fix_base))
