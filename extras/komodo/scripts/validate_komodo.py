"""Check the converted diagnostic articulation without changing the training stack."""
import argparse
from pathlib import Path
from komodo_reference.validation import validate_articulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--usd', type=Path, required=True)
parser.add_argument('--mapping', type=Path, default=Path('extras/komodo/docs/komodo_mapping.json'))
parser.add_argument('--output', type=Path, default=Path('outputs/komodo/validation'))
args = parser.parse_args()
result = validate_articulation(args.usd, args.mapping, args.output)
if not all(result['checks'].values()):
    raise SystemExit('FAIL: one or more diagnostic articulation checks failed')
