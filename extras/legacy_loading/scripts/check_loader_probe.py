"""Fail when the saved loader probe has not demonstrated retention and deposition."""
import argparse
import json
from pathlib import Path
import numpy as np
from wheel_loader_rl.material_accounting import transfer_metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='Directory containing pile_motion.npz and summary.json')
    args = parser.parse_args()
    summary = json.loads((args.output / 'summary.json').read_text())
    with np.load(args.output / 'pile_motion.npz') as trace:
        names = list(trace['loader_body_names'])
        result = transfer_metrics(trace['positions_m'], trace['time_s'],
                                  trace['loader_body_poses'][:, names.index('bucket')],
                                  float(trace['radius_m']), summary['assumptions']['sphere_mass_kg'])
        finite = bool(np.isfinite(trace['loader_body_poses']).all() and np.isfinite(trace['loader_joint_positions']).all())
    print(json.dumps(result, indent=2))
    if not (finite and summary['all_states_finite'] and result.get('transfer_screen_passed', False)
            and result.get('conservation_error_kg') == 0):
        raise SystemExit('FAIL: loader/material feasibility screen has not passed')
    print('PASS: finite loader states, acquisition, retention, ground-pad release, and particle bookkeeping')


if __name__ == '__main__':
    main()
