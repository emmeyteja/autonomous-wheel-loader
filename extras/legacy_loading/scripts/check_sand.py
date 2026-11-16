"""Reproducible contact sensitivity sweep; save evidence without asserting calibration."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import subprocess
import sys
from wheel_loader_rl.config import load_config
from wheel_loader_rl.pile import generate_pile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('extras/legacy_loading/configs/sand_column.toml'))
    parser.add_argument('--rolling', type=float, nargs='+', default=[0.001, 0.005, 0.02])
    parser.add_argument('--seeds', type=int, nargs='+', default=[42, 43])
    parser.add_argument('--duration-s', type=float, default=20)
    parser.add_argument('--output', type=Path, default=Path('outputs/sand_check'))
    args = parser.parse_args()
    cfg = load_config(args.config)
    # Validate all candidates before starting costly simulation processes.
    for rolling in args.rolling:
        for seed in args.seeds:
            generate_pile(replace(cfg.pile, rolling_friction=rolling), replace(cfg, seed=seed).seed)
    results = []
    for rolling in args.rolling:
        for seed in args.seeds:
            output = args.output / f'rolling_{rolling:g}_seed_{seed}'
            output.mkdir(parents=True, exist_ok=True)
            config = output / 'config.toml'
            # Write an exact resolved config rather than modifying source text heuristically.
            data = asdict(replace(cfg, seed=seed, pile=replace(cfg.pile, rolling_friction=rolling)))
            lines = []
            for key, value in data.items():
                if not isinstance(value, dict):
                    lines.append(f'{key} = {json.dumps(value)}')
            for section, values in data.items():
                if isinstance(values, dict):
                    lines.append(f'\n[{section}]')
                    lines.extend(f'{key} = {json.dumps(value)}' for key, value in values.items())
            config.write_text('\n'.join(lines) + '\n')
            with (output / 'run.log').open('w') as log:
                subprocess.run([sys.executable, str(Path(__file__).with_name('demo_pile.py')),
                                '--config', str(config), '--duration-s', str(args.duration_s), '--no-video',
                                '--output', str(output)], stdout=log, stderr=subprocess.STDOUT, check=True)
            summary = json.loads((output / 'summary.json').read_text())
            result = {'seed': seed, 'rolling_friction': rolling, 'settling': summary['settling'],
                      'wall_time_s': summary['wall_time_s'], 'particle_count': summary['particle_count']}
            fits = list(summary['settling']['surface_slopes'].values())
            result['reference_angle_deg'] = 34.0
            result['reference_tolerance_deg'] = 5.0
            result['reference_shape_screen_passed'] = all(fit is not None and fit['r_squared'] >= 0.8
                                                        and abs(fit['angle_deg'] - 34) <= 5 for fit in fits)
            result['calibration_status'] = 'not_validated'
            results.append(result)
            (args.output / 'comparison.json').write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
