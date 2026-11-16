# Earlier loading and particle experiments

- Preserved for reference; excluded from active installation and tests.
- Contains the old rail-constrained loader approximation, coarse-sphere pile demos, material-accounting probes and their original tests/docs.
- The old chassis slides along one axis, its wheels are fixed and its two attachment hinges do not form a Z-bar. It is not the active wheel loader.
- Dry-sand settling and material-transfer fidelity remain unvalidated. Historical results do not satisfy the new WA320 gates.
- Source imports remain `wheel_loader_rl` only inside this optional archive.

From the repository root, with the historical optional dependencies available:

```bash
pip install -e extras/legacy_loading
PYTHONPATH=extras/legacy_loading/src pytest extras/legacy_loading/tests -q
./extras/legacy_loading/scripts/demo_pile.sh
```

Historical documents retain their earlier milestone terminology; the active architecture takes precedence.
