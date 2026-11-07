"""Optimize a spherical separation-angle case from its feedback-policy seed.

Usage: python src/spherical-angle-optimize.py ANGLE_DEG
Seed: results/spherical/angle/policy-ANGLE.json (src/spherical-angle-search.mjs).
Uses the isothermal upper-atmosphere continuation so the apex is not capped artificially.
"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('p2', ROOT / 'src/p2-collocate.py')
p2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)

angle = float(sys.argv[1])
tag = f'{angle:g}'
change = dict(flightPathDeg=angle, pitchDeg=angle, upperAtmosphere='isothermal')
params = dict(p2.load(ROOT / 'config/spherical.json'), **change)
contract = p2.load(ROOT / 'config/p0-contract.json')
path = ROOT / f'results/spherical/angle/angle-{tag}.json'
history = p2.load(path)['history'] if path.exists() else []


def replay(candidate, step):
    request = dict(candidate=candidate, stepS=step, change=change, samples=False)
    out = subprocess.run(['node', 'src/spherical-replay.mjs'], cwd=ROOT, input=json.dumps(request),
                         text=True, capture_output=True, check=True)
    result = json.loads(out.stdout)
    result['trace'] = []
    return result


def run(seed, step, shooting):
    began = time.perf_counter()
    print('START', tag, 'shooting' if shooting else 'collocation', step, flush=True)
    try:
        candidate = p2.optimize(params, contract, seed, step, 600, shooting=shooting, substeps=8, max_wall_time=600)
    except RuntimeError as error:
        print('ERROR', error, flush=True)
        return None
    checks = {str(dt): replay(candidate, dt) for dt in (.005, .0025)}
    history.append(dict(candidate=candidate, replay=checks, elapsedSeconds=time.perf_counter() - began))
    path.write_text(json.dumps(dict(change=change, model='spherical', history=history), indent=1) + '\n')
    print('DONE', step, candidate['status'], round(candidate['propellantKg'], 1),
          {dt: (r['passes'], {k: v for k, v in r['violations'].items() if v}) for dt, r in checks.items()}, flush=True)
    return candidate if all(r['passes'] for r in checks.values()) else None


converged = [row['candidate'] for row in history if all(r['passes'] for r in row['replay'].values())]
if converged:
    seed = p2.from_candidate(converged[-1], {})
    plan = [(.4, True), (.4, True)]
else:
    seed = p2.load(ROOT / f'results/spherical/angle/policy-{tag}.json')['result']
    plan = [(1.0, False), (.85, True), (.55, True), (.4, True)]
for step, shooting in plan:
    candidate = run(seed, step, shooting)
    if candidate:
        seed = p2.from_candidate(candidate, {})
