"""Minimize the 15-degree spherical inventory with an entry burn and a dynamic-pressure cap.

Usage: python src/spherical-q-optimize.py QMAX_KPA
Seed: results/spherical/qcap/policy-QMAX.json (src/spherical-q-search.mjs).
The cap is imposed at mesh knots and collocation midpoints with a 1% planning margin;
acceptance requires the replayed continuous peak to stay at or below the stated cap.
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

cap_kpa = float(sys.argv[1])
cap = cap_kpa * 1000
tag = f'{cap_kpa:g}'
params = p2.load(ROOT / 'config/spherical.json')
contract = p2.load(ROOT / 'config/p0-contract.json')
path = ROOT / f'results/spherical/qcap/qcap-{tag}.json'
history = p2.load(path)['history'] if path.exists() else []


def replay(candidate, step):
    request = dict(candidate=candidate, stepS=step, change={}, samples=False)
    out = subprocess.run(['node', 'src/spherical-replay.mjs'], cwd=ROOT, input=json.dumps(request),
                         text=True, capture_output=True, check=True)
    result = json.loads(out.stdout)
    result['trace'] = []
    result['withinQCap'] = result['peakQPa']['value'] <= cap
    return result


def run(seed, step, shooting):
    began = time.perf_counter()
    print('START', tag, 'shooting' if shooting else 'collocation', step, flush=True)
    try:
        candidate = p2.optimize(params, contract, seed, step, 800, shooting=shooting, substeps=8, max_q=0.99 * cap, max_wall_time=900)
    except RuntimeError as error:
        print('ERROR', error, flush=True)
        return None
    checks = {str(dt): replay(candidate, dt) for dt in (.005, .0025)}
    history.append(dict(candidate=candidate, replay=checks, qCapPa=cap, elapsedSeconds=time.perf_counter() - began))
    path.write_text(json.dumps(dict(qCapPa=cap, model='spherical', history=history), indent=1) + '\n')
    print('DONE', step, candidate['status'], round(candidate['propellantKg'], 1),
          {dt: (r['passes'], r['withinQCap'], round(r['peakQPa']['value']), {k: v for k, v in r['violations'].items() if v})
           for dt, r in checks.items()}, flush=True)
    return candidate if all(r['passes'] and r['withinQCap'] for r in checks.values()) else None


accepted = [row for row in history if all(r['passes'] and r['withinQCap'] for r in row['replay'].values())]
converged = [min(accepted, key=lambda row: row['candidate']['propellantKg'])['candidate']] if accepted else []
if converged:
    seed = p2.from_candidate(converged[-1], {})
    plan = [(.55, True), (.4, True), (.4, True)]
else:
    seed = p2.load(ROOT / f'results/spherical/qcap/policy-{tag}.json')['result']
    plan = [(.85, True), (.85, True), (.55, True), (.55, True), (.4, True)]
for step, shooting in plan:
    candidate = run(seed, step, shooting)
    if candidate:
        seed = p2.from_candidate(candidate, {})
