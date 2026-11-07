"""Reoptimize the spherical reference for one changed parameter (Isp or crossflow coefficient).

Usage: python src/spherical-sensitivity.py NAME KEY VALUE [--steps 0.55 0.4] [--rounds 3]
Each run warm-starts from the accepted spherical nominal (or the case's own best converged
candidate when results already exist) and replays every candidate at 0.005 and 0.0025 s.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('p2', ROOT / 'src/p2-collocate.py')
p2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)
DIRECTORY = ROOT / 'results/spherical/sensitivity'


def replay(candidate, change, step):
    request = dict(candidate=candidate, stepS=step, change=change, samples=False)
    out = subprocess.run(['node', 'src/spherical-replay.mjs'], cwd=ROOT, input=json.dumps(request),
                         text=True, capture_output=True, check=True)
    result = json.loads(out.stdout)
    result['trace'] = []
    return result


def accepted(row):
    return row['candidate']['success'] and all(r['passes'] for r in row['replay'].values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name')
    parser.add_argument('key', choices=['ispS', 'crossflowCd'])
    parser.add_argument('value', type=float)
    parser.add_argument('--steps', nargs='+', type=float, default=[.55, .4])
    parser.add_argument('--max-iter', type=int, default=600)
    parser.add_argument('--seed', type=Path, help='result file whose best accepted candidate seeds this case')
    parser.add_argument('--wall', type=float, default=140)
    args = parser.parse_args()
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    path = DIRECTORY / f'{args.name}.json'
    history = p2.load(path)['history'] if path.exists() else []
    change = {args.key: args.value}
    params = dict(p2.load(ROOT / 'config/spherical.json'), **change)
    contract = p2.load(ROOT / 'config/p0-contract.json')
    converged = [row for row in history if accepted(row)]
    if converged:
        start = min(converged, key=lambda row: row['candidate']['propellantKg'])['candidate']
        origin = f'{path.name} best accepted'
    elif args.seed:
        rows = [row for row in p2.load(args.seed)['history'] if accepted(row)]
        start = min(rows, key=lambda row: row['candidate']['propellantKg'])['candidate']
        origin = f'{args.seed.name} best accepted'
    else:
        start = p2.load(ROOT / 'results/spherical/selected.json')['candidate']
        origin = 'results/spherical/selected.json'
    seed = p2.from_candidate(start, {})
    for step in args.steps:
        began = time.perf_counter()
        print('START', args.name, change, step, origin, flush=True)
        try:
            candidate = p2.optimize(params, contract, seed, step, args.max_iter, shooting=True, substeps=8,
                                    max_wall_time=args.wall)
        except RuntimeError as error:
            print('ERROR', error, flush=True)
            continue
        checks = {str(dt): replay(candidate, change, dt) for dt in (.005, .0025)}
        row = dict(candidate=candidate, replay=checks, seedSource=origin,
                   elapsedSeconds=time.perf_counter() - began)
        history.append(row)
        path.write_text(json.dumps(dict(change=change, model='spherical', history=history), indent=1) + '\n')
        print('DONE', args.name, step, candidate['status'], round(candidate['propellantKg'], 1),
              {dt: (r['passes'], {k: v for k, v in r['violations'].items() if v}) for dt, r in checks.items()},
              flush=True)
        if candidate['success']:
            seed = p2.from_candidate(candidate, {})
            origin = f'{path.name} step {step}'


if __name__ == '__main__':
    main()
