"""Collect accepted spherical-Earth cases into results/spherical/summary.json.

A case is accepted when its candidate passes every touchdown check in both the 0.005 s and
0.0025 s spherical replays (and, for dynamic-pressure caps, stays within the cap). The lowest
accepted inventory in each result file is reported; it is a local solution, not a global minimum.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'results/spherical'


def accepted(row, cap=None):
    checks = row.get('replay', {}).values()
    return bool(checks) and all(r['passes'] and (cap is None or r['peakQPa']['value'] <= cap) for r in checks)


def best(path, cap=None):
    history = json.loads(path.read_text())['history']
    rows = [row for row in history if accepted(row, cap)]
    if not rows:
        return dict(source=path.relative_to(ROOT).as_posix(), accepted=False, attempts=len(history))
    row = min(rows, key=lambda r: r['candidate']['propellantKg'])
    fine = row['replay']['0.0025']
    powered = [phase for phase in fine['phases'] if phase['powered']]
    return dict(source=path.relative_to(ROOT).as_posix(), accepted=True, attempts=len(history),
                propellantKg=fine['propellantKg'], methaneKg=fine['methaneKg'], liquidOxygenKg=fine['liquidOxygenKg'],
                consumedKg=fine['consumedKg'], retainedKg=fine['touchdown']['reserveKg'],
                shipLocationM=fine['shipLocationM'], contactTimeS=fine['timeS'],
                maxAltitudeM=fine['maxAltitudeM'], peakQPa=fine['peakQPa']['value'],
                peakLoadG=fine['peakLoadG']['value'], ignitions=fine['ignitionCount'],
                burns=[dict(name=p['name'], startS=p['startS'], endS=p['endS'], consumedKg=p['consumedKg'],
                            startAltitudeM=p['startState'][1]) for p in powered],
                touchdown=fine['touchdown'], solverStatus=row['candidate']['status'],
                meshS=row['candidate']['maxIntervalTargetS'])


summary = dict(note=__doc__.strip(), nominal=None, sensitivity={}, angle={}, qcap={})
selected = json.loads((BASE / 'selected.json').read_text())
summary['nominal'] = dict(source='results/spherical/selected.json', propellantKg=selected['replay']['propellantKg'],
                          shipLocationM=selected['replay']['shipLocationM'],
                          peakQPa=selected['replay']['peakQPa']['value'])
for path in sorted((BASE / 'sensitivity').glob('*.json')):
    summary['sensitivity'][path.stem] = best(path)
for path in sorted((BASE / 'angle').glob('angle-*.json')):
    summary['angle'][path.stem] = best(path)
for path in sorted((BASE / 'qcap').glob('qcap-*.json')):
    cap = json.loads(path.read_text())['qCapPa']
    summary['qcap'][path.stem] = dict(best(path, cap), qCapPa=cap)
(BASE / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
for group in ('sensitivity', 'angle', 'qcap'):
    for name, case in summary[group].items():
        print(group, name, case['accepted'], round(case.get('propellantKg', 0)), round(case.get('shipLocationM', 0)),
              round(case.get('peakQPa', 0)), case.get('ignitions'))
