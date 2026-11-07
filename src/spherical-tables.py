"""Write LaTeX tables for the spherical parameter, angle, and dynamic-pressure-cap studies.

Reads results/spherical/summary.json (src/spherical-summary.py) and writes
paper/spherical-sensitivity-tables.tex, paper/angle-table.tex and paper/qcap-results.tex.
Planar comparator values are the accepted planar results reported in Sections 6-7.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
s = json.loads((ROOT / 'results/spherical/summary.json').read_text())
nominal = s['nominal']['propellantKg']
t = lambda kg: f'{kg / 1000:.2f}'


def case(name):
    c = s['sensitivity'].get(name) or s['angle'].get(name) or s['qcap'].get(name)
    if not c or not c['accepted']:
        raise SystemExit(f'{name} has no accepted candidate')
    return c


isp_rows = [(300, case('isp-300'), 28.781), (330, None, 26.228), (360, case('isp-360'), 24.199)]
lines = [r'\begin{table}[htbp]', r'\centering',
         r'\caption{Effective-$I_{sp}$ comparison on the spherical model. Each design reoptimizes the inventory, controls, and ship position. Masses in tonnes, rounded independently. The last column is the earlier planar result.}',
         r'\label{tab:p3-isp-budget}', r'\begin{tabular}{@{}rrrrrrr@{}}', r'\toprule',
         r'$I_{sp}$ (s) & Inventory & Consumed & Retained & Methane & Oxygen & Planar \\', r'\midrule']
for isp, c, planar in isp_rows:
    if c is None:
        sel = json.loads((ROOT / 'results/spherical/selected.json').read_text())['replay']
        c = dict(propellantKg=sel['propellantKg'], consumedKg=sel['consumedKg'], retainedKg=sel['touchdown']['reserveKg'],
                 methaneKg=sel['methaneKg'], liquidOxygenKg=sel['liquidOxygenKg'])
    lines.append(f"{isp} & {t(c['propellantKg'])} & {t(c['consumedKg'])} & {t(c['retainedKg'])} & "
                 f"{t(c['methaneKg'])} & {t(c['liquidOxygenKg'])} & {planar:.2f} \\\\")
lines += [r'\bottomrule', r'\end{tabular}', r'\end{table}', '']
cn_rows = [(0.8, case('cn-0.8'), None), (1.0, case('cn-1.0'), None), (1.2, None, 26.228), (1.6, case('cn-1.6'), 27.130)]
lines += [r'\begin{table}[htbp]', r'\centering',
          r'\caption{Crossflow-coefficient comparison on the spherical model, axial coefficient fixed at 0.8. The last column is the earlier planar result; a dash marks a case without a planar solution.}',
          r'\label{tab:p3-drag}', r'\begin{tabular}{@{}rrrrr@{}}', r'\toprule',
          r'$C_n$ & Inventory (t) & Peak $q$ (kPa) & Peak specific force ($g_0$) & Planar (t) \\', r'\midrule']
sel = json.loads((ROOT / 'results/spherical/selected.json').read_text())['replay']
for cn, c, planar in cn_rows:
    if c is None:
        c = dict(propellantKg=sel['propellantKg'], peakQPa=sel['peakQPa']['value'], peakLoadG=sel['peakLoadG']['value'])
    lines.append(f"{cn:.1f} & {t(c['propellantKg'])} & {c['peakQPa'] / 1000:.1f} & {c['peakLoadG']:.2f} & "
                 f"{'--' if planar is None else f'{planar:.2f}'} \\\\")
lines += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
(ROOT / 'paper/spherical-sensitivity-tables.tex').write_text('\n'.join(lines) + '\n')

angle = [(0, case('angle-0'), 196.00, 24.71), (15, None, 278.31, 26.23), (30, case('angle-30'), 260.00, 67.00)]
lines = [r'\begin{table}[htbp]', r'\centering',
         r'\caption{Accepted strategies for three separation angles. Ship position is surface arc distance from the separation point. Inventories include the 5 t reserve. The planar values stopped the atmosphere at 84.852 km geopotential altitude.}',
         r'\label{tab:p2-separation-angle}', r'\small', r'\begin{tabular}{@{}rrrrrr@{}}', r'\toprule',
         r'Angle & Apex (km) & Ship (km) & Inventory (t) & Planar ship (km) & Planar inventory (t) \\', r'\midrule']
for deg, c, pship, pinv in angle:
    if c is None:
        c = dict(maxAltitudeM=sel['maxAltitudeM'], shipLocationM=sel['shipLocationM'], propellantKg=sel['propellantKg'])
    apex = max(c['maxAltitudeM'], 70000)
    lines.append(f"${deg}^\\circ$ & {apex / 1000:.1f} & {c['shipLocationM'] / 1000:.2f} & {t(c['propellantKg'])} & "
                 f"{pship:.2f} & {pinv:.2f} \\\\")
lines += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
(ROOT / 'paper/angle-table.tex').write_text('\n'.join(lines) + '\n')

lines = [r'\begin{table}[htbp]', r'\centering',
         r'\caption{Dynamic-pressure caps for the $15^\circ$ spherical case. Each row is the best accepted solution for that limit. Masses in tonnes.}',
         r'\label{tab:qcap}', r'\begin{tabular}{@{}lllrrrr@{}}', r'\toprule',
         r'Cap (kPa) & Status & Burns & Inventory & Entry burn & Landing burn & Peak $q$ (kPa) \\', r'\midrule',
         f"none & optimized & 1 & {t(sel['propellantKg'])} & -- & {t(sel['consumedKg'])} & {sel['peakQPa']['value'] / 1000:.1f} \\\\"]
for key, c in sorted(s['qcap'].items(), key=lambda kv: -kv[1]['qCapPa']):
    cap = c['qCapPa'] / 1000
    if c['accepted']:
        entry, landing = c['burns'][0]['consumedKg'], c['burns'][-1]['consumedKg']
        lines.append(f"{cap:.0f} & optimized & {c['ignitions']} & {t(c['propellantKg'])} & {t(entry)} & {t(landing)} & {c['peakQPa'] / 1000:.1f} \\\\")
    else:
        policy = json.loads((ROOT / f'results/spherical/qcap/policy-{cap:g}.json').read_text())
        r = policy['result']
        burns = [p for p in r['phases'] if p['powered']]
        lines.append(f"{cap:.0f} & feedback bound & {r['ignitionCount']} & {t(r['propellantKg'])} & {t(burns[0]['consumedKg'])} & "
                     f"{t(burns[-1]['consumedKg'])} & {r['peakQPa']['value'] / 1000:.1f} \\\\")
lines += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
caps = {round(c['qCapPa'] / 1000): c for c in s['qcap'].values() if c['accepted']}
a, b = caps[200], caps[150]
def entry(c):
    e = c['burns'][0]
    return f"a {e['endS'] - e['startS']:.0f} s entry burn starting at {e['startAltitudeM'] / 1000:.1f} km altitude"
lines += ['', (f"A 200 kPa cap raises the separation inventory from {sel['propellantKg'] / 1000:.2f} t to {a['propellantKg'] / 1000:.2f} t. "
    f"The added propellant goes into {entry(a)}, while the landing burn still uses about {a['burns'][-1]['consumedKg'] / 1000:.0f} t. "
    f"A 150 kPa cap needs {b['propellantKg'] / 1000:.2f} t, with {entry(b)}. "
    f"Lowering the limit from 200 to 150 kPa costs another {(b['propellantKg'] - a['propellantKg']) / 1000:.1f} t. "
    "These solutions pass the same replay checks as the reference. They were still improving when the optimization was stopped, "
    "so they are upper estimates of the minimum.")]
(ROOT / 'paper/qcap-results.tex').write_text('\n'.join(lines) + '\n')
(ROOT / 'paper/qcap-macros.tex').write_text(f"\\newcommand{{\\QcapTwoHundred}}{{{a['propellantKg'] / 1000:.2f}}}\n\\newcommand{{\\QcapOneFifty}}{{{b['propellantKg'] / 1000:.2f}}}\n")
print('tables written')
