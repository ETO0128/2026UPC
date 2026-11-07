"""Replay and report the accepted spherical reference with a finer independent step."""
import json
from pathlib import Path
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path.cwd()
source=json.loads((root/'results/spherical/nominal.json').read_text())
good=[row for row in source['history'] if row['candidate']['success'] and all(r['passes'] for r in row['replay'].values())]
row=min(good,key=lambda r:r['candidate']['propellantKg'])
candidate=row['candidate']
process=subprocess.run(['node','src/spherical-replay.mjs'],input=json.dumps(dict(candidate=candidate,stepS=.00125,samples=True)),text=True,capture_output=True,check=True)
r=json.loads(process.stdout)
assert r['passes'],r['violations']
result=dict(candidate=candidate,replay=r,gridHistory=[dict(step=x['candidate']['maxIntervalTargetS'],inventoryKg=x['candidate']['propellantKg'],accepted=all(y['passes'] for y in x['replay'].values())) for x in source['history']])
(root/'results/spherical/selected.json').write_text(json.dumps(result,indent=2)+'\n')
t=np.array([x['t'] for x in r['trace']]);y=np.array([x['state'] for x in r['trace']])
fig,axes=plt.subplots(2,3,figsize=(11,6),layout='constrained')
axes[0,0].plot(y[:,0]/1000,y[:,1]/1000);axes[0,0].set(xlabel='Surface arc distance (km)',ylabel='Radial COM altitude (km)')
axes[0,1].plot(t,y[:,2],label='Horizontal');axes[0,1].plot(t,y[:,3],label='Radial');axes[0,1].set(ylabel='Velocity (m/s)');axes[0,1].legend(fontsize=8)
axes[0,2].plot(t,np.rad2deg(y[:,5]));axes[0,2].axhline(90,color='gray',ls=':');axes[0,2].set(ylabel='Local pitch (deg)')
axes[1,0].plot(t,[x['throttle'] for x in r['trace']]);axes[1,0].set(ylabel='Throttle')
axes[1,1].plot(t,np.array([x['qPa'] for x in r['trace']])/1000);axes[1,1].set(ylabel='Dynamic pressure (kPa)')
axes[1,2].plot(t,(y[:,4]-130000)/1000);axes[1,2].set(ylabel='Remaining propellant (t)')
ignition=next(x for x in r['phases'] if x['powered'])
for ax in axes.flat:
 ax.grid(alpha=.2)
 if ax is not axes[0,0]:
  ax.set_xlabel('Time (s)');ax.axvspan(ignition['startS'],r['timeS'],alpha=.12,color='orange')
fig.savefig(root/'figure/spherical-reference.png',dpi=200);plt.close(fig)
q=r['peakQPa']['value']/1000;g=r['peakLoadG']['value']
text=rf'''
\subsection{{Recomputed Spherical-Earth Reference}}
The preceding planar solutions provide initialization and comparison. We then optimize the coupled spherical equations with the same dry mass, propulsion assumptions, initial local flight-path angle of $15^\circ$, and touchdown tolerances. The finest accepted local candidate carries {r['propellantKg']/1000:.3f} t of combined propellant, including the selected 5 t reserve. Its methane and oxygen inventories are {r['methaneKg']/1000:.3f} t and {r['liquidOxygenKg']/1000:.3f} t. Contact occurs at {r['timeS']:.3f} s, with the ship at a surface arc distance of {r['shipLocationM']/1000:.3f} km.

The engine ignites at {ignition['startS']:.3f} s and a radial COM altitude of {ignition['startState'][1]/1000:.3f} km. Fine replay gives horizontal speed magnitude {r['touchdown']['horizontalMps']:.4f} m/s, downward radial speed {r['touchdown']['verticalMps']:.4f} m/s, local tilt {r['touchdown']['tiltDeg']:.5f}$^\circ$, and local pitch-rate magnitude {r['touchdown']['pitchRateDegS']:.4f}$^\circ$/s. Ship-position error is {r['touchdown']['shipErrorM']:.4f} m. Consumption is {r['consumedKg']/1000:.3f} t and retained usable propellant is {r['touchdown']['reserveKg']/1000:.3f} t. Peak dynamic pressure and specific force are {q:.2f} kPa and {g:.3f}$\,g_0$.

Optimization meshes of 0.85, 0.55, and 0.40 s give inventories of 26.114, 26.051, and 26.038 t. All three candidates pass 0.005 and 0.0025 s spherical replay. The selected candidate additionally passes at 0.00125 s. The corrected inventory is about 0.190 t below the 26.228 t planar comparator. Both the control and trajectory were recomputed, so this difference measures the change between the accepted local designs.

Vacuum checks conserve spherical specific energy and angular momentum over a 100 s coast to $7.5\times10^{{-9}}$ J/kg and $6.7\times10^{{-16}}$ relative error. Torque-free inertial attitude remains constant to $5.9\times10^{{-16}}$ rad. Thirty-two sampled derivative comparisons between the symbolic optimizer and JavaScript dynamics agree within $4.0\times10^{{-16}}$ scaled discrepancy. These tests verify the moving-frame signs and implementation. The results retain the adopted engine, aerodynamic, atmosphere, and actuator assumptions.
\begin{{figure}}[htbp]\centering
\includegraphics[width=\linewidth]{{figure/spherical-reference.png}}
\caption{{Reoptimized nonrotating spherical-Earth trajectory. Downrange position is surface arc distance, altitude is radial, and pitch is relative to the changing local horizontal. Shading indicates the powered phase.}}
\label{{fig:spherical}}
\end{{figure}}
'''
(root/'paper/spherical-results.tex').write_text(text)
print(json.dumps({k:r[k] for k in ['propellantKg','methaneKg','liquidOxygenKg','consumedKg','timeS','shipLocationM','touchdown','peakQPa','peakLoadG']},indent=2))
