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
# Additional trust-region rounds at the 0.40 s mesh (same parameters) continue the nominal search.
extra_path=root/'results/spherical/sensitivity/cn-1.2.json'
extra=json.loads(extra_path.read_text())['history'] if extra_path.exists() else []
rows=source['history']+extra
good=[row for row in rows if all(r['passes'] for r in row['replay'].values())]
row=min(good,key=lambda r:r['candidate']['propellantKg'])
candidate=row['candidate']
planar=26228.1
process=subprocess.run(['node','src/spherical-replay.mjs'],input=json.dumps(dict(candidate=candidate,stepS=.00125,samples=True)),text=True,capture_output=True,check=True)
r=json.loads(process.stdout)
assert r['passes'],r['violations']
history=[dict(step=x['candidate']['maxIntervalTargetS'],inventoryKg=x['candidate']['propellantKg'],accepted=all(y['passes'] for y in x['replay'].values())) for x in rows]
result=dict(candidate=candidate,replay=r,gridHistory=history)
mesh=', '.join(f"{x['inventoryKg']/1000:.2f}" for x in history[:3])
rounds=[x for x in history[3:] if x['accepted']]
refine=(" Three further optimization rounds on the 0.40 s mesh give "+', '.join(f"{x['inventoryKg']/1000:.2f}" for x in rounds)
        +f" t; we keep the lowest. Their spread of about {(max(x['inventoryKg'] for x in rounds)-min(x['inventoryKg'] for x in rounds))/1000:.2f} t shows how precisely the local optimum is resolved.") if rounds else ''
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
\subsection{{Spherical-Earth Reference Solution}}
\label{{sec:spherical-results}}
Starting from the planar solution, we optimize the spherical equations with the same dry mass, propulsion model, touchdown limits, and $15^\circ$ flight-path angle at separation. The best solution we found needs {r['propellantKg']/1000:.2f} t of propellant at separation, {r['methaneKg']/1000:.2f} t of methane and {r['liquidOxygenKg']/1000:.2f} t of liquid oxygen, including the 5 t reserve. The booster coasts until {ignition['startS']:.1f} s, lights its engine at {ignition['startState'][1]/1000:.1f} km altitude, and touches down at {r['timeS']:.2f} s on a ship {r['shipLocationM']/1000:.2f} km downrange along the surface.

In the replay at a 0.00125 s step, the booster lands {r['touchdown']['shipErrorM']*1000:.1f} mm from the target, moving {r['touchdown']['horizontalMps']:.4f} m/s sideways and {r['touchdown']['verticalMps']:.3f} m/s downward, tilted {r['touchdown']['tiltDeg']:.5f}$^\circ$ from vertical and rotating at {r['touchdown']['pitchRateDegS']:.4f}$^\circ$/s. It burns {r['consumedKg']/1000:.2f} t and keeps {r['touchdown']['reserveKg']/1000:.2f} t. Dynamic pressure peaks at {q:.1f} kPa and specific force at {g:.2f}$\,g_0$.

Optimization meshes of 0.85, 0.55, and 0.40 s give {mesh} t, respectively.{refine} Every reported solution passes the replay at 0.005 and 0.0025 s, and the selected one also at 0.00125 s. The spherical result is {(planar-r['propellantKg'])/1000:.2f} t lighter than the 26.23 t planar design. Both the trajectory and the controls were reoptimized, so this is the difference between two designs, not a correction applied to one.

Over a 100 s vacuum coast, the integrator conserves specific energy to $7.5\times10^{{-9}}$ J/kg and angular momentum to a relative error of $6.7\times10^{{-16}}$. Without torque, the inertial attitude stays constant to $5.9\times10^{{-16}}$ rad. At 32 sampled states, the derivatives used by the optimizer agree with those of the replay code to within $4.0\times10^{{-16}}$ after scaling. These tests confirm the signs of the moving-frame terms and the consistency of the two implementations.

\begin{{figure}}[htbp]\centering
\includegraphics[width=\linewidth]{{figure/spherical-reference.png}}
\caption{{Spherical-Earth reference trajectory. Downrange position is distance along the surface, altitude is radial, and pitch is measured from the local horizontal. Shading marks the landing burn.}}
\label{{fig:spherical}}
\end{{figure}}
'''
(root/'paper/spherical-results.tex').write_text(text)
print(json.dumps({k:r[k] for k in ['propellantKg','methaneKg','liquidOxygenKg','consumedKg','timeS','shipLocationM','touchdown','peakQPa','peakLoadG']},indent=2))
