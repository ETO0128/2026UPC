"""Generate P1 figures from saved verification results; run from repository root."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Circle
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':150,'savefig.dpi':220})
v=json.loads(Path('results/p1-verification.json').read_text())
s=json.loads(Path('results/p1-verification-series.json').read_text())
fig,ax=plt.subplots(figsize=(8,5))
e=np.array([.55,.835]);n=np.array([-e[1],e[0]]);c=np.array([0.,0.]);tail=-2*e; nose=2*e
ax.add_patch(Polygon([tail-.24*n,nose-.24*n,nose+.24*n,tail+.24*n],facecolor='#d8e5f2',edgecolor='#193b61',lw=2))
def arrow(start,vec,label,color,offset=(0,0)):
 end=start+vec;ax.annotate('',xy=end,xytext=start,arrowprops={'arrowstyle':'->','lw':2,'color':color});ax.text(*(end+offset),label,color=color,fontsize=11)
ax.add_patch(Circle(c,.065,color='#193b61'));ax.text(.15,-.05,'COM')
cp=.65*e;ax.add_patch(Circle(cp,.06,color='#c2410c'));ax.text(*(cp+np.array([.25,.1])),'CP')
arrow(c,np.array([0.,-1.4]),'mg','#444',(.1,-.1))
arrow(cp,1.15*n,r'$F_n$','#c2410c',(-.2,.1))
arrow(cp,-.95*e,r'$F_a$','#c2410c',(.1,-.2))
rot=np.array([[np.cos(.15),-np.sin(.15)],[np.sin(.15),np.cos(.15)]])
arrow(tail,1.55*rot@e,r'$T(\theta+\delta)$','#087f5b',(-1.1,.1))
arrow(np.array([1.5,-1.5]),np.array([1.,0.]),'x','#666',(.1,0))
arrow(np.array([1.5,-1.5]),np.array([0.,1.]),'h','#666',(.1,0))
ax.text(-2.1,1.75,r'$\mathbf{u}=\mathbf{v}-\mathbf{w}$'+'\n'+r'$\tau_a=\sum s_i F_{n,i}$',fontsize=12)
ax.text(-2.1,-2.25,'Rigid cylinder; signed pitch measured from +x.\nCP offset and gimbal angle exaggerated.',fontsize=10)
ax.set(xlim=(-2.6,3),ylim=(-2.4,2.4),aspect='equal');ax.axis('off');fig.tight_layout();fig.savefig('figure/p1-forces.png');plt.close(fig)
fig,axes=plt.subplots(2,3,figsize=(11,6.5))
for label,key,style in [('Nominal','nominal','-'),('Pitch disturbance','perturbed','--')]:
 rows=s[key];t=np.array([r['t'] for r in rows]);y=np.array([r['state'] for r in rows]);
 data=[y[:,1]-32.5,y[:,3],(y[:,5]-np.pi/2)*180/np.pi,(180000-y[:,4])/1000,np.clip(np.array([r['command']['throttle'] for r in rows]),.4,1),y[:,0]]
 for ax,values in zip(axes.flat,data):ax.plot(t,values,style,label=label,lw=1.8)
for ax,title,unit in zip(axes.flat,['COM height above upright contact','Vertical speed','Pitch error','Consumed propellant','Applied throttle','Horizontal displacement'],['m','m/s','deg','t','1','m']):
 ax.set(title=title,xlabel='Time (s)',ylabel=unit);ax.grid(alpha=.2)
axes[1,1].axhline(.4,color='gray',ls=':',lw=1)
axes[1,2].axhline(5,color='#c2410c',ls=':',label='Fixed-ship tolerance')
axes[0,0].legend(fontsize=8);axes[1,2].legend(fontsize=8)
fig.tight_layout();fig.savefig('figure/p1-terminal.png');plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(11,3.5))
a=v['timeConvergence'];axes[0].loglog([r['dt'] for r in a],[r['positionErrorM'] for r in a],'o-');axes[0].set(xlabel='Time step (s)',ylabel='Position difference (m)',title='Smooth dynamics')
a=v['stripConvergence'];axes[1].loglog([r['strips'] for r in a],[r['relativeError'] for r in a],'o-');axes[1].set(xlabel='Strip count',ylabel='Relative torque error',title='Pure-rotation quadrature')
a=v['eventConvergence'];axes[2].loglog([r['dt'] for r in a[:-1]],[abs(r['consumedKg']-a[-1]['consumedKg']) for r in a[:-1]],'o-');axes[2].set(xlabel='Time step (s)',ylabel='Consumption difference (kg)',title='Terminal event')
for ax in axes:ax.grid(alpha=.25,which='both')
from matplotlib.ticker import FixedLocator, FixedFormatter, NullFormatter
for ax in [axes[0],axes[2]]:
 ax.xaxis.set_major_locator(FixedLocator([.01,.02,.04,.08]));ax.xaxis.set_major_formatter(FixedFormatter(['0.01','0.02','0.04','0.08']));ax.xaxis.set_minor_formatter(NullFormatter())
axes[1].xaxis.set_major_locator(FixedLocator([10,20,40,80,160]));axes[1].xaxis.set_major_formatter(FixedFormatter(['10','20','40','80','160']));axes[1].xaxis.set_minor_formatter(NullFormatter())
fig.tight_layout();fig.savefig('figure/p1-convergence.png');plt.close(fig)
print('Generated 3 P1 figures')
