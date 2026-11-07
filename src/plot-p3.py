import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
r=json.loads(Path('results/p3/fixed-policy.json').read_text());rows=[c for c in r['cases'] if not c['isReferenceParameterValue']]
keys=['contact','ship','horizontal','vertical','tilt','pitchRate','reserve']
a=[]
for c in rows:
 f=c['fine'];a.append([int(f['violations'][k]) if k=='contact' or f['event']=='contact' else 2 for k in keys])
fig,ax=plt.subplots(figsize=(9,12));ax.imshow(a,aspect='auto',cmap=ListedColormap(['#d1e7dd','#e8a09a','#ddd']),vmin=0,vmax=2)
ax.set_yticks(range(len(rows)),[f"{c['parameter']} = {c['value']}" for c in rows],fontsize=8)
ax.set_xticks(range(len(keys)),['Contact','Position','Horiz. speed','Vert. speed','Tilt','Pitch rate','Reserve'],rotation=30,ha='right',fontsize=9)
ax.set_title('Fixed inventory, ship and open-loop controls\nGreen: pass; red: fail; grey: no touchdown assessment',fontsize=12,pad=16)
ax.tick_params(length=0);fig.tight_layout();fig.savefig('figure/p3-fixed-policy.png',dpi=200);plt.close(fig)
print('Generated fixed-policy sensitivity figure')
