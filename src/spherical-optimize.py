"""Reoptimize the spherical-Earth reference using the existing P2 optimizer."""
import importlib.util
import json
from pathlib import Path
import subprocess
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('p2',ROOT/'src/p2-collocate.py')
p2=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p2)
p=p2.load(ROOT/'config/spherical.json')
contract=p2.load(ROOT/'config/p0-contract.json')
path=ROOT/'results/spherical/nominal.json'
path.parent.mkdir(parents=True,exist_ok=True)
original=p2.load(ROOT/'results/p2-p0-v1/single-collocation.json')['history'][4]['candidate']
history=p2.load(path)['history'] if path.exists() else []
seed=p2.from_candidate(history[-1]['candidate'] if history else original,{})
for step in ([.4] if history else [.85,.55,.4]):
 print('START spherical refinement',step,flush=True)
 candidate=p2.optimize(p,contract,seed,step,600,shooting=True,substeps=8)
 replay={}
 for dt in [.005,.0025]:
  out=subprocess.run(['node','src/spherical-replay.mjs'],cwd=ROOT,input=json.dumps(dict(candidate=candidate,stepS=dt,samples=dt==.0025)),text=True,capture_output=True,check=True)
  replay[str(dt)]=json.loads(out.stdout)
 row=dict(candidate=candidate,replay=replay)
 history.append(row)
 path.write_text(json.dumps(dict(model=p['description'],history=history),indent=2)+'\n')
 print('DONE',candidate['status'],candidate['propellantKg'],[(dt,r['passes'],r['touchdown']) for dt,r in replay.items()],flush=True)
 if candidate['success']:
  seed=p2.from_candidate(candidate,{})
