import importlib.util,json,subprocess
from pathlib import Path
import numpy as np
root=Path.cwd()
spec=importlib.util.spec_from_file_location('p2',root/'src/p2-collocate.py');p2=importlib.util.module_from_spec(spec);spec.loader.exec_module(p2)
p=p2.load(root/'config/spherical.json');f=p2.p1_rhs(p)
rng=np.random.default_rng(758)
rows=[]
for _ in range(32):
 state=[rng.uniform(0,300000),rng.uniform(100,80000),rng.uniform(-300,1700),rng.uniform(-900,500),rng.uniform(140000,220000),rng.uniform(-2,5),rng.uniform(-.2,.2)]
 control=[rng.uniform(.4,1),rng.uniform(-1,1)];rows.append(dict(state=state,control=control))
code="import fs from 'node:fs';import {rhs} from './src/planar.mjs';const p=JSON.parse(fs.readFileSync('config/spherical.json'));const rows=JSON.parse(fs.readFileSync(0,'utf8'));console.log(JSON.stringify(rows.map(r=>rhs(0,r.state,p,()=>({throttle:r.control[0],gimbalRad:r.control[1]*p.gimbalLimitDeg*Math.PI/180})))));"
r=subprocess.run(['node','--input-type=module','-e',code],input=json.dumps(rows),capture_output=True,text=True,check=True)
actual=np.array(json.loads(r.stdout));expected=np.array([np.array(f(x['state'],x['control'])).ravel()*p2.SCALE for x in rows]);err=float(np.max(np.abs(actual-expected)/(1+np.abs(expected))));assert err<1e-11,err
(root/'results/spherical/symbolic-check.json').write_text(json.dumps(dict(samples=32,maxScaledDerivativeDiscrepancy=err),indent=2)+'\n');print('Symbolic/JS derivative check',err)
