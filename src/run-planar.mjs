import fs from 'node:fs';
import assert from 'node:assert/strict';
import {aero,rhs,inertia,run,pitchControl,terminalDemoControl,rad,g0} from './planar.mjs';
const p=JSON.parse(fs.readFileSync('config/reference.json','utf8').replace(/^\uFEFF/,''));
const off=()=>({throttle:0,gimbalRad:0});
// Physical sign/limit checks: axial/crossflow deceleration, rotational damping, tail-gimbal torque.
let y=[0,1000,100,0,180000,0,0],a=aero(y,p);
assert.ok(a.fx<0&&Math.abs(a.fz)<1e-10&&Math.abs(a.torque)<1e-10);
y=[0,1000,0,100,180000,0,0];a=aero(y,p);assert.ok(a.fz<0&&a.torque<0);
const symmetric={...p,cpOffsetM:0};y=[0,1000,0,0,180000,0,.1];assert.ok(aero(y,symmetric).torque<0);
y=[0,1000,0,0,180000,0,0];let d=rhs(0,y,p,()=>({throttle:1,gimbalRad:.01}));assert.ok(d[6]<0);assert.ok(Math.abs(d[4]+p.maxThrustN/(p.ispS*g0))<1e-9);
for(const theta of [0,.3,1.57]){y=[0,1000,100,30,180000,theta,.03];a=aero(y,symmetric);assert.ok(a.fx*100+a.fz*30+a.torque*.03<=1e-8);}
// Terminal demonstrator: vertical reference, feedback throttle and torque compensation.
const terminal=(_t,y,a)=>terminalDemoControl(y,a,p);
const initial=[0,1200,0,-90,180000,Math.PI/2,0];
const fine=run(p,initial,terminal,.01,100),coarse=run(p,initial,terminal,.02,100);
assert.equal(fine.event,'contact');assert.ok(Math.abs(fine.state[3]-coarse.state[3])<.1);
assert.ok(fine.passesChosenTouchdownCriteria);
const perturbed=run(p,[...initial.slice(0,5),89*rad,.005],terminal,.01,100);
assert.equal(perturbed.event,'contact');
assert.ok(Math.abs(perturbed.state[5]-Math.PI/2)<rad);
const initialFull=[0,p.altitudeM,p.speedMps*Math.cos(p.flightPathDeg*rad),p.speedMps*Math.sin(p.flightPathDeg*rad),p.dryMassKg+p.propellantKg,p.pitchDeg*rad,p.pitchRateRadS];
// Deliberately simple scheduled burn, for integration demonstration only; not optimized guidance.
const full=run(p,initialFull,(t,y,a)=>y[1]<15000&&y[3]<0?pitchControl(Math.PI/2,y,a,p,1):off(),.02,600);
const summarize=r=>{const {samples,...rest}=r;return rest;};
fs.writeFileSync('results/planar-summary.json',JSON.stringify({checks:'Axial/crossflow signs, damping, gimbal sign, mass flow, dissipativity and terminal step comparison passed',reference:p.description,terminalFine:summarize(fine),terminalCoarse:summarize(coarse),terminalPerturbed:summarize(perturbed),fullScheduled:summarize(full),warning:'Terminal demonstration is not a complete separation-to-landing solution. Full scheduled burn is not optimized.'},null,2)+'\n');
fs.writeFileSync('results/planar-trajectory.json',JSON.stringify({terminal:fine.samples,fullScheduled:full.samples},null,2)+'\n');
console.log(fs.readFileSync('results/planar-summary.json','utf8'));
