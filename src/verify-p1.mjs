import fs from 'node:fs';
import assert from 'node:assert/strict';
import {atmosphere} from './atmosphere.mjs';
import {rhs,rk4,aero,inertia,run,pitchControl,rad,g0,clearance} from './planar.mjs';
const p=JSON.parse(fs.readFileSync('config/reference.json','utf8').replace(/^\uFEFF/,''));
const off=()=>({throttle:0,gimbalRad:0}),on=()=>({throttle:1,gimbalRad:0});
const checks=[];
function check(name,error,tolerance,unit='1'){assert.ok(error<=tolerance,`${name}: ${error} > ${tolerance}`);checks.push({name,error,tolerance,unit});}
function integrate(y,params,command,dt,duration){for(let t=0;t<duration-1e-10;t+=dt)y=rk4(t,y,Math.min(dt,duration-t),params,command);return y;}
const vacuum={...p,axialCd:0,crossflowCd:0,cpOffsetM:0};
// Known atmosphere layer bases expressed using geometric altitude.
for(const [H,T,P] of [[0,288.15,101325],[11000,216.65,22632.06],[20000,216.65,5474.889],[32000,228.65,868.0187],[47000,270.65,110.9063]]){
 const a=atmosphere(6356766*H/(6356766-H));check(`Atmosphere pressure H=${H}`,Math.abs(a.pressurePa-P)/P,2e-5);check(`Atmosphere temperature H=${H}`,Math.abs(a.temperatureK-T),1e-8,'K');
}
assert.throws(()=>atmosphere(100000));checks.push({name:'Out-of-range atmosphere rejected',error:0,tolerance:0,unit:'boolean'});
const y0=[0,70000,0,-100,210000,.4,.02],yf=integrate(y0,vacuum,off,.05,20),R=6356766;
const energy=y=>.5*y[3]**2-g0*R*R/(R+y[1]);
check('Vacuum vertical mechanical energy',Math.abs(energy(yf)-energy(y0)),1e-5,'J/kg');
check('Coast mass conservation',Math.abs(yf[4]-y0[4]),1e-9,'kg');
check('Torque-free pitch',Math.abs(yf[5]-(y0[5]+y0[6]*20)),1e-10,'rad');
const burn0=[0,50000,0,0,210000,0,0],duration=10,burn=integrate(burn0,vacuum,on,.02,duration),mExact=burn0[4]-p.maxThrustN*duration/(p.ispS*g0),dv=p.ispS*g0*Math.log(burn0[4]/mExact);
check('Constant-thrust mass history',Math.abs(burn[4]-mExact),1e-6,'kg');
check('Ideal rocket equation horizontal delta-v',Math.abs(burn[2]-dv),1e-7,'m/s');
// gravity affects vertical motion, not the exact horizontal rocket equation comparison.
const gAt70=g0*(R/(R+70000))**2,vz0=p.speedMps*Math.sin(15*rad),vx0=p.speedMps*Math.cos(15*rad);
const analytic={verticalSpeedAtSeparation:vz0,horizontalSpeedAtSeparation:vx0,constantGravityApexM:70000+vz0*vz0/(2*g0),constantGravityApexTimeS:vz0/g0,gravityAt70km:gAt70,initialThrustWeight:p.maxThrustN/(210000*g0),dryThrustWeight:p.maxThrustN/(p.dryMassKg*g0),idealDeltaVWith80tAnd5tReserve:p.ispS*g0*Math.log(210000/135000),constantMassStoppingDistanceAt180t90mps:90**2/(2*(p.maxThrustN/180000-g0))};
// Pure axial / crossflow magnitude and uniform-wind moment.
const air=atmosphere(1000),base=[0,1000,100,0,180000,0,0];let a=aero(base,p);
check('Axial force exact limit',Math.abs(a.fx+.5*air.density*p.axialCd*Math.PI*p.diameterM**2/4*10000),1e-8,'N');
a=aero([0,1000,0,100,180000,0,0],p);
check('Crossflow force exact limit',Math.abs(a.fz+.5*air.density*p.crossflowCd*p.diameterM*p.lengthM*10000),1e-7,'N');
check('Static pressure-center moment',Math.abs(a.torque-p.cpOffsetM*a.fn),1e-6,'N m');
const windA=aero([0,1000,0,0,180000,Math.PI/2,0],{...p,windMps:10});assert.ok(Math.abs(windA.torque)>100);checks.push({name:'Uniform wind produces nonzero offset moment',error:0,tolerance:0,unit:'boolean'});
// Strip convergence against exact rotational drag integral, centered distribution.
const strip=[];const omega=.1,exact=-air.density*p.crossflowCd*p.diameterM*omega*Math.abs(omega)*p.lengthM**4/64;
for(const n of [10,20,40,80,160]){const val=aero([0,1000,0,0,180000,0,omega],{...p,strips:n,cpOffsetM:0}).torque;strip.push({strips:n,relativeError:Math.abs((val-exact)/exact)});}
check('160-strip pure-rotation quadrature',strip.at(-1).relativeError,1e-4);
// Fixed-time smooth dynamics convergence against a finer independently stepped reference.
const smooth0=[0,5000,100,-100,180000,1.4,.01],ref=integrate(smooth0,p,off,.00125,4),time=[];
for(const dt of [.08,.04,.02,.01]){const y=integrate(smooth0,p,off,dt,4);time.push({dt,positionErrorM:Math.hypot(y[0]-ref[0],y[1]-ref[1]),pitchErrorRad:Math.abs(y[5]-ref[5])});}
check('Smooth RK4 position at dt=.01',time.at(-1).positionErrorM,1e-6,'m');
check('RK4 error decreases',time.at(-1).positionErrorM/time[0].positionErrorM,.01);
function terminal(t,y,a){const gap=Math.max(.01,y[1]-p.lengthM/2),desired=-Math.min(90,Math.sqrt(3*gap+1));const throttle=Math.max(.001,(y[4]*(g0+4*(desired-y[3]))-a.fz)/p.maxThrustN);return pitchControl(Math.PI/2,y,a,p,throttle);}
const terminal0=[0,1200,0,-90,180000,Math.PI/2,0],events=[];
for(const dt of [.08,.04,.02,.01,.005]){const r=run(p,terminal0,terminal,dt,100);check(`Contact clearance dt=${dt}`,Math.abs(clearance(r.state,p)),1e-6,'m');assert.ok(r.passesChosenTouchdownCriteria);events.push({dt,timeS:r.timeS,vh:r.state[3],consumedKg:r.consumedKg});}
const perturbed=run(p,[0,1200,0,-90,180000,89*rad,.005],terminal,.01,100);assert.ok(perturbed.passesChosenTouchdownCriteria);
const fuel=run(p,[0,1000,0,0,p.dryMassKg+1,Math.PI/2,0],on,.01,1);assert.equal(fuel.event,'fuel_exhausted');check('Fuel event no dry-mass overshoot',Math.max(0,p.dryMassKg-fuel.state[4]),0,'kg');
const terminalResult=run(p,terminal0,terminal,.005,100);
const result={status:'internal verification, not flight-data validation',checks,analytic,stripConvergence:strip,timeConvergence:time,eventConvergence:events,terminal:{timeS:terminalResult.timeS,state:terminalResult.state,consumedKg:terminalResult.consumedKg},perturbed:{timeS:perturbed.timeS,state:perturbed.state,consumedKg:perturbed.consumedKg},fuelEvent:{event:fuel.event,remainingKg:fuel.remainingKg,meaning:'conservative stop before depletion; not an exact depletion-time solution'}};
fs.writeFileSync('results/p1-verification.json',JSON.stringify(result,null,2)+'\n');
fs.writeFileSync('results/p1-verification-series.json',JSON.stringify({nominal:terminalResult.samples,perturbed:perturbed.samples},null,2)+'\n');
console.log(JSON.stringify(result,null,2));
