import {atmosphere} from './atmosphere.mjs';
export const g0=9.80665, rad=Math.PI/180;
export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export function inertia(p){
 // Dry body uniform cylinder; remaining propellant idealized at COM with negligible pitch inertia.
 return p.dryMassKg*(3*(p.diameterM/2)**2+p.lengthM**2)/12;
}
export function aero(y,p){
 const [x,h,vx,vz,m,theta,omega]=y;
 const e=[Math.cos(theta),Math.sin(theta)],n=[-e[1],e[0]];
 const air=atmosphere(h),u=[vx-p.windMps,vz];
 const axial=u[0]*e[0]+u[1]*e[1],normal=u[0]*n[0]+u[1]*n[1];
 const fa=-.5*air.density*p.axialCd*Math.PI*p.diameterM**2/4*axial*Math.abs(axial);
 let fn=0,torque=0;
 const ds=p.lengthM/p.strips;
 // Equivalent crossflow strip distribution shifted to represent uncertain CP offset.
 for(let i=0;i<p.strips;i++){
  const s=-p.lengthM/2+(i+.5)*ds+p.cpOffsetM;
  const inertialPitchRate = p.geometryModel === 'spherical' ? omega - vx / (6356766 + h) : omega;
  const local=normal+inertialPitchRate*s;
  const f=-.5*air.density*p.crossflowCd*p.diameterM*ds*local*Math.abs(local);
  fn+=f;torque+=s*f;
 }
 return {fx:fa*e[0]+fn*n[0],fz:fa*e[1]+fn*n[1],torque,fa,fn,q:.5*air.density*(u[0]**2+u[1]**2),mach:Math.hypot(...u)/Math.sqrt(1.4*287.05287*air.temperatureK)};
}
export function rhs(t,y,p,command){
 const [x,h,vx,vz,m,theta,omega]=y,a=aero(y,p);
 const c=command(t,y,a),on=c.throttle>0&&m>p.dryMassKg;
 const T=on?p.maxThrustN*clamp(c.throttle,p.minThrottle,1):0;
 const delta=clamp(c.gimbalRad||0,-p.gimbalLimitDeg*rad,p.gimbalLimitDeg*rad);
 // Tail at -L/2 along body axis: positive gimbal generates negative pitch torque.
 const torqueT=-p.lengthM/2*T*Math.sin(delta);
 const forceX = (a.fx + T * Math.cos(theta + delta)) / m;
 const forceH = (a.fz + T * Math.sin(theta + delta)) / m;
 const accelerationPitch = (a.torque + torqueT) / inertia(p);
 if (p.geometryModel === 'spherical') {
  const radius = 6356766 + h;
  return [6356766 / radius * vx, vz,
   forceX - vx * vz / radius,
   -g0 * (6356766 / radius) ** 2 + forceH + vx * vx / radius,
   -T / (p.ispS * g0), omega,
   accelerationPitch + forceX / radius - 2 * vx * vz / (radius * radius)];
 }
 return [vx,vz,forceX,-g0*(6356766/(6356766+h))**2+forceH,-T/(p.ispS*g0),omega,accelerationPitch];
}
export function rk4(t,y,dt,p,control){
 const add=(k,f)=>y.map((v,i)=>v+f*k[i]);
 const k1=rhs(t,y,p,control),k2=rhs(t+dt/2,add(k1,dt/2),p,control),k3=rhs(t+dt/2,add(k2,dt/2),p,control),k4=rhs(t+dt,add(k3,dt),p,control);
 return y.map((v,i)=>v+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6);
}
export function clearance(y,p){
 if (p.geometryModel === 'spherical') {
  const radius = 6356766 + y[1];
  const sine = Math.sin(y[5]);
  const cosine = Math.cos(y[5]);
  const axial = clamp(-radius * sine, -p.lengthM / 2, p.lengthM / 2);
  const normal = clamp(-radius * cosine, -p.diameterM / 2, p.diameterM / 2);
  return Math.hypot(axial * cosine - normal * sine, radius + axial * sine + normal * cosine) - 6356766 - p.deckHeightM;
 }
 return y[1]-p.deckHeightM-p.lengthM/2*Math.abs(Math.sin(y[5]))-p.diameterM/2*Math.abs(Math.cos(y[5]));
}
export function run(p,initial,control,dt=.02,maxTime=500){
 let y=[...initial],t=0,peakQ=0;const samples=[];
 while(t<maxTime){
  const a=aero(y,p);peakQ=Math.max(peakQ,a.q);
  if(samples.length===0||t-samples.at(-1).t>=1) samples.push({t,state:[...y],q:a.q,command:control(t,y,a)});
  // Stop conservatively before dry-mass crossing: no RK stage may burn past dry mass.
  const c=control(t,y,a),flow=c.throttle>0?p.maxThrustN*clamp(c.throttle,p.minThrottle,1)/(p.ispS*g0):0;
  const step=Math.min(dt, maxTime-t, .4*Math.max(y[1]-p.deckHeightM,.001)/Math.max(1,Math.abs(y[3])));
  if(flow>0&&y[4]-p.dryMassKg<=flow*step*1.01)return finish('fuel_exhausted');
  const next=rk4(t,y,step,p,control);
  if(clearance(next,p)<=0){
   const f=clearance(y,p)/(clearance(y,p)-clearance(next,p));
   y=y.map((v,i)=>v+f*(next[i]-v));t+=step*f;return finish('contact');
  }
  y=next;t+=step;
  if(!y.every(Number.isFinite))throw Error('Nonfinite integration state');
 }
 return finish('timeout');
 function finish(event){
  const tilt=Math.abs(Math.atan2(Math.sin(y[5]-Math.PI/2),Math.cos(y[5]-Math.PI/2)))/rad;
  return {event,timeS:t,state:y,peakDynamicPressurePa:peakQ,consumedKg:initial[4]-y[4],remainingKg:y[4]-p.dryMassKg,passesChosenTouchdownCriteria:event==='contact'&&Math.abs(y[2])<=p.touchdown.horizontalMps&&Math.abs(y[3])<=p.touchdown.verticalMps&&tilt<=p.touchdown.tiltDeg&&Math.abs(y[6])/rad<=p.touchdown.pitchRateDegS&&y[4]-p.dryMassKg>=p.reserveKg,samples};
 }
}
export function pitchControl(target,y,a,p,throttle){
 const tau=inertia(p)*(0.5*(target-y[5])-1.2*y[6])-a.torque;
 const T=p.maxThrustN*clamp(throttle,p.minThrottle,1);
 return {throttle,gimbalRad:Math.asin(clamp(-tau/(p.lengthM/2*T),-1,1))};
}

// Local P1 demonstration: regulate descent speed and pitch, without tracking a deck position.
export function terminalDemoControl(y, a, p) {
 const gap = Math.max(.01, y[1] - p.lengthM / 2);
 const desiredSpeed = -Math.min(90, Math.sqrt(2 * 1.5 * gap + 1));
 const acceleration = 4 * (desiredSpeed - y[3]);
 const throttle = Math.max(.001, (y[4] * (g0 + acceleration) - a.fz) / p.maxThrustN);
 return pitchControl(Math.PI / 2, y, a, p, throttle);
}
