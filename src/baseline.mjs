import fs from 'node:fs';
import assert from 'node:assert/strict';
const cfg=JSON.parse(fs.readFileSync('config/baseline.json','utf8').replace(/^\uFEFF/,''));
const G=9.80665,R=6356766,Ra=287.05287;
// COESA lower-atmosphere lapse rates; geopotential layer boundaries (m).
const H=[0,11000,20000,32000,47000,51000,71000,84852];
const lapse=[-.0065,0,.001,.0028,0,-.0028,-.002];
const temps=[288.15],press=[101325];
for(let i=0;i<7;i++){
 const d=H[i+1]-H[i],b=lapse[i],t=temps[i]+b*d;
 temps.push(t);press.push(b===0?press[i]*Math.exp(-G*d/(Ra*temps[i])):press[i]*(t/temps[i])**(-G/(Ra*b)));
}
function atmosphere(z){
 const h=R*z/(R+z);
 if(h<0||h>H.at(-1))throw Error('Atmosphere altitude outside implemented range');
 let i=0;while(i<6&&h>=H[i+1])i++;
 const d=h-H[i],b=lapse[i],t=temps[i]+b*d;
 const p=b===0?press[i]*Math.exp(-G*d/(Ra*temps[i])):press[i]*(t/temps[i])**(-G/(Ra*b));
 return {temperatureK:t,pressurePa:p,density:p/(Ra*t)};
}
const a=cfg.given,s=cfg.scenario,rad=Math.PI/180;
const end=Math.PI*a.diameterM**2/4,side=a.lengthM*a.diameterM;
function derivative(t,y,control){
 const [x,h,vx,vh,m]=y;
 const {thrust,angle}=control(t,y);
 if(thrust<0||m<a.dryMassKg)throw Error('Invalid thrust or mass');
 const wx=vx-s.windXMps,wz=vh,u=Math.hypot(wx,wz);
 const alignment=u===0?1:Math.max(-1,Math.min(1,(wx*Math.cos(s.axisAngleDeg*rad)+wz*Math.sin(s.axisAngleDeg*rad))/u));
 const area=end*Math.abs(alignment)+side*Math.sqrt(Math.max(0,1-alignment**2));
 const drag=.5*atmosphere(h).density*s.cd*area*u;
 return [vx,vh,(thrust*Math.cos(angle)-drag*wx)/m,-G*(R/(R+h))**2+(thrust*Math.sin(angle)-drag*wz)/m,-thrust/(s.ispS*G)];
}
function step(t,y,dt,control){
 const add=(k,f)=>y.map((v,i)=>v+f*k[i]);
 const k1=derivative(t,y,control),k2=derivative(t+dt/2,add(k1,dt/2),control),k3=derivative(t+dt/2,add(k2,dt/2),control),k4=derivative(t+dt,add(k3,dt),control);
 return y.map((v,i)=>v+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6);
}
function simulate(dt){
 let y=[0,a.separationAltitudeM,a.separationSpeedMps*Math.cos(s.flightPathAngleDeg*rad),a.separationSpeedMps*Math.sin(s.flightPathAngleDeg*rad),a.dryMassKg+s.propellantKg],t=0;
 const contact=a.lengthM/2,control=()=>({thrust:s.thrustN,angle:Math.PI/2});
 while(t<2000){
  const localDt=Math.min(dt,0.8*y[1]/Math.max(1,-y[3]));
  const next=step(t,y,localDt,control);
  if(next[4]<a.dryMassKg)throw Error('Propellant exhausted');
  if(next[1]<=contact){
   const f=(y[1]-contact)/(y[1]-next[1]);
   const hit=y.map((v,i)=>v+f*(next[i]-v));
   return {dt,timeS:t+f*localDt,xM:hit[0],comHeightM:hit[1],vxMps:hit[2],vhMps:hit[3],massKg:hit[4],safeLanding:false};
  }
  y=next;t+=localDt;
 }
 throw Error('No contact event');
}
assert.ok(Math.abs(atmosphere(0).density-1.225)<.001);
for(let i=1;i<7;i++){
 const z=R*H[i]/(R-H[i]);
 assert.ok(Math.abs(atmosphere(z-.01).pressurePa/atmosphere(z+.01).pressurePa-1)<1e-5);
}
const trial=[0,1000,0,0,160000],powered=derivative(0,trial,()=>({thrust:1e6,angle:Math.PI/2}));
assert.ok(Math.abs(powered[4]+1e6/(s.ispS*G))<1e-10);
const coarse=simulate(.5),fine=simulate(.25);
assert.equal(fine.massKg,a.dryMassKg+s.propellantKg);
assert.ok(Math.abs(coarse.timeS-fine.timeS)<.1);
const result={warning:cfg.status,model:'Prescribed upright axis; no rotational dynamics; unpowered impact, NOT landing solution',geometry:{endAreaM2:end,sideAreaM2:side},coriolisInitialUpperBoundMps2:2*7.292115e-5*a.separationSpeedMps,coarse,fine,checks:'Sea-level density, layer continuity, mass flow, coast mass conservation, preliminary time-step comparison passed'};
fs.writeFileSync('results/p1-baseline.json',JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(result,null,2));
