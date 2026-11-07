// Fixed-ship, fixed absolute control replay. No trajectory re-optimization.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import {readJSON,replay} from './p2-flight.mjs';
import {clamp,rad} from './planar.mjs';
const contract=readJSON('config/p0-contract.json'),base=readJSON(contract.parameterFile);
const summary=readJSON('results/p2-p0-v1/summary.json').strategies.single;
const index=Number(summary.source.match(/history\[(\d+)\]/)[1]);
const candidate=readJSON('results/p2-p0-v1/single-collocation.json').history[index].candidate;
function phases(p){
 return candidate.phases.map((phase,index)=>({name:phase.name,powered:phase.powered,duration:phase.knots.at(-1)-phase.knots[0]+(index===candidate.phases.length-1?.5:0),command(t){
  if(!phase.powered)return {throttle:0,gimbalRad:0};
  let l=0,r=phase.knots.length-1;while(r-l>1){const m=(l+r)>>1;if(t>=phase.knots[m])l=m;else r=m;}
  const f=clamp((t-phase.knots[l])/(phase.knots[r]-phase.knots[l]),0,1);
  return {throttle:clamp(phase.throttle[l]+f*(phase.throttle[r]-phase.throttle[l]),p.minThrottle,1),gimbalRad:clamp(phase.gimbalRad[l]+f*(phase.gimbalRad[r]-phase.gimbalRad[l]),-p.gimbalLimitDeg*rad,p.gimbalLimitDeg*rad)};
 }}));
}
const cases=[{parameter:'nominal',value:0,patch:{}}];
const add=(parameter,values,patch)=>{for(const value of values)cases.push({parameter,value,patch:patch(value)});};
add('ispS',[300,326.7,330,333.3,360],v=>({ispS:v}));
add('maxThrustMultiplier',[.8,.99,1,1.01,1.2],v=>({maxThrustN:base.maxThrustN*v}));
add('axialCd',[.4,.792,.8,.808,1.2],v=>({axialCd:v}));
add('crossflowCd',[.8,1.188,1.2,1.212,1.6],v=>({crossflowCd:v}));
add('cpOffsetOverLength',[-.05,0,.0495,.05,.0505],v=>({cpOffsetM:base.lengthM*v}));
add('windMps',[-10,-1,0,1,10],v=>({windMps:v}));
add('flightPathDeg',[0,14.9,15,15.1,30],v=>({flightPathDeg:v,pitchDeg:v}));
add('pitchOffsetDeg',[-2,-.1,0,.1,2],v=>({pitchDeg:base.pitchDeg+v}));
add('initialPitchRateDegS',[-1,-.01,0,.01,1],v=>({pitchRateRadS:v*rad}));
add('minThrottle',[.3,.4,.5],v=>({minThrottle:v}));
add('gimbalLimitDeg',[5,10,15],v=>({gimbalLimitDeg:v}));
// Budget/composition changes do not alter inventory or schedule, intentionally.
add('oxygenFuelRatio',[3,3.5,4],v=>({oxygenFuelRatio:v}));
add('reserveKg',[0,5000,10000],v=>({reserveKg:v}));
const result=[];
for(const [i,c] of cases.entries()){
 const p={...base,...c.patch},k=structuredClone(contract);
 if(c.parameter==='reserveKg')k.touchdown.usableReserveMinKg=c.value;
 const pair=[];
 for(const step of [.01,.005]){
  const r=replay(p,k,candidate.propellantKg,phases(p),{step,ship:summary.shipLocationM});
  const {trace,...small}=r;pair.push(small);
 }
 const failed=Object.entries(pair[1].violations).filter(([,v])=>v).map(([k])=>k);
 result.push({...c,coarse:pair[0],fine:pair[1],failedChecks:failed,isReferenceParameterValue:Object.entries(c.patch).every(([key,val])=>base[key]===val),stepAgreement:pair[0].passes===pair[1].passes&&pair[0].event===pair[1].event&&JSON.stringify(pair[0].violations)===JSON.stringify(pair[1].violations)});
 if(i%10===0)console.log(`${i+1}/${cases.length}: ${c.parameter}=${c.value}: ${pair[1].event}, passes=${pair[1].passes}`);
}
assert.ok(result[0].fine.passes,'Nominal replay must reproduce passing baseline');
for(const c of result.filter(c=>c.parameter==='oxygenFuelRatio'))assert.ok(Math.abs(c.fine.finalState[0]-result[0].fine.finalState[0])<1e-8,'Composition-only change altered dynamics');
const report={scope:'Fixed nominal inventory, absolute time schedule, absolute gimbal commands and ship. Only physical command saturation changes with capability. No optimization or statistical probability.',source:summary.source,nominalInventoryKg:candidate.propellantKg,shipLocationM:summary.shipLocationM,scheduleEndExtensionS:.5,cases:result,counts:{total:result.length,passed:result.filter(c=>c.fine.passes).length,stepDisagreements:result.filter(c=>!c.stepAgreement).length,changedCases:result.filter(c=>!c.isReferenceParameterValue).length,changedCasesPassed:result.filter(c=>!c.isReferenceParameterValue&&c.fine.passes).length}};
fs.mkdirSync('results/p3',{recursive:true});fs.writeFileSync('results/p3/fixed-policy.json',JSON.stringify(report,null,2)+'\n');
const columns=['parameter','value','event','passes','failed','shipErrorM','horizontalMps','verticalMps','tiltDeg','pitchRateDegS','reserveKg','peakQPa','peakLoadG','stepAgreement'];
const lines=result.map(c=>[c.parameter,c.value,c.fine.event,c.fine.passes,c.failedChecks.join('|'),...['shipErrorM','horizontalMps','verticalMps','tiltDeg','pitchRateDegS','reserveKg'].map(k=>c.fine.touchdown[k]),c.fine.peakQPa.value,c.fine.peakLoadG.value,c.stepAgreement].join(','));
fs.writeFileSync('results/p3/fixed-policy.csv',columns.join(',')+'\n'+lines.join('\n')+'\n');console.log(report.counts);
