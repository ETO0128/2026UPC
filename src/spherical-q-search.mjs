// Feedback-policy seed for the 15-degree spherical case with an entry burn and a dynamic-pressure cap.
// Usage: node src/spherical-q-search.mjs QMAX_KPA
import fs from 'node:fs';
import {policyPhases, readJSON, replay} from './p2-flight.mjs';

const qMax = Number(process.argv[2]) * 1000;
if (!(qMax > 0)) throw Error('Give the dynamic-pressure cap in kPa');
const contract = readJSON('config/p0-contract.json');
const p = readJSON('config/spherical.json');
const evaluate = (design, step, samples = false, ship = null) =>
  replay(p, contract, design.propellantKg, policyPhases(p, design), {step, samples, ship});
const ok = r => r.passes && r.peakQPa.value <= qMax;

let best = null, trials = 0, passed = 0;
function attempt(design, step) {
  const result = evaluate(design, step);
  trials++;
  if (!ok(result)) return null;
  passed++;
  if (!best || design.propellantKg < best.design.propellantKg) best = {design, result};
  return {design, result};
}
const seeds = [];
for (const mass of [50, 60, 70, 80, 90, 100, 110, 120, 135, 150]) for (const entryAlt of [50, 60, 70]) {
  for (const duration of [20, 30, 45, 60, 80, 100]) for (const bias of [0]) for (const altitude of [6, 9, 12]) {
    const c = attempt({brakeMps2: 1, approachMps: 1.15, propellantKg: mass * 1000, ignitionAltitudeM: altitude * 1000,
      biasDeg: 25, entry: {altitudeM: entryAlt * 1000, durationS: duration, biasDeg: bias}}, 0.15);
    if (c) seeds.push(c);
  }
}
seeds.sort((a, b) => a.design.propellantKg - b.design.propellantKg);
for (const seed of seeds.slice(0, 4)) {
  let current = seed;
  for (const step of [1, 0.5, 0.25, 0.125]) for (let i = 0; i < 4; i++) {
    let next = null;
    for (const da of [-1, 0, 1]) for (const db of [-1, 0, 1]) for (const dd of [-1, 0, 1]) {
      const s = current.design;
      const c = attempt({...s, propellantKg: s.propellantKg - 1400 * step,
        ignitionAltitudeM: s.ignitionAltitudeM + da * 800 * step, biasDeg: s.biasDeg + db * 4 * step,
        entry: {...s.entry, durationS: Math.max(0.5, s.entry.durationS + dd * 4 * step)}}, 0.09);
      if (c && (!next || c.design.propellantKg < next.design.propellantKg)) next = c;
    }
    if (!next || next.design.propellantKg >= current.design.propellantKg) break;
    current = next;
  }
}
const out = {qMaxPa: qMax, model: 'spherical, entry burn + landing burn', trials, passed};
if (best) {
  for (const add of [0, 25, 100, 250, 500, 1000]) {
    const design = {...best.design, propellantKg: best.design.propellantKg + add};
    const fine = evaluate(design, 0.01, true);
    const finer = evaluate(design, 0.005, true, fine.shipLocationM);
    if (ok(fine) && ok(finer)) { out.design = design; out.result = finer; break; }
  }
}
fs.mkdirSync('results/spherical/qcap', {recursive: true});
fs.writeFileSync(`results/spherical/qcap/policy-${qMax / 1000}.json`, JSON.stringify(out) + '\n');
console.log(JSON.stringify({qMax, trials, passed, design: out.design, peakQ: out.result?.peakQPa, ship: out.result?.shipLocationM}));
