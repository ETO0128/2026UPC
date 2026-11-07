// Feedback-policy seeds for spherical-Earth separation-angle cases (coast first, one landing burn).
// Usage: node src/spherical-angle-search.mjs ANGLE_DEG
import fs from 'node:fs';
import {policyPhases, readJSON, replay} from './p2-flight.mjs';

const angle = Number(process.argv[2]);
if (!Number.isFinite(angle)) throw Error('Give the separation flight-path angle in degrees');
const contract = readJSON('config/p0-contract.json');
const p = {...readJSON('config/spherical.json'), flightPathDeg: angle, pitchDeg: angle, upperAtmosphere: 'isothermal'};
const defaults = {brakeMps2: 1.5, approachMps: 1.15};
const evaluate = (design, step, samples = false, ship = null) =>
  replay(p, contract, design.propellantKg, policyPhases(p, design), {step, samples, ship});

let best = null, trials = 0, passed = 0;
function attempt(design, step) {
  const result = evaluate(design, step);
  trials++;
  if (!result.passes) return null;
  passed++;
  if (!best || design.propellantKg < best.design.propellantKg) best = {design, result};
  return {design, result};
}
const seeds = [];
for (const mass of [30, 35, 40, 45, 50, 55, 60, 65, 75, 90]) for (const altitude of [6, 8, 10, 12, 14, 16, 20, 25]) {
  for (const bias of [5, 15, 25, 35, 45]) {
    const c = attempt({...defaults, propellantKg: mass * 1000, ignitionAltitudeM: altitude * 1000, biasDeg: bias}, 0.15);
    if (c) seeds.push(c);
  }
}
seeds.sort((a, b) => a.design.propellantKg - b.design.propellantKg);
for (const seed of seeds.slice(0, 5)) {
  let current = seed;
  for (const step of [1, 0.5, 0.25, 0.125, 0.0625]) for (let i = 0; i < 4; i++) {
    let next = null;
    for (const da of [-1, 0, 1]) for (const db of [-1, 0, 1]) for (const dk of [-1, 0, 1]) {
      const s = current.design;
      const c = attempt({...s, propellantKg: s.propellantKg - 1600 * step,
        ignitionAltitudeM: s.ignitionAltitudeM + da * 1200 * step, biasDeg: s.biasDeg + db * 5 * step,
        brakeMps2: s.brakeMps2 + dk * 0.5 * step}, 0.09);
      if (c && (!next || c.design.propellantKg < next.design.propellantKg)) next = c;
    }
    if (!next || next.design.propellantKg >= current.design.propellantKg) break;
    current = next;
  }
}
const out = {angleDeg: angle, model: 'spherical, isothermal upper atmosphere', trials, passed};
if (best) {
  for (const add of [0, 10, 25, 50, 100, 250, 500]) {
    const design = {...best.design, propellantKg: best.design.propellantKg + add};
    const fine = evaluate(design, 0.01, true);
    const finer = evaluate(design, 0.005, true, fine.shipLocationM);
    if (fine.passes && finer.passes) { out.design = design; out.result = finer; break; }
  }
}
// Unpowered screen: does a coast-first trajectory stay within the extended atmosphere?
const screen = evaluate({...defaults, propellantKg: 40000, ignitionAltitudeM: 8000, biasDeg: 15}, 0.05);
out.coastScreen = {event: screen.event, maxAltitudeM: screen.maxAltitudeM};
fs.mkdirSync('results/spherical/angle', {recursive: true});
fs.writeFileSync(`results/spherical/angle/policy-${angle}.json`, JSON.stringify(out) + '\n');
console.log(JSON.stringify({angle, trials, passed, design: out.design, maxAlt: screen.maxAltitudeM,
  ship: out.result?.shipLocationM, consumed: out.result?.consumedKg}));
