// Search the P1 feedback-policy family and export candidates for P2 collocation.
import fs from 'node:fs';
import {policyPhases, readJSON, replay} from './p2-flight.mjs';

const contract = readJSON('config/p0-contract.json');
const reference = readJSON(contract.parameterFile);
const output = process.argv[2] ?? 'results/p2-p0-v1';
const defaults = {brakeMps2: 1.5, approachMps: 1.15};

function evaluate(p, design, step, samples = false, ship = null) {
  return replay(p, contract, design.propellantKg, policyPhases(p, design), {step, samples, ship});
}

function search(p, starts, steps, vary) {
  let best = null;
  let trials = 0;
  let passed = 0;
  const seeds = [];
  function attempt(design, step) {
    const result = evaluate(p, design, step);
    trials++;
    if (!result.passes) return null;
    passed++;
    const candidate = {design, result};
    if (!best || design.propellantKg < best.design.propellantKg) best = candidate;
    return candidate;
  }

  for (const start of starts) {
    const candidate = attempt(start, 0.15);
    if (candidate) seeds.push(candidate);
  }
  seeds.sort((a, b) => a.design.propellantKg - b.design.propellantKg);
  for (const seed of seeds.slice(0, 5)) {
    let current = seed;
    for (const step of steps) {
      for (let i = 0; i < 4; i++) {
        let next = null;
        for (const choice of vary(current.design, step)) {
          const candidate = attempt(choice, 0.09);
          if (candidate && (!next || choice.propellantKg < next.design.propellantKg)) next = candidate;
        }
        if (!next || next.design.propellantKg >= current.design.propellantKg) break;
        current = next;
      }
    }
  }
  if (!best) return {trials, passed};
  const design = {...best.design};
  // The search mesh is coarser than replay. Add a documented numerical buffer
  // only if the candidate misses reserve after the independent finer replay.
  for (const add of [0, 10, 25, 50, 100, 250, 500]) {
    design.propellantKg = best.design.propellantKg + add;
    const fine = evaluate(p, design, 0.01, true);
    const finer = evaluate(p, design, 0.005, true, fine.shipLocationM);
    if (fine.passes && finer.passes) {
      return {trials, passed, design, result: finer, convergence: {coarserStepS: 0.01,
        coarserTimeS: fine.timeS, coarserFinalState: fine.finalState, coarserConsumedKg: fine.consumedKg}};
    }
  }
  return {trials, passed, failedFineReplay: best.design};
}

function single(p, nominal = null) {
  const starts = [];
  for (const mass of [40, 45, 50, 55, 60, 65, 75, 90]) {
    for (const altitude of [8, 10, 12, 14, 16, 20, 25, 30]) {
      for (const bias of [5, 15, 25, 35, 45]) {
        starts.push({...defaults, propellantKg: mass * 1000, ignitionAltitudeM: altitude * 1000, biasDeg: bias});
      }
    }
  }
  if (nominal) starts.push(nominal);
  return search(p, starts, [1, 0.5, 0.25, 0.125, 0.0625], (s, step) => {
    const choices = [];
    for (const altitude of [-1, 0, 1]) for (const bias of [-1, 0, 1]) {
      for (const brake of [-1, 0, 1]) {
        choices.push({...s, propellantKg: s.propellantKg - 1600 * step,
          ignitionAltitudeM: s.ignitionAltitudeM + altitude * 1200 * step,
          biasDeg: s.biasDeg + bias * 5 * step, brakeMps2: s.brakeMps2 + brake * 0.5 * step});
      }
    }
    return choices;
  });
}

function double(p, singleDesign) {
  const starts = [];
  for (const mass of [-6, -3, 0, 3, 8, 16]) for (const altitude of [16, 20, 25, 35, 45]) {
    for (const duration of [1, 3, 7]) for (const bias of [0, 20, 40]) {
      starts.push({...singleDesign, propellantKg: singleDesign.propellantKg + mass * 1000,
        entry: {altitudeM: altitude * 1000, durationS: duration, biasDeg: bias}});
    }
  }
  return search(p, starts, [1, 0.5, 0.25, 0.125, 0.0625], (s, step) => {
    const choices = [];
    for (const altitude of [-1, 0, 1]) for (const bias of [-1, 0, 1]) {
      for (const duration of [-1, 0, 1]) {
        choices.push({...s, propellantKg: s.propellantKg - 1400 * step,
          ignitionAltitudeM: s.ignitionAltitudeM + altitude * 800 * step,
          biasDeg: s.biasDeg + bias * 4 * step,
          entry: {...s.entry, durationS: Math.max(0.05, s.entry.durationS + duration * step)}});
      }
    }
    return choices;
  });
}

function steepAngle(p, nominal) {
  // A focused near-cap warm start supplements the coarse ascent grid.
  const starts = [{...nominal, propellantKg: 70000, ignitionAltitudeM: 7800,
    biasDeg: 44, brakeMps2: 0.55, ascent: {durationS: 34}}];
  for (const mass of [55, 60, 65, 70, 73, 75, 78]) for (const duration of [27, 30, 33, 36]) {
    for (const altitude of [5, 6.5, 8, 10, 12]) for (const bias of [15, 23, 31]) {
      starts.push({...nominal, propellantKg: mass * 1000, ignitionAltitudeM: altitude * 1000,
        biasDeg: bias, ascent: {durationS: duration}});
    }
  }
  return search(p, starts, [1, 0.5, 0.25, 0.125, 0.0625], (s, step) => {
    const choices = [];
    for (const altitude of [-1, 0, 1]) for (const bias of [-1, 0, 1]) {
      for (const duration of [-1, 0, 1]) {
        choices.push({...s, propellantKg: s.propellantKg - 1600 * step,
          ignitionAltitudeM: s.ignitionAltitudeM + altitude * 800 * step,
          biasDeg: s.biasDeg + bias * 4 * step, ascent: {durationS: s.ascent.durationS + duration * step}});
      }
    }
    return choices;
  });
}

const results = {contractVersion: contract.version, reference, single: single(reference)};
console.log('15-degree single burn', results.single.design ?? results.single);
if (results.single.design) {
  results.double = double(reference, results.single.design);
  console.log('15-degree two burns', results.double.design ?? results.double);
}
const level = {...reference, flightPathDeg: 0, pitchDeg: 0};
results.level = single(level, results.single.design);
console.log('0-degree single burn', results.level.design ?? results.level);
const steep = {...reference, flightPathDeg: 30, pitchDeg: 30};
results.steepCoastScreen = evaluate(steep, results.single.design ?? {
  ...defaults, propellantKg: 80000, ignitionAltitudeM: 12000, biasDeg: 15,
}, 0.005);
console.log('30-degree coast screen', results.steepCoastScreen.event, results.steepCoastScreen.timeS);
if (results.single.design) {
  results.steep = steepAngle(steep, results.single.design);
  console.log('30-degree ascent arrest and landing', results.steep.design ?? results.steep);
}
fs.mkdirSync(output, {recursive: true});
fs.writeFileSync(`${output}/policy-search.json`, JSON.stringify(results) + '\n');
