import assert from 'node:assert/strict';
import fs from 'node:fs';
import { spawnSync } from 'node:child_process';
import { readJSON } from './p2-flight.mjs';

const plan = readJSON('results/p3/replanned/nominal.json');
const reference = readJSON('config/reference.json');
const summary = readJSON('results/p3/replanned/summary.json');
const nominal = summary.cases.find(item => item.case === 'nominal');
const actualWindsMps = [-1, -.2, -.1, -.05, -.02, -.01, 0, .01, .02, .05, .1, .2, 1];
const cases = [];

for (const actualWindMps of actualWindsMps) {
  const results = {};
  for (const stepS of [.005, .0025]) {
    const request = {
      mode: 'execute', change: { ...plan.change, windMps: actualWindMps },
      propellantKg: plan.propellantKg, shipLocationM: summary.shipLocationM,
      coastTimeS: plan.coastPrediction.timeS, phases: plan.candidate.phases, stepS,
    };
    const processResult = spawnSync(process.execPath, ['src/p3-replan.mjs'], {
      input: JSON.stringify(request), encoding: 'utf8', maxBuffer: 4 * 1024 * 1024,
    });
    if (processResult.error) throw processResult.error;
    if (processResult.status !== 0) throw new Error(processResult.stderr);
    const result = JSON.parse(processResult.stdout);
    if (actualWindMps === reference.windMps) {
      const expected = nominal.replay[String(stepS)];
      assert.equal(result.passes, expected.passes);
      assert.ok(Math.abs(result.consumedKg - expected.consumedKg) < 1e-6);
    }
    results[String(stepS)] = {
      event: result.event, passes: result.passes, timeS: result.timeS,
      touchdown: result.event === 'contact' ? result.touchdown : null,
      finalState: result.finalState, clearanceM: result.clearanceM,
      failedChecks: Object.keys(result.violations).filter(key => result.violations[key]),
      consumedKg: result.consumedKg, peakQPa: result.peakQPa, peakLoadG: result.peakLoadG,
    };
  }
  cases.push({ actualWindMps, windEstimateErrorMps: actualWindMps - reference.windMps, replay: results });
}

const output = {
  estimatedWindMps: reference.windMps, propellantKg: plan.propellantKg,
  shipLocationM: summary.shipLocationM, scheduleSource: 'results/p3/replanned/nominal.json',
  scope: 'Uniform wind estimation error with the revised nominal commands held fixed from separation',
  cases,
};
fs.writeFileSync('results/p3/wind-estimation.json', JSON.stringify(output, null, 2) + '\n');
for (const item of cases) {
  const result = item.replay['0.0025'];
  console.log(item.actualWindMps, result.event, result.passes,
    result.touchdown === null ? null : result.touchdown.shipErrorM,
    result.failedChecks.join(','));
}
