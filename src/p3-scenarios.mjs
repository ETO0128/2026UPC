import fs from 'node:fs';
import { rad, rk4, run, terminalDemoControl } from './planar.mjs';
import { readJSON, replay, scheduledPhases } from './p2-flight.mjs';

const contract = readJSON('config/p0-contract.json');
const reference = readJSON(contract.parameterFile);
const scenarios = contract.sensitivity;
const p2Directory = 'results/p2-p0-v1';
const p2Summary = readJSON(`${p2Directory}/summary.json`);
const selected = p2Summary.strategies[p2Summary.nominalSelected];
const source = /^([\w-]+-collocation\.json) history\[(\d+)\]$/.exec(selected?.source ?? '');
if (p2Summary.contractVersion !== contract.version || !selected?.verified || !source) {
  throw new Error('P3 requires an accepted P2 control schedule for the current reference contract');
}
const { candidate } = readJSON(`${p2Directory}/${source[1]}`).history[Number(source[2])];
const p2Phases = scheduledPhases(candidate.phases);
const p2Options = { step: selected.stepS, ship: selected.shipLocationM };

function summarizeFullFlight(factor, value, params) {
  const result = replay(params, contract, selected.propellantKg, p2Phases, p2Options);
  return {
    factor, value, event: result.event, passes: result.passes, timeS: result.timeS,
    touchdown: result.event === 'contact' ? result.touchdown : null,
    signedShipErrorM: result.event === 'contact' ? result.finalState[0] - p2Options.ship : null,
    failedTouchdownChecks: result.event === 'contact'
      ? Object.keys(result.violations).filter(key => result.violations[key]) : [],
    finalState: result.finalState, clearanceM: result.clearanceM,
    consumedKg: result.consumedKg, peakQPa: result.peakQPa, peakLoadG: result.peakLoadG,
  };
}

const terminalInitial = [0, 1200, 0, -90, 180000, Math.PI / 2, 0];
const terminalStepS = 0.005;
const nominal = run(reference, terminalInitial, (_t, state, air) => terminalDemoControl(state, air, reference), terminalStepS, 100);
if (nominal.event !== 'contact') throw new Error('The nominal terminal demonstration did not reach the deck');
const shipXM = nominal.state[0];
const shipToleranceM = contract.touchdown.shipTargetToleranceM;

function summarizeTerminal(factor, value, result) {
  const [x, , vx, vh, , pitch, pitchRate] = result.state;
  const shipErrorM = result.event === 'contact' ? x - shipXM : null;
  return {
    factor, value, event: result.event,
    shipErrorM, vxMps: vx, vhMps: vh, pitchDeg: pitch / rad, pitchRateDegS: pitchRate / rad,
    consumedKg: result.consumedKg, remainingKg: result.remainingKg,
    peakDynamicPressurePa: result.peakDynamicPressurePa,
    passesSpeedAttitudeAndReserve: result.passesChosenTouchdownCriteria,
    capturesFixedShip: result.passesChosenTouchdownCriteria && Math.abs(shipErrorM) <= shipToleranceM,
  };
}

const terminal = [summarizeTerminal('nominal', null, nominal)];
const fullFlight = [summarizeFullFlight('nominal', null, reference)];
if (!fullFlight[0].passes || Math.abs(fullFlight[0].consumedKg - selected.consumedKg) > 1e-6) {
  throw new Error('The nominal P3 replay does not reproduce the selected P2 result');
}
const changes = [
  ['windMps', scenarios.windMps, reference.windMps],
  ['pitchOffsetDeg', scenarios.pitchOffsetDeg, 0],
  ['initialPitchRateDegS', scenarios.initialPitchRateDegS, 0],
];
for (const [factor, values, nominalValue] of changes) {
  for (const value of values.filter(value => value !== nominalValue)) {
    const params = factor === 'windMps' ? { ...reference, windMps: value } : reference;
    const initial = [...terminalInitial];
    if (factor === 'pitchOffsetDeg') initial[5] += value * rad;
    if (factor === 'initialPitchRateDegS') initial[6] += value * rad;
    const result = run(params, initial, (_t, state, air) => terminalDemoControl(state, air, params), terminalStepS, 100);
    terminal.push(summarizeTerminal(factor, value, result));
    fullFlight.push(summarizeFullFlight(factor, value, {
      ...params,
      pitchDeg: reference.pitchDeg + (factor === 'pitchOffsetDeg' ? value : 0),
      pitchRateRadS: reference.pitchRateRadS + (factor === 'initialPitchRateDegS' ? value * rad : 0),
    }));
  }
}

const coastStepS = 0.02;
const off = () => ({ throttle: 0, gimbalRad: 0 });
function coastCoverage(angleDeg) {
  const angle = angleDeg * rad;
  let state = [0, reference.altitudeM, reference.speedMps * Math.cos(angle),
    reference.speedMps * Math.sin(angle), reference.dryMassKg + reference.propellantKg,
    angle, reference.pitchRateRadS];
  for (let time = 0; time < 600; time += coastStepS) {
    if (state[3] <= 0) return { angleDeg, status: 'apexWithinModel', timeS: time, altitudeM: state[1] };
    try {
      state = rk4(time, state, coastStepS, reference, off);
    } catch (error) {
      if (error.message !== 'Atmosphere altitude outside implemented range') throw error;
      return { angleDeg, status: 'outsideAtmosphereModel', lastSupportedTimeS: time, lastSupportedAltitudeM: state[1] };
    }
  }
  throw new Error(`Coast did not reach an apex or the model boundary at ${angleDeg} degrees`);
}

const output = {
  scope: 'Deterministic perturbations with fixed P2 commands and ship position, local terminal comparisons, and unpowered ascent coverage.',
  fixedShipReplay: { scheduleSource: selected.source, propellantKg: selected.propellantKg,
    shipLocationM: p2Options.ship, stepS: p2Options.step, cases: fullFlight },
  terminalInitial, terminalStepS, shipXM, shipToleranceM, terminal,
  coastStepS, coast: scenarios.flightPathDeg.map(coastCoverage),
};
fs.writeFileSync('results/p3-scenarios.json', JSON.stringify(output, null, 2) + '\n');
