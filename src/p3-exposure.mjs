// Flight-environment requirements; no material limits or wall heat-transfer model.
import fs from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {spawnSync} from 'node:child_process';
import {atmosphere} from './atmosphere.mjs';
import {g0, rad} from './planar.mjs';
import {readJSON, replay, scheduledPhases} from './p2-flight.mjs';

const peakKeys = ['qPa', 'loadG', 'aeroForceN', 'normalForceAbsN', 'aeroTorqueAbsNm',
  'thrustTorqueAbsNm', 'netTorqueAbsNm', 'flowKineticFluxWm2', 'airSpeedMps'];

export function createExposureRecorder(params) {
  const peaks = {};
  const phasePeaks = {};
  const trace = [];
  let last;
  function record({timeS, state, phase, powered, aero, control, loadG}) {
    const air = atmosphere(state[1]);
    const ux = state[2] - params.windMps;
    const uz = state[3];
    const speed = Math.hypot(ux, uz);
    const theta = state[5];
    const axial = ux * Math.cos(theta) + uz * Math.sin(theta);
    const normal = -ux * Math.sin(theta) + uz * Math.cos(theta);
    const thrust = control.throttle * params.maxThrustN;
    const thrustTorque = -params.lengthM / 2 * thrust * Math.sin(control.gimbalRad);
    const netTorque = aero.torque + thrustTorque;
    const row = {timeS, phase, powered: Number(powered), altitudeM: state[1], massKg: state[4],
      vxMps: state[2], vzMps: state[3], pitchDeg: theta / rad, pitchRateDegS: state[6] / rad,
      airSpeedMps: speed, axialAirMps: axial, normalAirMps: normal,
      incidenceDeg: Math.atan2(Math.abs(normal), Math.abs(axial)) / rad,
      qPa: aero.q, mach: aero.mach, loadG,
      aeroForceN: Math.hypot(aero.fx, aero.fz), axialForceN: aero.fa, normalForceN: aero.fn,
      normalForceAbsN: Math.abs(aero.fn), thrustN: thrust,
      aeroLoadG: Math.hypot(aero.fx, aero.fz) / (state[4] * g0), thrustLoadG: thrust / (state[4] * g0),
      axialProperG: (aero.fa + thrust * Math.cos(control.gimbalRad)) / (state[4] * g0),
      normalProperG: (aero.fn + thrust * Math.sin(control.gimbalRad)) / (state[4] * g0),
      aeroTorqueNm: aero.torque, thrustTorqueNm: thrustTorque, netTorqueNm: netTorque,
      aeroTorqueAbsNm: Math.abs(aero.torque), thrustTorqueAbsNm: Math.abs(thrustTorque),
      netTorqueAbsNm: Math.abs(netTorque),
      flowKineticFluxWm2: .5 * air.density * speed ** 3,
      throttle: control.throttle, gimbalDeg: control.gimbalRad / rad};
    phasePeaks[phase] ??= {};
    for (const key of peakKeys) {
      if (!peaks[key] || row[key] > peaks[key].value) peaks[key] = {value: row[key], at: row};
      if (!phasePeaks[phase][key] || row[key] > phasePeaks[phase][key].value) {
        phasePeaks[phase][key] = {value: row[key], at: row};
      }
    }
    if (!trace.length || phase !== trace.at(-1).phase || timeS - trace.at(-1).timeS >= .2 - 1e-10) trace.push(row);
    last = row;
  }
  function finish() {
    if (!last) throw new Error('No flight records were collected');
    // Include the actual peak points in the plotted/archived decimated trace.
    const rows = [...trace, last, ...Object.values(peaks).map(peak => peak.at)];
    const unique = new Map(rows.map(row => [`${row.timeS}:${row.phase}`, row]));
    return {peaks, phasePeaks, trace: [...unique.values()].sort((a, b) => a.timeS - b.timeS)};
  }
  return {record, finish};
}

function saveCsv(filename, rows) {
  const keys = Object.keys(rows[0]);
  fs.writeFileSync(filename, keys.join(',') + '\n' + rows.map(row => keys.map(key => row[key]).join(',')).join('\n') + '\n');
}

function main() {
  const contract = readJSON('config/p0-contract.json');
  const params = readJSON(contract.parameterFile);
  const selected = readJSON('results/p2-p0-v1/summary.json');
  const p2 = selected.strategies[selected.nominalSelected];
  const match = /^([\w-]+-collocation\.json) history\[(\d+)\]$/.exec(p2.source);
  if (!match) throw new Error('Unrecognized selected P2 source');
  const candidate = readJSON(`results/p2-p0-v1/${match[1]}`).history[Number(match[2])].candidate;
  const recorder = createExposureRecorder(params);
  const stepS = .0025;
  const baseline = replay(params, contract, p2.propellantKg, scheduledPhases(candidate.phases), {
    step: stepS, ship: p2.shipLocationM, onRecord: recorder.record,
  });
  const reference = readJSON('results/p3/tracking/reference.json');
  const controller = readJSON('results/p3/tracking/controller.json');
  const processResult = spawnSync(process.execPath, ['src/p3-tracking.mjs', 'exposure'], {
    input: JSON.stringify({reference, controller, stepS}), encoding: 'utf8', maxBuffer: 16 * 1024 * 1024,
  });
  if (processResult.status !== 0) throw new Error(processResult.stderr || `Exposure replay exited ${processResult.status}`);
  const cases = JSON.parse(processResult.stdout);
  const nominal = cases.find(item => item.actualWindMps === 0);
  const directory = 'results/p3/safety';
  fs.mkdirSync(directory, {recursive: true});
  saveCsv(`${directory}/tracking-nominal.csv`, nominal.trace);
  const baselineExposure = recorder.finish();
  saveCsv(`${directory}/p2-nominal.csv`, baselineExposure.trace);
  const stripTrace = ({trace, ...rest}) => rest;
  const output = {stepS, controlPeriodS: .02,
    interpretation: {loadG: '|F_aero + F_thrust| / (m g0), non-gravitational specific force at COM',
      normalForceN: 'Integrated crossflow strips, including local rotational velocity',
      torques: 'External pitching moments about COM; not internal structural bending moments',
      flowKineticFluxWm2: '0.5 rho |v_air|^3, free-stream kinetic-energy flux; not wall heat flux',
      thresholds: contract.safety},
    p2: {event: baseline.event, passes: baseline.passes, propellantKg: p2.propellantKg,
      timeS: baseline.timeS, ignitionTimeS: baseline.phases.find(phase => phase.powered).startS,
      ...stripTrace(baselineExposure)},
    trackingPropellantKg: reference.propellantKg, cases: cases.map(stripTrace)};
  fs.writeFileSync(`${directory}/summary.json`, JSON.stringify(output, null, 2) + '\n');
  for (const [label, entry] of [['P2 nominal', output.p2], ...output.cases.map(item => [`wind ${item.actualWindMps}`, item])]) {
    console.log(label, JSON.stringify(Object.fromEntries(Object.entries(entry.peaks).map(([key, peak]) =>
      [key, {value: peak.value, timeS: peak.at.timeS, altitudeM: peak.at.altitudeM}]))));
  }
  if (!baseline.passes || cases.some(item => !item.passes)) throw new Error('An existing landing case changed outcome');
}

if (process.argv[1] && import.meta.url === pathToFileURL(path.resolve(process.argv[1])).href) main();
