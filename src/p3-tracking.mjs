import fs from 'node:fs';
import { readJSON, replay, scheduledPhases } from './p2-flight.mjs';
import { clamp, rad, rhs, rk4 } from './planar.mjs';

const contract = readJSON('config/p0-contract.json');
const reference = readJSON(contract.parameterFile);
const mode = process.argv[2];
const request = JSON.parse(fs.readFileSync(0, 'utf8'));
const plan = mode === 'simulate' ? request.reference : request;
const [coast, burn] = scheduledPhases(plan.candidate.phases);
coast.duration += plan.coastTimeS;
const residualCommand = burn.command;
burn.command = (time, state, air) => residualCommand(time - plan.coastTimeS, state, air);
const selected = replay(reference, contract, plan.propellantKg, [coast, burn], { step: .005, ship: plan.shipLocationM });
if (!selected.passes) throw new Error(`The lower-demand reference failed replay: ${JSON.stringify(selected.violations)}`);
const ignition = selected.phases.find(phase => phase.powered);
const duration = selected.timeS - ignition.startS;
const gimbalLimit = reference.gimbalLimitDeg * rad;
const stateScale = request.controller?.stateScale ?? [100, 100, 30, 30, 1000, .2, .2];

function boundedControl(throttle, gimbal) {
  return { throttle: clamp(throttle, reference.minThrottle, 1), gimbalRad: gimbalLimit * clamp(gimbal, -1, 1) };
}

function nominalCommand(t) {
  return burn.command(ignition.startS + t);
}

function linearize() {
  const count = Math.ceil(duration / .1);
  const dt = duration / count;
  const states = [ignition.startState];
  const a = [];
  const b = [];
  const epsilon = .001;
  for (let i = 0; i < count; i++) {
    const time = i * dt;
    const state = states[i];
    const propagate = (initial, command, steps = 4) => {
      let y = initial;
      for (let j = 0; j < steps; j++) y = rk4(time + j * dt / steps, y, dt / steps, reference, command);
      return y;
    };
    states.push(propagate(state, nominalCommand, 20));
    const slope = (upper, lower, width) => upper.map((value, k) => (value - lower[k]) / (width * stateScale[k]));
    a.push(stateScale.map((scale, k) => {
      const upper = [...state];
      const lower = [...state];
      upper[k] += epsilon * scale;
      lower[k] -= epsilon * scale;
      return slope(propagate(upper, nominalCommand), propagate(lower, nominalCommand), 2 * epsilon);
    }));
    const middle = nominalCommand(time + dt / 2);
    const nominalInputs = [middle.throttle, middle.gimbalRad / gimbalLimit];
    b.push(nominalInputs.map((input, k) => {
      const lower = input <= (k === 0 ? reference.minThrottle : -1) + epsilon ? 0 : -epsilon;
      const upper = input >= 1 - epsilon ? 0 : epsilon;
      const command = offset => t => {
        const { throttle, gimbalRad } = nominalCommand(t);
        return boundedControl(throttle + (k === 0 ? offset : 0), gimbalRad / gimbalLimit + (k === 1 ? offset : 0));
      };
      return slope(propagate(state, command(upper)), propagate(state, command(lower)), upper - lower);
    }));
  }
  const derivatives = states.map((state, i) => rhs(i * dt, state, reference, nominalCommand));
  return { dt, stateScale, states, derivatives, a, b };
}

function feedbackPhase(controller) {
  let start;
  const trackingStats = { updates: 0, minThrottle: 1, maxThrottle: 0, maxGimbalDeg: 0 };
  const { dt, states, derivatives, gains } = controller;
  const command = (time, state) => {
    start ??= time;
    const elapsed = time - start;
    const index = Math.min(Math.floor(Math.max(0, elapsed) / dt), gains.length - 1);
    const fraction = clamp(elapsed / dt - index, 0, 1);
    const s = fraction;
    const target = states[index].map((value, k) =>
      (2 * s ** 3 - 3 * s ** 2 + 1) * value + (s ** 3 - 2 * s ** 2 + s) * dt * derivatives[index][k]
      + (-2 * s ** 3 + 3 * s ** 2) * states[index + 1][k] + (s ** 3 - s ** 2) * dt * derivatives[index + 1][k]);
    const error = state.map((value, k) => (value - target[k]) / stateScale[k]);
    const pitchError = state[5] - target[5];
    error[5] = Math.atan2(Math.sin(pitchError), Math.cos(pitchError)) / stateScale[5];
    const [throttleCorrection, gimbalCorrection] = gains[index].map(row => row.reduce((sum, value, k) => sum - value * error[k], 0));
    const { throttle, gimbalRad } = nominalCommand(elapsed);
    const output = boundedControl(throttle + throttleCorrection, gimbalRad / gimbalLimit + gimbalCorrection);
    trackingStats.updates++;
    trackingStats.minThrottle = Math.min(trackingStats.minThrottle, output.throttle);
    trackingStats.maxThrottle = Math.max(trackingStats.maxThrottle, output.throttle);
    trackingStats.maxGimbalDeg = Math.max(trackingStats.maxGimbalDeg, Math.abs(output.gimbalRad) / rad);
    return output;
  };
  return [coast, { ...burn, command, samplePeriodS: .02, trackingStats }];
}

function simulate({ controller, stepS }) {
  const options = { step: stepS, ship: plan.shipLocationM };
  const cases = [-.1, -.05, 0, .05, .1].map(actualWindMps => {
    const params = { ...reference, windMps: actualWindMps };
    const results = {};
    for (const control of ['fixed', 'feedback']) {
      const phases = control === 'fixed' ? [coast, burn] : feedbackPhase(controller);
      const result = replay(params, contract, plan.propellantKg, phases, options);
      results[control] = {
        event: result.event, passes: result.passes, timeS: result.timeS,
        ignitionTimeS: result.phases.find(phase => phase.powered)?.startS ?? null,
        touchdown: result.event === 'contact' ? result.touchdown : null,
        finalState: result.finalState, clearanceM: result.clearanceM,
        consumedKg: result.consumedKg, retainedUsableKg: result.touchdown.reserveKg,
        trackingStats: phases[1].trackingStats ?? null,
        peakQPa: result.peakQPa, peakLoadG: result.peakLoadG,
        failedChecks: result.event === 'contact' ? Object.keys(result.violations).filter(key => result.violations[key]) : [],
      };
    }
    return { actualWindMps, results };
  });
  const benchmark = feedbackPhase(controller)[1];
  benchmark.command(ignition.startS, controller.states[0]);
  const began = performance.now();
  for (let i = 0; i < 10000; i++) {
    const index = i % controller.gains.length;
    benchmark.command(ignition.startS + index * controller.dt, controller.states[index]);
  }
  return { stepS, controlPeriodS: .02, meanFeedbackEvaluationMs: (performance.now() - began) / 10000,
    estimatedWindMps: reference.windMps, stateInformation: 'Exact ground-relative state; no measurement noise or delay',
    propellantKg: plan.propellantKg, shipLocationM: plan.shipLocationM, cases };
}

const result = mode === 'check' ? {passes: selected.passes} : mode === 'linearize' ? linearize() : simulate(request);
console.log(JSON.stringify(result));
