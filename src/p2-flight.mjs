// Event-resolved separation-to-deck replay using the team's P1 dynamics.
import fs from 'node:fs';
import {aero, clamp, clearance, g0, pitchControl, rad, rk4} from './planar.mjs';

export function readJSON(path) {
  return JSON.parse(fs.readFileSync(path, 'utf8').replace(/^\uFEFF/, ''));
}

const off = () => ({throttle: 0, gimbalRad: 0});
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));

export function initialState(p, contract, propellant) {
  const flight = p.flightPathDeg * rad;
  return [contract.initialConditions.xM, p.altitudeM, p.speedMps * Math.cos(flight),
    p.speedMps * Math.sin(flight), p.dryMassKg + propellant, p.pitchDeg * rad, p.pitchRateRadS];
}

function landingCommand(p, bias, brake, verticalSpeed) {
  return (time, y, a) => {
    const gap = Math.max(0, y[1] - p.deckHeightM - p.lengthM / 2);
    const blend = clamp((gap - 168) / 1800, 0, 1);
    const retrograde = Math.atan2(-y[3], -y[2]);
    const terminalPitch = Math.PI / 2 + clamp(0.015 * y[2], -0.3, 0.3);
    const target = terminalPitch + blend * wrap(retrograde + bias * rad - terminalPitch);
    const desiredVh = -Math.sqrt(2 * brake * gap + verticalSpeed ** 2);
    const gravity = g0 * (6356766 / (6356766 + y[1])) ** 2;
    const throttle = clamp((y[4] * (gravity + 4 * (desiredVh - y[3])) - a.fz) / p.maxThrustN,
      p.minThrottle, 1);
    const command = pitchControl(y[5] + wrap(target - y[5]), y, a, p, throttle);
    command.gimbalRad = clamp(command.gimbalRad, -p.gimbalLimitDeg * rad, p.gimbalLimitDeg * rad);
    return command;
  };
}

export function policyPhases(p, design) {
  const {ignitionAltitudeM, biasDeg, brakeMps2, approachMps, entry, ascent} = design;
  const phases = [];
  if (ascent) {
    const command = (time, y, a) => {
      const u = pitchControl(-Math.PI / 2, y, a, p, 1);
      u.gimbalRad = clamp(u.gimbalRad, -p.gimbalLimitDeg * rad, p.gimbalLimitDeg * rad);
      return u;
    };
    phases.push({name: 'ascent arrest', powered: true, command, duration: ascent.durationS});
  }
  if (entry) {
    phases.push({name: 'pre-entry coast', powered: false, command: off, altitude: entry.altitudeM});
    const command = landingCommand(p, entry.biasDeg, brakeMps2, approachMps);
    phases.push({name: 'entry burn', powered: true, command, duration: entry.durationS});
  }
  phases.push({name: 'coast', powered: false, command: off, altitude: ignitionAltitudeM});
  phases.push({name: 'landing burn', powered: true,
    command: landingCommand(p, biasDeg, brakeMps2, approachMps)});
  return phases;
}

export function scheduledPhases(schedule) {
  const phases = schedule.map(phase => {
    const times = phase.knots;
    function command(t) {
      if (!phase.powered) return off();
      let left = 0;
      let right = times.length - 1;
      while (right - left > 1) {
        const mid = (left + right) >> 1;
        if (t >= times[mid]) left = mid;
        else right = mid;
      }
      const fraction = clamp((t - times[left]) / (times[right] - times[left]), 0, 1);
      return {throttle: phase.throttle[left] + fraction * (phase.throttle[right] - phase.throttle[left]),
        gimbalRad: phase.gimbalRad[left] + fraction * (phase.gimbalRad[right] - phase.gimbalRad[left])};
    }
    return {name: phase.name, powered: phase.powered, command, duration: times.at(-1) - times[0]};
  });
  // Preserve the P2 allowance for contact just after the last control knot.
  if (phases.at(-1).powered) phases.at(-1).duration += 0.5;
  return phases;
}

export function replay(p, contract, propellant, phases, {step = 0.02, samples = false, ship = null, startState = null} = {}) {
  let y = startState === null ? initialState(p, contract, propellant) : [...startState];
  const initial = [...y];
  let time = 0;
  let event = 'schedule_ended';
  let peakQ = {value: 0};
  let peakLoad = {value: 0};
  let maxAltitudeM = y[1];
  let minimumClearanceM = clearance(y, p);
  let maxMach = 0;
  let ignitionCount = 0;
  let powered = false;
  const trace = [];
  const executed = [];
  const deckVelocity = contract.environment.deckVelocityMps;

  function record(phase, force = false) {
    const a = aero(y, p);
    const u = phase.command(time, y, a);
    const thrust = u.throttle * p.maxThrustN;
    const angle = y[5] + u.gimbalRad;
    const load = Math.hypot(a.fx + thrust * Math.cos(angle), a.fz + thrust * Math.sin(angle)) / (y[4] * g0);
    if (a.q > peakQ.value) peakQ = {value: a.q, timeS: time, altitudeM: y[1]};
    if (load > peakLoad.value) peakLoad = {value: load, timeS: time, altitudeM: y[1]};
    maxMach = Math.max(maxMach, a.mach);
    maxAltitudeM = Math.max(maxAltitudeM, y[1]);
    minimumClearanceM = Math.min(minimumClearanceM, clearance(y, p));
    if (samples && (force || !trace.length || time - trace.at(-1).t >= 0.18)) {
      trace.push({t: time, phase: phase.name, state: [...y], throttle: u.throttle,
        gimbalRad: u.gimbalRad, qPa: a.q, mach: a.mach, loadG: load});
    }
  }

  try {
    flight: for (const phase of phases) {
      const start = time;
      const startState = [...y];
      const finish = start + (phase.duration ?? (600 - start));
      if (phase.altitude !== undefined && y[1] < phase.altitude && y[3] < 0) {
        event = 'missed_ignition_altitude';
        break;
      }
      if (phase.powered && !powered) ignitionCount++;
      powered = phase.powered;
      record(phase, true);
      let phaseEnded = false;
      while (time < finish - 1e-9) {
        const dt = Math.min(step, finish - time);
        const next = rk4(time, y, dt, p, phase.command);
        if (!next.every(Number.isFinite)) {
          event = 'nonfinite_state';
          break flight;
        }
        const ignition = phase.altitude !== undefined && y[1] >= phase.altitude && next[1] <= phase.altitude;
        const contact = clearance(next, p) <= 0;
        const depleted = next[4] <= p.dryMassKg;
        if (ignition || contact || depleted) {
          const surface = ignition ? state => state[1] - phase.altitude
            : contact ? state => clearance(state, p) : state => state[4] - p.dryMassKg;
          let lo = 0;
          let hi = dt;
          for (let i = 0; i < 24; i++) {
            const mid = (lo + hi) / 2;
            if (surface(rk4(time, y, mid, p, phase.command)) > 0) lo = mid;
            else hi = mid;
          }
          y = rk4(time, y, hi, p, phase.command);
          time += hi;
          phaseEnded = true;
          if (!ignition) event = contact ? 'contact' : 'fuel_exhausted';
          break;
        }
        y = next;
        time += dt;
        record(phase);
      }
      record(phase, true);
      executed.push({name: phase.name, powered, startS: start, endS: time, startState,
        endState: [...y], consumedKg: startState[4] - y[4]});
      if (event === 'contact' || event === 'fuel_exhausted') break;
      if (phase.altitude !== undefined && !phaseEnded) break;
    }
  } catch (error) {
    if (error.message !== 'Atmosphere altitude outside implemented range') throw error;
    event = 'atmosphere_out_of_range';
  }

  const touchdown = contract.touchdown;
  const target = ship ?? y[0];
  const remainder = y[4] - p.dryMassKg;
  const residual = contract.budget.unusableResidualKg;
  const values = {shipErrorM: Math.abs(y[0] - target), horizontalMps: Math.abs(y[2] - deckVelocity[0]),
    verticalMps: Math.abs(y[3] - deckVelocity[1]), tiltDeg: Math.abs(wrap(y[5] - Math.PI / 2)) / rad,
    pitchRateDegS: Math.abs(y[6]) / rad, reserveKg: remainder - residual};
  const violations = {
    contact: event !== 'contact',
    ship: values.shipErrorM > touchdown.shipTargetToleranceM,
    horizontal: values.horizontalMps > touchdown.horizontalSpeedMaxMps,
    vertical: values.verticalMps > touchdown.verticalSpeedMaxMps,
    tilt: values.tiltDeg > touchdown.tiltMaxDeg,
    pitchRate: values.pitchRateDegS > touchdown.pitchRateMaxDegS,
    reserve: values.reserveKg < touchdown.usableReserveMinKg,
    ignitions: ignitionCount > contract.operations.maximumIgnitions,
  };
  const methane = propellant / (1 + p.oxygenFuelRatio);
  return {event, passes: !Object.values(violations).some(Boolean), violations, propellantKg: propellant,
    methaneKg: methane, liquidOxygenKg: propellant - methane, consumedKg: initial[4] - y[4],
    reserveKg: touchdown.usableReserveMinKg, residualKg: residual,
    extraRemainingKg: remainder - residual - touchdown.usableReserveMinKg, timeS: time,
    initialState: initial, finalState: y, shipLocationM: target, touchdown: values,
    clearanceM: clearance(y, p), minimumClearanceM, ignitionCount, phases: executed,
    peakQPa: peakQ, peakLoadG: peakLoad, maxAltitudeM, maxMach, stepS: step, trace};
}
