// Replay a collocation control schedule through the original JavaScript P1 model.
import fs from 'node:fs';
import {readJSON, replay} from './p2-flight.mjs';

const contract = readJSON('config/p0-contract.json');
const p = readJSON(contract.parameterFile);
const request = JSON.parse(fs.readFileSync(0, 'utf8'));
if (request.flightPathDeg !== undefined) {
  p.flightPathDeg = request.flightPathDeg;
  p.pitchDeg = request.flightPathDeg;
}
const phases = request.phases.map(phase => {
  const times = phase.knots;
  function command(t) {
    if (!phase.powered) return {throttle: 0, gimbalRad: 0};
    let left = 0;
    let right = times.length - 1;
    while (right - left > 1) {
      const mid = (left + right) >> 1;
      if (t >= times[mid]) left = mid;
      else right = mid;
    }
    const fraction = Math.max(0, Math.min(1, (t - times[left]) / (times[right] - times[left])));
    return {throttle: phase.throttle[left] + fraction * (phase.throttle[right] - phase.throttle[left]),
      gimbalRad: phase.gimbalRad[left] + fraction * (phase.gimbalRad[right] - phase.gimbalRad[left])};
  }
  return {name: phase.name, powered: phase.powered, command,
    duration: times.at(-1) - times[0]};
});
if (phases.at(-1).powered) phases.at(-1).duration += 0.5;
const options = {samples: request.samples ?? false, ship: request.shipLocationM};
const coarse = replay(p, contract, request.propellantKg, phases, {...options, step: request.stepS ?? 0.01});
if (request.singleReplay) {
  console.log(JSON.stringify({fine: coarse}));
} else {
  const fine = replay(p, contract, request.propellantKg, phases, {...options, step: 0.005});
  console.log(JSON.stringify({coarse, fine}));
}
