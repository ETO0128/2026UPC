// Replay a collocation control schedule through the original JavaScript P1 model.
import fs from 'node:fs';
import {readJSON, replay, scheduledPhases} from './p2-flight.mjs';

const contract = readJSON('config/p0-contract.json');
const p = readJSON(contract.parameterFile);
const request = JSON.parse(fs.readFileSync(0, 'utf8'));
if (request.flightPathDeg !== undefined) {
  p.flightPathDeg = request.flightPathDeg;
  p.pitchDeg = request.flightPathDeg;
}
const phases = scheduledPhases(request.phases);
const options = {samples: request.samples ?? false, ship: request.shipLocationM};
const coarse = replay(p, contract, request.propellantKg, phases, {...options, step: request.stepS ?? 0.01});
if (request.singleReplay) {
  console.log(JSON.stringify({fine: coarse}));
} else {
  const fine = replay(p, contract, request.propellantKg, phases, {...options, step: 0.005});
  console.log(JSON.stringify({coarse, fine}));
}
