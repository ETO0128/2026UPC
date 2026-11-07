import fs from 'node:fs';
import { readJSON, replay, scheduledPhases } from './p2-flight.mjs';

const contract = readJSON('config/p0-contract.json');
const reference = readJSON(contract.parameterFile);
const request = JSON.parse(fs.readFileSync(0, 'utf8'));
const params = { ...reference, ...request.change };
const options = { step: request.stepS ?? .005, samples: request.samples ?? false, ship: request.shipLocationM };
let result;
if (request.mode === 'observe') {
  const coast = { name: 'coast', powered: false, command: () => ({ throttle: 0, gimbalRad: 0 }),
    altitude: request.observationAltitudeM, duration: 400 };
  result = replay(params, contract, request.propellantKg, [coast], { ...options, startState: request.startState ?? null });
} else {
  const phases = scheduledPhases(request.phases);
  if (request.coastTimeS !== undefined) {
    if (phases[0].powered) throw new Error('A full-flight corrective schedule must start with coast');
    phases[0].duration += request.coastTimeS;
    for (const phase of phases.slice(1)) {
      const command = phase.command;
      phase.command = (time, state, air) => command(time - request.coastTimeS, state, air);
    }
  }
  result = replay(params, contract, request.propellantKg, phases, { ...options, startState: request.startState ?? null });
}
console.log(JSON.stringify(result));
