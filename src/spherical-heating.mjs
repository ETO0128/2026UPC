// Order-of-magnitude stagnation-point heating along the spherical reference trajectory.
// Sutton-Graves correlation for air: q = k sqrt(rho / Rn) U^3, k = 1.7415e-4 (SI units, W/m^2).
// The effective nose radius of a tumbling or tail-first cylinder is not known, so two values bracket it.
import fs from 'node:fs';
import {atmosphere} from './atmosphere.mjs';

const k = 1.7415e-4;
const radii = [1.0, 3.5];
const selected = JSON.parse(fs.readFileSync('results/spherical/selected.json', 'utf8'));
const trace = selected.replay.trace;
const out = {source: 'results/spherical/selected.json replay trace', correlation: 'Sutton-Graves, k=1.7415e-4 SI', cases: []};
for (const rn of radii) {
  let peak = {value: 0}, load = 0, prev = null;
  for (const row of trace) {
    const [, h, vx, vh] = row.state;
    const rho = atmosphere(h, true).density;
    const u = Math.hypot(vx, vh);
    const flux = k * Math.sqrt(rho / rn) * u ** 3;
    if (flux > peak.value) peak = {value: flux, timeS: row.t, altitudeM: h, speedMps: u};
    if (prev) load += 0.5 * (flux + prev.flux) * (row.t - prev.t);
    prev = {flux, t: row.t};
  }
  out.cases.push({noseRadiusM: rn, peakFluxWm2: peak.value, peakTimeS: peak.timeS, peakAltitudeM: peak.altitudeM,
    peakSpeedMps: peak.speedMps, heatLoadJm2: load});
}
fs.writeFileSync('results/spherical/heating.json', JSON.stringify(out, null, 2) + '\n');
for (const c of out.cases) {
  console.log(`Rn=${c.noseRadiusM} m: peak ${(c.peakFluxWm2 / 1e3).toFixed(1)} kW/m2 at ${c.peakTimeS.toFixed(1)} s, ` +
    `${(c.peakAltitudeM / 1e3).toFixed(1)} km, ${c.peakSpeedMps.toFixed(0)} m/s; load ${(c.heatLoadJm2 / 1e6).toFixed(2)} MJ/m2`);
}
