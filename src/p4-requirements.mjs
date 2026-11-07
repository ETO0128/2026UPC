import fs from 'node:fs';
const read = file => JSON.parse(fs.readFileSync(file, 'utf8').replace(/^\uFEFF/, ''));
const params = read('config/reference.json');
const record = read('results/p3/safety/summary.json').p2;
const selected = read('results/p2-p0-v1/summary.json').strategies.single;
const index = Number(selected.source.match(/history\[(\d+)\]/)[1]);
const candidate = read('results/p2-p0-v1/single-collocation.json').history[index].candidate;
let maxGimbalRateDegS = 0;
let maxThrottleRatePerS = 0;
for (const phase of candidate.phases.filter(phase => phase.powered)) {
  for (let i = 1; i < phase.knots.length; i++) {
    const duration = phase.knots[i] - phase.knots[i - 1];
    maxGimbalRateDegS = Math.max(maxGimbalRateDegS, Math.abs(phase.gimbalRad[i] - phase.gimbalRad[i - 1]) / duration * 180 / Math.PI);
    maxThrottleRatePerS = Math.max(maxThrottleRatePerS, Math.abs(phase.throttle[i] - phase.throttle[i - 1]) / duration);
  }
}
const radius = 6356766;
const angle = selected.shipLocationM / radius;
const result = {
  interpretation: 'Requirements and approximation scales for the nominal local-plane trajectory',
  requiredDynamicPressurePa: record.peaks.qPa.value,
  requiredSpecificForceG: record.peaks.loadG.value,
  maximumExternalAeroMomentNm: record.peaks.aeroTorqueAbsNm.value,
  maximumNetPitchMomentNm: record.peaks.netTorqueAbsNm.value,
  maximumAvailableThrustMomentNm: params.lengthM / 2 * params.maxThrustN * Math.sin(params.gimbalLimitDeg * Math.PI / 180),
  maximumWithinBurnGimbalRateDegS: maxGimbalRateDegS,
  maximumWithinBurnThrottleRatePerS: maxThrottleRatePerS,
  actuatorNote: 'Rates are slopes inside control phases. Ignition discontinuities require separate startup modeling.',
  downrangeM: selected.shipLocationM,
  centralAngleDeg: angle * 180 / Math.PI,
  tangentSurfaceOffsetM: radius * (1 - Math.cos(angle)),
  initialCoriolisAccelerationUpperBoundMps2: 2 * 7.292115e-5 * params.speedMps,
  geometryNote: 'Offset estimates the scale of curvature relative to a fixed tangent plane. It is not a corrected trajectory or a landing-error prediction.'
};
fs.writeFileSync('results/p3/design-requirements.json', JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
