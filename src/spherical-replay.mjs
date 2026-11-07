import fs from 'node:fs';
import {readJSON,replay,scheduledPhases} from './p2-flight.mjs';
const req=JSON.parse(fs.readFileSync(0,'utf8'));
const p={...readJSON('config/spherical.json'),...req.change};
const result=replay(p,readJSON('config/p0-contract.json'),req.candidate.propellantKg,scheduledPhases(req.candidate.phases),{step:req.stepS??.005,ship:req.candidate.shipLocationM,samples:req.samples??false});
console.log(JSON.stringify(result));
