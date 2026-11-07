const G=9.80665,R=6356766,Ra=287.05287;
// COESA lower-atmosphere lapse rates; geopotential layer boundaries (m).
const H=[0,11000,20000,32000,47000,51000,71000,84852];
const lapse=[-.0065,0,.001,.0028,0,-.0028,-.002];
const temps=[288.15],press=[101325];
for(let i=0;i<7;i++){
 const d=H[i+1]-H[i],b=lapse[i],t=temps[i]+b*d;
 temps.push(t);press.push(b===0?press[i]*Math.exp(-G*d/(Ra*temps[i])):press[i]*(t/temps[i])**(-G/(Ra*b)));
}
// Optional isothermal continuation above 84.852 km geopotential. Density there is
// below 7e-6 kg/m^3, so the extension only prevents an artificial altitude cap.
export const UPPER_LIMIT_GEOPOTENTIAL_M=200000;
export function atmosphere(z,extended=false){
 const h=R*z/(R+z);
 if(h<0||h>(extended?UPPER_LIMIT_GEOPOTENTIAL_M:H.at(-1)))throw Error('Atmosphere altitude outside implemented range');
 if(h>H.at(-1)){
  const t=temps.at(-1),p=press.at(-1)*Math.exp(-G*(h-H.at(-1))/(Ra*t));
  return {temperatureK:t,pressurePa:p,density:p/(Ra*t)};
 }
 let i=0;while(i<6&&h>=H[i+1])i++;
 const d=h-H[i],b=lapse[i],t=temps[i]+b*d;
 const p=b===0?press[i]*Math.exp(-G*d/(Ra*temps[i])):press[i]*(t/temps[i])**(-G/(Ra*b));
 return {temperatureK:t,pressurePa:p,density:p/(Ra*t)};
}
