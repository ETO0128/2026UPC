const G=9.80665,R=6356766,Ra=287.05287;
// COESA lower-atmosphere lapse rates; geopotential layer boundaries (m).
const H=[0,11000,20000,32000,47000,51000,71000,84852];
const lapse=[-.0065,0,.001,.0028,0,-.0028,-.002];
const temps=[288.15],press=[101325];
for(let i=0;i<7;i++){
 const d=H[i+1]-H[i],b=lapse[i],t=temps[i]+b*d;
 temps.push(t);press.push(b===0?press[i]*Math.exp(-G*d/(Ra*temps[i])):press[i]*(t/temps[i])**(-G/(Ra*b)));
}
export function atmosphere(z){
 const h=R*z/(R+z);
 if(h<0||h>H.at(-1))throw Error('Atmosphere altitude outside implemented range');
 let i=0;while(i<6&&h>=H[i+1])i++;
 const d=h-H[i],b=lapse[i],t=temps[i]+b*d;
 const p=b===0?press[i]*Math.exp(-G*d/(Ra*temps[i])):press[i]*(t/temps[i])**(-G/(Ra*b));
 return {temperatureK:t,pressurePa:p,density:p/(Ra*t)};
}
