import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {advance,prepareField,rotate,moments,cross,norm,dot,fieldGrid,fieldLines} from './physics.js';
const fixture=JSON.parse(readFileSync(new URL('./reference.json',import.meta.url)));
function close(actual,expected,tolerance=1e-12){actual.forEach((v,i)=>assert.ok(Math.abs(v-expected[i])<tolerance,`${v} vs ${expected[i]}`));}
test('browser quaternion RK4 agrees with the Python dynamics model',()=>{
 const c=fixture.config,s={y:[...c.quaternion,...c.omega],time:0};advance(s,c,20);close(s.y,fixture.state);assert.ok(Math.abs(s.time-20)<1e-10);
});
test('browser Biot–Savart agrees with Python for rotated, displaced coils',()=>{
 const field=prepareField(fixture.config,fixture.config.quaternion);
 fixture.points.forEach((p,i)=>close(field(p).map((v,j)=>v-fixture.config.field[j]),fixture.field[i],1e-12));
});
test('zero-torque free rotation keeps angular velocity',()=>{
 const c={...fixture.config,quaternion:[1,0,0,0],field:[0,0,0],omega:[0,0,.2]},s={y:[...c.quaternion,...c.omega],time:0};advance(s,c,2);
 close(rotate(s.y.slice(0,4),[1,0,0]),[Math.cos(.4),Math.sin(.4),0],1e-10);close(s.y.slice(4),c.omega);
});
test('work budget keeps pending simulation time instead of skipping physics',()=>{
 const c=fixture.config,s={y:[...c.quaternion,...c.omega],time:0};advance(s,c,100,2);assert.ok(s.time>0&&s.time<100);
});
test('field lines keep 12 seeds and include a uniform external field when coils are off',()=>{
 const c={...fixture.config,currents:[0,0,0],field:[0,0,40e-6]},q=[1,0,0,0],grid=fieldGrid(c,q),paths=fieldLines(c,q,grid);
 assert.equal(paths.length,12);for(const path of paths){assert.ok(path.length>1);for(const p of path)close(p.slice(0,2),path[0].slice(0,2));}
});
