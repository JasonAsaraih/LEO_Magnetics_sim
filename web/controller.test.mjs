import test from 'node:test';
import assert from 'node:assert/strict';
import {PIDController,allocateCurrents} from './controller.js';
import {advance,initializeController} from './physics.js';
import {eulerQuaternion,attitudeError,rotate,cross,perpendicular,norm} from './math.js';
const settings={enabled:true,targetRate:[0,0,0],targetQuaternion:[1,0,0,0],targetEulerDegrees:[0,0,0],kp:1e-5,ki:0,kd:2.58e-4,currentLimits:[1,1,1],period:.05,derivativeFilter:.1};
const config={side:.1,mass:1,currents:[.1,.1,.1],turns:[100,100,100],radiusFraction:.4,quaternion:[1,0,0,0],omega:[0,0,0],field:[0,0,40e-6],controller:settings};
function close(a,b,tol=1e-12){a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<tol,`${v} versus ${b[i]}`));}
function simulate(c,duration){const state={y:[...c.quaternion,...c.omega],time:0};initializeController(state,c);while(duration-state.time>1e-10)advance(state,c,duration);return state;}

test('magnetic allocator gives the achievable torque in a rotated cube',()=>{
 const q=eulerQuaternion(.2,.3,.4),requested=[2e-6,-3e-6,5e-6],c={...config,field:[10e-6,-20e-6,40e-6]};
 const currents=allocateCurrents(requested,q,c),m=currents.map((v,i)=>v*c.turns[i]*Math.PI*(c.side*c.radiusFraction)**2);
 close(cross(rotate(q,m),c.field),perpendicular(requested,c.field));
 close(allocateCurrents(requested,q,{...c,field:[0,0,0]}),[0,0,0]);
});
test('zero-rate mode converges to a reachable final pitch',()=>{
 const target=eulerQuaternion(0,.3,0),c={...config,controller:{...settings,targetQuaternion:target}},state=simulate(c,150);
 assert.equal(state.controller.mode,'attitude');
 assert.ok(norm(attitudeError(target,state.y.slice(0,4)))<1e-4);
 assert.ok(norm(state.y.slice(4))<1e-5);
});
test('nonzero-rate mode ignores attitude target and converges on achievable rate',()=>{
 const c={...config,controller:{...settings,targetRate:[.01,0,0],targetQuaternion:eulerQuaternion(2,1,1),kp:1e-4,kd:0}},state=simulate(c,100);
 assert.equal(state.controller.mode,'rate');
 close(state.y.slice(4),[.01,0,0],3e-5);
});
test('current limits, unconstrained peak demand, anti-windup and derivative peaks',()=>{
 const c={...config,controller:{...settings,kp:0,kd:1e-5,currentLimits:[.1,.1,.1]}},pid=new PIDController(c);
 const first=pid.update([1,0,0,0,1,0,0],0);
 close(first,[0,-.1,0]);assert.ok(pid.peaks.required[1]>.1);
 pid.update([1,0,0,0,0,0,0],.05);
 assert.ok(Math.abs(pid.peaks.appliedDerivative[1]-2)<1e-12);
 assert.ok(pid.peaks.requiredDerivative[1]>9);
 const frozen=new PIDController({...config,controller:{...settings,targetQuaternion:eulerQuaternion(0,1,0),ki:1e-4,currentLimits:[.001,.001,.001]}});
 for(let i=0;i<100;i++)frozen.update([1,0,0,0,0,0,0],.05);
 close(frozen.integral,[0,0,0]);
 const reset=new PIDController(c);reset.update([1,0,0,0,0,0,0],0);
 close(reset.peaks.appliedDerivative,[0,0,0]);close(reset.peaks.required,[0,0,0]);
});
test('attitude error takes the short path across ±180 degrees',()=>{
 const current=eulerQuaternion(0,0,179*Math.PI/180),target=eulerQuaternion(0,0,-179*Math.PI/180);
 assert.ok(Math.abs(norm(attitudeError(target,current))-2*Math.PI/180)<1e-12);
 close(attitudeError(target,current),attitudeError(target,current.map(v=>-v)));
});
test('controller timing and derivative peaks are independent of render cadence',()=>{
 const c={...config,omega:[.02,0,0]},one=simulate(c,2),many={y:[...c.quaternion,...c.omega],time:0};
 initializeController(many,c);for(let t=.02;t<2;t+=.02)advance(many,c,t);advance(many,c,2);
 close(one.y,many.y,1e-10);
 assert.equal(one.controller.updates,many.controller.updates);
 close(one.controller.peaks.requiredDerivative,many.controller.peaks.requiredDerivative,1e-8);
});

test('initial browser PID commands agree with the Python calculation tables',async()=>{
 const {readFileSync}=await import('node:fs');
 const cases=JSON.parse(readFileSync(new URL('./controller-reference.json',import.meta.url)));
 for(const {config,expected} of cases){const pid=new PIDController(config);close(pid.update([...config.quaternion,...config.omega],0),expected);}
});
