/** Sampled PID torque control and magnetic-current allocation, all in SI units.
 * Edit desiredTorque() to implement another controller using the same sensor
 * inputs, allocator, limits, and peak logs. Angles are radians; rates rad/s.
 */
import {add,scale,norm,cross,dot,rotate,conjugate,attitudeError,perpendicular,quaternionEuler} from './math.js';

export function desiredTorque({mode,angleError,rateError,integral,derivative,gains}) {
  // Attitude PID: proportional quaternion rotation error, integral of that
  // error, and derivative damping from target minus measured angular velocity.
  // Rate PID: proportional/integral rate error, filtered derivative of rate error.
  const error=mode==='attitude'?angleError:rateError;
  const damping=mode==='attitude'?rateError:derivative;
  return error.map((v,i)=>gains.kp*v+gains.ki*integral[i]+gains.kd*damping[i]);
}

export function allocateCurrents(requestedTorque,q,c) {
  const squared=dot(c.field,c.field),radius=c.side*c.radiusFraction;
  const momentWorld=squared>1e-24?scale(cross(c.field,requestedTorque),1/squared):[0,0,0];
  const bodyMoment=rotate(conjugate(q),momentWorld);
  return bodyMoment.map((v,i)=>v/(c.turns[i]*Math.PI*radius**2));
}

export class PIDController {
  constructor(config) {
    this.config=config;this.settings=config.controller;
    this.mode=norm(this.settings.targetRate)<1e-12?'attitude':'rate';
    this.integral=[0,0,0];this.derivative=[0,0,0];this.previousError=null;
    this.previousRequired=null;this.previousApplied=null;
    this.peaks={required:[0,0,0],applied:[0,0,0],requiredDerivative:[0,0,0],appliedDerivative:[0,0,0]};
    this.updates=0;
  }
  update(y,dt) {
    const q=y.slice(0,4),rateError=add(this.settings.targetRate,scale(y.slice(4),-1));
    const angleError=attitudeError(this.settings.targetQuaternion,q);
    const error=this.mode==='attitude'?angleError:rateError;
    if(dt>0 && this.previousError) {
      const raw=scale(add(error,scale(this.previousError,-1)),1/dt);
      const alpha=dt/(this.settings.derivativeFilter+dt);
      this.derivative=add(this.derivative,scale(add(raw,scale(this.derivative,-1)),alpha));
    }
    // Integrate only the component magnetorquers can actuate. Freeze the
    // integral on current saturation to prevent accumulation beyond limits.
    const candidateIntegral=add(this.integral,scale(perpendicular(error,this.config.field),dt));
    const input={mode:this.mode,angleError,rateError,angles:quaternionEuler(q),quaternion:q,angularVelocity:y.slice(4),targetRate:this.settings.targetRate,integral:candidateIntegral,derivative:this.derivative,gains:this.settings};
    let requestedTorque=desiredTorque(input),required=allocateCurrents(requestedTorque,q,this.config);
    const saturated=required.some((v,i)=>Math.abs(v)>this.settings.currentLimits[i]);
    if(saturated) {
      input.integral=this.integral;requestedTorque=desiredTorque(input);required=allocateCurrents(requestedTorque,q,this.config);
    } else this.integral=candidateIntegral;
    const currents=required.map((v,i)=>Math.max(-this.settings.currentLimits[i],Math.min(this.settings.currentLimits[i],v)));
    const requiredDerivative=dt>0&&this.previousRequired?scale(add(required,scale(this.previousRequired,-1)),1/dt):[0,0,0];
    const appliedDerivative=dt>0&&this.previousApplied?scale(add(currents,scale(this.previousApplied,-1)),1/dt):[0,0,0];
    for(const [key,values] of Object.entries({required,applied:currents,requiredDerivative,appliedDerivative}))
      values.forEach((v,i)=>this.peaks[key][i]=Math.max(this.peaks[key][i],Math.abs(v)));
    this.previousError=error;this.previousRequired=required;this.previousApplied=currents;this.updates++;
    const bodyMoment=currents.map((v,i)=>v*this.config.turns[i]*Math.PI*(this.config.side*this.config.radiusFraction)**2);
    const actualTorque=cross(rotate(q,bodyMoment),this.config.field);
    this.telemetry={mode:this.mode,currents,required,requiredDerivative,appliedDerivative,
      peaks:structuredClone(this.peaks),angleError,rateError,requestedTorque,actualTorque,
      unachievableTorque:add(requestedTorque,scale(actualTorque,-1)),
      saturated:required.some((v,i)=>Math.abs(v)>this.settings.currentLimits[i]),updates:this.updates};
    return currents;
  }
}
