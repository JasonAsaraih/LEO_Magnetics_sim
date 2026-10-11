// SI-unit equations shared by the browser worker and numerical tests.
import {add,scale,dot,norm,cross,multiply,rotate} from './math.js';
export {add,scale,dot,norm,cross,multiply,rotate} from './math.js';
import {PIDController} from './controller.js';
export function moments(c) {return c.currents.map((v,i)=>v*c.turns[i]*Math.PI*(c.side*c.radiusFraction)**2);}
export function derivative(y,c) {const q=y.slice(0,4),w=y.slice(4),m=rotate(q,moments(c));return [...scale(multiply([0,...w],q),.5),...scale(cross(m,c.field),6/(c.mass*c.side**2))];}
export function step(y,c,dt) {
  const k1=derivative(y,c),k2=derivative(add(y,scale(k1,dt/2)),c),k3=derivative(add(y,scale(k2,dt/2)),c),k4=derivative(add(y,scale(k3,dt)),c);
  const result=add(y,scale(add(add(k1,scale(k2,2)),add(scale(k3,2),k4)),dt/6));
  const qn=norm(result.slice(0,4));for(let i=0;i<4;i++)result[i]/=qn;return result;
}
export function initializeController(state,c) {
  state.currents=[...c.currents];
  if(c.controller?.enabled) {
    state.controller=new PIDController(c);
    state.currents=state.controller.update(state.y,0);
    state.nextControl=state.time+c.controller.period;
  }
}
export function advance(state,c,target,budget=2000) {
  if(!state.currents)initializeController(state,c);
  for(let i=0;i<budget && target-state.time>1e-12;i++) {
    if(state.controller && state.time>=state.nextControl-1e-10) {
      state.currents=state.controller.update(state.y,c.controller.period);
      state.nextControl+=c.controller.period;
    }
    const active={...c,currents:state.currents};
    const frequency=Math.sqrt(norm(moments(active))*norm(c.field)*6/(c.mass*c.side**2));
    const dt=Math.min(target-state.time,.05,.03/Math.max(norm(state.y.slice(4)),frequency,1e-12),state.controller?state.nextControl-state.time:Infinity);
    state.y=step(state.y,active,dt);state.time+=dt;
  }
}
export function wires(c,q,segments=240) {
  const axes=[[1,0,0],[0,1,0],[0,0,1]],radius=c.side*c.radiusFraction;
  return axes.map((axis,i)=>{
    const u=axes[(i+1)%3],v=axes[(i+2)%3],center=scale(axis,c.side/2),wire=[];
    for(let j=0;j<=segments;j++){const a=j*2*Math.PI/segments;wire.push(rotate(q,add(center,add(scale(u,radius*Math.cos(a)),scale(v,radius*Math.sin(a))))));}
    return wire;
  });
}
export function prepareField(c,q) {
  const segments=[];
  for(const [i,wire] of wires(c,q).entries()) {
    if(c.currents[i]===0)continue;
    for(let j=0;j<wire.length-1;j++){
      const a=wire[j],b=wire[j+1],dl=add(b,scale(a,-1));
      segments.push({a,dl,mid:scale(add(a,b),.5),factor:1e-7*c.currents[i]*c.turns[i],length2:dot(dl,dl)});
    }
  }
  return p=>{
    const field=[...c.field];
    for(const s of segments) {
      const delta=add(p,scale(s.a,-1)),fraction=Math.max(0,Math.min(1,dot(delta,s.dl)/s.length2));
      if(norm(add(delta,scale(s.dl,-fraction)))<.002)return [NaN,NaN,NaN];
      const r=add(p,scale(s.mid,-1)),factor=s.factor/norm(r)**3,b=cross(s.dl,r);
      for(let i=0;i<3;i++)field[i]+=b[i]*factor;
    }
    return field;
  };
}
export function fieldGrid(c,q,n=9) {
  const extent=c.side*2,field=prepareField(c,q),points=[],values=[];
  for(let x=0;x<n;x++)for(let y=0;y<n;y++)for(let z=0;z<n;z++) {
    const p=[x,y,z].map(v=>-extent+v*2*extent/(n-1));points.push(...p);values.push(...field(p));
  }
  return {n,extent,points,values};
}
export function interpolate(grid,p) {
  const {n,extent,values}=grid,coord=p.map(v=>(v+extent)/(2*extent)*(n-1));
  if(coord.some(v=>v<0||v>n-1))return [NaN,NaN,NaN];
  const base=coord.map(v=>Math.min(Math.floor(v),n-2)),fraction=coord.map((v,i)=>v-base[i]),result=[0,0,0];
  for(let x=0;x<2;x++)for(let y=0;y<2;y++)for(let z=0;z<2;z++){
    const bits=[x,y,z],weight=bits.reduce((w,v,i)=>w*(v?fraction[i]:1-fraction[i]),1);
    if(weight===0)continue;
    const index=((base[0]+x)*n*n+(base[1]+y)*n+base[2]+z)*3;
    for(let i=0;i<3;i++)result[i]+=weight*values[index+i];
  }
  return result;
}
export function fieldLines(c,q,grid) {
  const axes=[[1,0,0],[0,1,0],[0,0,1]],seeds=[];
  for(let i=0;i<3;i++)for(let j=0;j<4;j++) {
    const a=j*Math.PI/2;
    seeds.push(rotate(q,add(scale(axes[i],c.side/2),add(scale(axes[(i+1)%3],c.side*c.radiusFraction*.5*Math.cos(a)),scale(axes[(i+2)%3],c.side*c.radiusFraction*.5*Math.sin(a))))));
  }
  const direction=p=>{const b=interpolate(grid,p),length=norm(b);return length>1e-20 && Number.isFinite(length)?scale(b,1/length):null;};
  const stepLength=c.side/60;
  return seeds.map(seed=>{
    const branches=[-1,1].map(sign=>{
      let p=seed;const path=[p];
      for(let k=0;k<360;k++) {
        const d1=direction(p);if(!d1)break;
        const d2=direction(add(p,scale(d1,sign*stepLength/2)));if(!d2)break;
        const next=add(p,scale(d2,sign*stepLength));if(!direction(next))break;
        path.push(next);p=next;
        if(k>20 && norm(add(p,scale(seed,-1)))<stepLength*.8)break;
      }
      return path;
    });
    return branches[0].slice(1).reverse().concat(branches[1]);
  });
}
