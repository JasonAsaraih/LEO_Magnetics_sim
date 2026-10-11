// Quaternion convention: [w,x,y,z], mapping body vectors into the world frame.
export const add=(a,b)=>a.map((v,i)=>v+b[i]);
export const scale=(a,s)=>a.map(v=>v*s);
export const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
export const norm=a=>Math.hypot(...a);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const conjugate=q=>[q[0],-q[1],-q[2],-q[3]];
export function multiply(a,b) {return [a[0]*b[0]-dot(a.slice(1),b.slice(1)),...add(add(scale(b.slice(1),a[0]),scale(a.slice(1),b[0])),cross(a.slice(1),b.slice(1)))];}
export function rotate(q,v) {const t=scale(cross(q.slice(1),v),2);return add(add(v,scale(t,q[0])),cross(q.slice(1),t));}
export function eulerQuaternion(roll,pitch,yaw) {
  const [r,p,y]=[roll,pitch,yaw].map(v=>v/2),[cr,cp,cy]=[r,p,y].map(Math.cos),[sr,sp,sy]=[r,p,y].map(Math.sin);
  return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy];
}
export function quaternionEuler([w,x,y,z]) {
  return [Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x)))),Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
}
export function attitudeError(target,current) {
  let q=multiply(target,conjugate(current));q=scale(q,1/norm(q));
  if(q[0]<0)q=scale(q,-1); // shortest rotation, independent of quaternion sign
  const length=norm(q.slice(1));
  return length<1e-12?scale(q.slice(1),2):scale(q.slice(1),2*Math.atan2(length,q[0])/length);
}
export function perpendicular(vector,field) {
  const squared=dot(field,field);
  return squared>1e-24?add(vector,scale(field,-dot(vector,field)/squared)):[0,0,0];
}
