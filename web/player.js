import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {rotate,moments,cross,norm,scale} from './physics.js';
import {attitudeError,quaternionEuler} from './math.js';
const $=id=>document.getElementById(id);
const defaults={side:.1,mass:1,currents:[.1,.1,.1],turns:[100,100,100],radiusFraction:.4,quaternion:[1,0,0,0],omega:[0,0,0],field:[0,0,40e-6]};
let config;
try{config=JSON.parse($('config').textContent);}catch{config=defaults;}
const world=new THREE.Scene();world.background=new THREE.Color('#0e1522');
const view=$('view'),renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));view.appendChild(renderer.domElement);
const camera=new THREE.PerspectiveCamera(42,1,config.side*.001,config.side*1000);
camera.up.set(0,0,1);camera.position.set(config.side*4,config.side*4,config.side*3);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
new ResizeObserver(()=>{renderer.setSize(view.clientWidth,view.clientHeight);camera.aspect=view.clientWidth/view.clientHeight;camera.updateProjectionMatrix();}).observe(view);
const body=new THREE.Group();world.add(body);
const box=new THREE.BoxGeometry(config.side,config.side,config.side);
body.add(new THREE.Mesh(box,new THREE.MeshBasicMaterial({color:'#8c9bb1',transparent:true,opacity:.1,depthWrite:false})));
body.add(new THREE.LineSegments(new THREE.EdgesGeometry(box),new THREE.LineBasicMaterial({color:'#8c9bb1'})));
const axes=[[1,0,0],[0,1,0],[0,0,1]],colors=['#ef8b2c','#4caf70','#8168ed'];
for(let i=0;i<3;i++){
  const points=[];
  for(let j=0;j<=120;j++){
    const a=j*2*Math.PI/120,p=scale(axes[i],config.side/2);
    p[(i+1)%3]+=config.side*config.radiusFraction*Math.cos(a);p[(i+2)%3]+=config.side*config.radiusFraction*Math.sin(a);points.push(new THREE.Vector3(...p));
  }
  body.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:colors[i]})));
}
world.add(new THREE.AxesHelper(config.side*.75));
const vectorNames=['m X','m Y','m Z','m net','B external','Torque τ'];
const vectorColors=[...colors,'#d3a300','#00a6a6','#d528b8'];
const vectorHelpers=vectorNames.map((name,i)=>{
  const arrow=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(),1,vectorColors[i]);world.add(arrow);
  const label=document.createElement('div');label.className='vector-label';label.style.color=vectorColors[i];view.appendChild(label);return {name,arrow,label,tip:new THREE.Vector3()};
});
let playing=false,speed=1,target=0,realTime=0,lastWall=performance.now(),lastRequest=0,busy=false,generation=0;
let current={time:0,y:[...config.quaternion,...config.omega],currents:[...config.currents]},grid=null,paths=[],history=[];
const workerURL=URL.createObjectURL(new Blob([WORKER_SOURCE],{type:'text/javascript'}));
const worker=new Worker(workerURL);URL.revokeObjectURL(workerURL);
let fieldLines=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:'#458acc',transparent:true,opacity:.75}));world.add(fieldLines);
const coneGeometry=new THREE.ConeGeometry(.22,1,5),coneMaterial=new THREE.MeshBasicMaterial({color:'#609ee5'});
const fieldArrows=new THREE.InstancedMesh(coneGeometry,coneMaterial,729);fieldArrows.count=0;world.add(fieldArrows);
const fieldDots=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:3,sizeAttenuation:false,vertexColors:true}));world.add(fieldDots);
function clock(now){const dt=Math.max(0,(now-lastWall)/1000);if(playing){target+=dt*speed;realTime+=dt;}lastWall=now;}
function request(force=false){if(busy)return;busy=true;worker.postMessage({type:'tick',generation,target,fields:$('lines').checked||$('arrows').checked,force});lastRequest=performance.now();}
function reset(){clock(performance.now());playing=false;target=realTime=0;generation++;history=[];drawHistory();current={time:0,y:[...config.quaternion,...config.omega],currents:[...config.currents]};busy=true;$('play').textContent='Play';$('status').textContent='Paused';worker.postMessage({type:'reset',config,generation,fields:true});}
worker.onmessage=({data})=>{
  if(data.generation!==generation)return;busy=false;
  if(data.error){playing=false;$('play').textContent='Play';$('error').textContent=data.error;return;}
  current=data;updateCurrentLog();
  if(data.time>0 && (!history.length||data.time>history.at(-1).time)){history.push({time:data.time,y:[...data.y],currents:[...data.currents],controller:data.controller});if(history.length>2000)history.shift();drawHistory();}
  if(data.grid){grid=data.grid;paths=data.lines;rebuildField();}
};
worker.onerror=error=>{busy=false;playing=false;$('play').textContent='Play';$('error').textContent=`Simulation worker failed: ${error.message}`;};
$('play').onclick=()=>{clock(performance.now());playing=!playing;$('play').textContent=playing?'Pause':'Play';$('status').textContent=playing?'Playing':'Paused';request();};
$('reset').onclick=()=>reset();
$('speed').oninput=()=>{clock(performance.now());speed=Number($('speed').value);$('speed-value').textContent=`${speed.toFixed(1)}×`;};
for(const id of ['lines','arrows'])$(id).onchange=()=>{fieldLines.visible=$('lines').checked;fieldArrows.visible=fieldDots.visible=$('arrows').checked;request(true);};
$('arrow-size').oninput=()=>rebuildArrows();
function rebuildArrows(){
  if(!grid)return;
  const dummy=new THREE.Object3D(),up=new THREE.Vector3(0,1,0);let count=0;
  const length=config.side*2*.05*Number($('arrow-size').value);
  for(let i=0;i<grid.points.length;i+=3){
    const b=grid.values.slice(i,i+3),strength=norm(b);if(!Number.isFinite(strength)||strength===0)continue;
    const direction=new THREE.Vector3(...b).normalize(),p=new THREE.Vector3(...grid.points.slice(i,i+3));
    dummy.position.copy(p).addScaledVector(direction,length/2);dummy.quaternion.setFromUnitVectors(up,direction);dummy.scale.setScalar(length);dummy.updateMatrix();fieldArrows.setMatrixAt(count++,dummy.matrix);
  }
  fieldArrows.count=count;fieldArrows.instanceMatrix.needsUpdate=true;fieldArrows.computeBoundingSphere();
}
function rebuildField(){
  const positions=[];for(const path of paths)for(let i=1;i<path.length;i++)positions.push(...path[i-1],...path[i]);
  fieldLines.geometry.dispose();fieldLines.geometry=new THREE.BufferGeometry();fieldLines.geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
  const dots=[],colorValues=[],magnitudes=[];
  for(let i=0;i<grid.values.length;i+=3){const value=norm(grid.values.slice(i,i+3));if(Number.isFinite(value)&&value>0)magnitudes.push(Math.log10(value));}
  const min=Math.min(...magnitudes),max=Math.max(...magnitudes);
  for(let i=0;i<grid.points.length;i+=3){
    const value=norm(grid.values.slice(i,i+3));if(!Number.isFinite(value)||value===0)continue;
    dots.push(...grid.points.slice(i,i+3));const fraction=(Math.log10(value)-min)/(max-min||1);
    const color=new THREE.Color().setHSL(.75-fraction*.6,.8,.55);colorValues.push(color.r,color.g,color.b);
  }
  fieldDots.geometry.dispose();fieldDots.geometry=new THREE.BufferGeometry();fieldDots.geometry.setAttribute('position',new THREE.Float32BufferAttribute(dots,3));fieldDots.geometry.setAttribute('color',new THREE.Float32BufferAttribute(colorValues,3));rebuildArrows();
}
function updateVectors(){
  const q=current.y.slice(0,4),bodyMoment=moments({...config,currents:current.currents||config.currents}),m=rotate(q,bodyMoment),tau=cross(m,config.field),gain=Number($('vector-scale').value),extent=config.side*2;
  body.quaternion.set(q[1],q[2],q[3],q[0]);
  const vectors=axes.map((axis,i)=>rotate(q,scale(axis,bodyMoment[i]))).concat([m,config.field,tau]);
  const origins=axes.map(axis=>rotate(q,scale(axis,config.side/2))).concat([[0,0,0],[0,0,0],[0,0,0]]);
  for(let i=0;i<vectorHelpers.length;i++){
    const {arrow,label,tip,name}=vectorHelpers[i],v=vectors[i],magnitude=norm(v),reference=i<4?.1:i===4?40e-6:4e-6,length=extent*.6*gain*magnitude/reference;
    arrow.position.set(...origins[i]);arrow.visible=magnitude>1e-18;
    if(arrow.visible){arrow.setDirection(new THREE.Vector3(...v).normalize());arrow.setLength(length,length*.18,length*.075);}
    tip.copy(arrow.position);if(magnitude>0)tip.addScaledVector(new THREE.Vector3(...v),extent*.6*gain/reference);
    const projected=tip.clone().project(camera);
    label.style.display=projected.z>=-1&&projected.z<=1?'block':'none';
    label.style.left=`${(projected.x*.5+.5)*view.clientWidth+4}px`;label.style.top=`${(-projected.y*.5+.5)*view.clientHeight-16+(i===2?20:0)}px`;
    label.textContent=`${name}: ${(magnitude*(i>=4?1e6:1)).toPrecision(3)} ${i<4?'A·m²':i===4?'µT':'µN·m'}`;
  }
  $('sim-time').textContent=`${current.time.toFixed(3)} s`;$('real-time').textContent=`${realTime.toFixed(3)} s`;
  $('angular-speed').textContent=`${(norm(current.y.slice(4))*180/Math.PI).toPrecision(4)} °/s`;
  $('moment').textContent=`${norm(m).toPrecision(4)} A·m²`;$('torque').textContent=`${(norm(tau)*1e6).toPrecision(4)} µN·m`;
  const ctrl=current.controller,angles=quaternionEuler(q).map(v=>(v*180/Math.PI).toFixed(1));
  $('angles').textContent=angles.join(' / ')+' °';
  $('controller-status').textContent=ctrl?`PID ${ctrl.mode==='attitude'?'attitude + zero-rate':'angular-velocity'} control${ctrl.saturated?' · current limit reached':''}`:'Manual currents';
  if(config.controller?.enabled) {
    const rateError=config.controller.targetRate.map((v,i)=>v-current.y[i+4]);
    const angle=norm(attitudeError(config.controller.targetQuaternion,q))*180/Math.PI;
    $('angle-error').textContent=ctrl?.mode==='attitude'?`${angle.toFixed(2)} °`:'Not targeted';
    $('rate-error').textContent=`${(norm(rateError)*180/Math.PI).toFixed(3)} °/s`;
    $('unmet-torque').textContent=ctrl?`${(norm(ctrl.unachievableTorque)*1e6).toPrecision(3)} µN·m`:'—';
    $('target-description').textContent=norm(config.field)<1e-12?'External field is zero: the coils have no torque authority.':ctrl?.mode==='attitude'?`Target: roll/pitch/yaw ${config.controller.targetEulerDegrees.join(' / ')}° and zero angular velocity. A fixed magnetic field cannot provide torque along B.`:`Target world angular velocity: ${config.controller.targetRate.map(v=>(v*180/Math.PI).toFixed(2)).join(' / ')} °/s. Final attitude is ignored for nonzero rate targets.`;
  }
  $('lag').textContent=target-current.time>.2?` ${Math.max(0,target-current.time).toFixed(2)} simulated seconds pending; lower the time scale if needed.`:'';
}
function drawHistory(){
  const canvas=$('history'),ctx=canvas.getContext('2d');canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=100*devicePixelRatio;ctx.scale(devicePixelRatio,devicePixelRatio);
  const width=canvas.clientWidth,height=100;ctx.clearRect(0,0,width,height);if(history.length<2)return;
  const values=history.map(p=>norm(p.y.slice(4))*180/Math.PI),maximum=Math.max(...values,1e-6),start=history[0].time,end=history.at(-1).time;
  ctx.strokeStyle='#6aa3ff';ctx.beginPath();history.forEach((p,i)=>{const x=50+(p.time-start)/(end-start||1)*(width-65),y=75-values[i]/maximum*55;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();ctx.fillStyle='#9baec8';ctx.font='11px system-ui';ctx.fillText(`${maximum.toPrecision(3)} °/s`,4,15);ctx.fillText(`${start.toFixed(1)} s`,50,95);ctx.fillText(`${end.toFixed(1)} s`,width-70,95);
}
function download(text,name){const url=URL.createObjectURL(new Blob([text],{type:'text/csv'})),a=document.createElement('a');a.href=url;a.download=name;a.click();URL.revokeObjectURL(url);}
$('export').onclick=()=>{
  const columns=['time_s','qw','qx','qy','qz','omega_x_rad_s','omega_y_rad_s','omega_z_rad_s',
    'current_x_A','current_y_A','current_z_A','required_x_A','required_y_A','required_z_A',
    'dI_x_A_s','dI_y_A_s','dI_z_A_s','required_dI_x_A_s','required_dI_y_A_s','required_dI_z_A_s'];
  const text=columns.join(',')+'\n'+history.map(p=>[p.time,...p.y,...p.currents,
    ...(p.controller?.required||p.currents),...(p.controller?.appliedDerivative||[0,0,0]),
    ...(p.controller?.requiredDerivative||[0,0,0])].join(',')).join('\n');
  download(text,'trajectory.csv');
};
function peakValues(){return current.controller?.peaks||{applied:config.currents.map(Math.abs),required:config.currents.map(Math.abs),appliedDerivative:[0,0,0],requiredDerivative:[0,0,0]};}
function updateCurrentLog(){
  const peaks=peakValues(),currents=current.currents||config.currents;
  $('current-log').innerHTML=axes.map((_,i)=>`<tr><td>${['X','Y','Z'][i]}</td><td id="current-${i}">${currents[i].toPrecision(5)}</td><td>${!config.inductanceH?'Not configured':config.inductanceH[i]===null?'Outside model':(config.inductanceH[i]*1000).toPrecision(4)}</td><td>${peaks.applied[i].toPrecision(5)}</td><td id="peak-required-${i}">${peaks.required[i].toPrecision(5)}</td><td>${peaks.appliedDerivative[i].toPrecision(5)}</td><td>${peaks.requiredDerivative[i].toPrecision(5)}</td></tr>`).join('');
}
$('export-peaks').onclick=()=>{
  const peaks=peakValues();
  const rows=['coil,max_required_A,max_applied_A,max_required_dI_dt_A_s,max_applied_dI_dt_A_s,current_limit_A,control_period_s'];
  axes.forEach((_,i)=>rows.push([['X','Y','Z'][i],peaks.required[i],peaks.applied[i],peaks.requiredDerivative[i],peaks.appliedDerivative[i],config.controller?.enabled?config.controller.currentLimits[i]:'',config.controller?.enabled?config.controller.period:''].join(',')));
  download(rows.join('\n'),'peak-currents.csv');
};
function frame(now){clock(now);controls.update();updateVectors();renderer.render(world,camera);if((playing||target-current.time>1e-9)&&now-lastRequest>50)request();requestAnimationFrame(frame);}
reset();requestAnimationFrame(frame);
// Read-only diagnostics for browser regression checks; no server round trips.
window.playerDiagnostics=()=>({time:current.time,target,realTime,speed,playing,quaternion:current.y.slice(0,4),camera:camera.position.toArray(),generation,fieldLines:paths.length,busy,currents:current.currents,controller:current.controller});
window.addEventListener('pagehide',()=>worker.terminate());
