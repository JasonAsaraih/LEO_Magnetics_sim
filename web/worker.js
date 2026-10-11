import {advance,initializeController,fieldGrid,fieldLines} from './physics.js';
let config,state,generation=0,lastField=-Infinity;
self.onmessage=({data})=>{
  try {
    if(data.type==='reset'){config=data.config;state={y:[...config.quaternion,...config.omega],time:0};initializeController(state,config);generation=data.generation;lastField=-Infinity;}
    if(data.generation!==generation)return;
    if(data.type==='tick')advance(state,config,data.target);
    const active={...config,currents:state.currents};
    const result={generation,time:state.time,y:state.y,currents:state.currents,controller:state.controller?.telemetry};
    if(data.fields && (performance.now()-lastField>500 || data.type==='reset' || data.force)) {
      const grid=fieldGrid(active,state.y.slice(0,4));
      result.grid=grid;result.lines=fieldLines(active,state.y.slice(0,4),grid);lastField=performance.now();
    }
    self.postMessage(result);
  } catch(error){self.postMessage({generation,error:error.message});}
};
