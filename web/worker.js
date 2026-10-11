import {advance,fieldGrid,fieldLines} from './physics.js';
let config,state,generation=0,lastField=-Infinity;
self.onmessage=({data})=>{
  try {
    if(data.type==='reset'){config=data.config;state={y:[...config.quaternion,...config.omega],time:0};generation=data.generation;lastField=-Infinity;}
    if(data.generation!==generation)return;
    if(data.type==='tick')advance(state,config,data.target);
    const result={generation,time:state.time,y:state.y};
    if(data.fields && (performance.now()-lastField>500 || data.type==='reset' || data.force)) {
      const grid=fieldGrid(config,state.y.slice(0,4));
      result.grid=grid;result.lines=fieldLines(config,state.y.slice(0,4),grid);lastField=performance.now();
    }
    self.postMessage(result);
  } catch(error){self.postMessage({generation,error:error.message});}
};
