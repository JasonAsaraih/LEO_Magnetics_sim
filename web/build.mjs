import {build} from 'esbuild';
import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
mkdirSync('web/dist',{recursive:true});
const worker=await build({entryPoints:['web/worker.js'],bundle:true,write:false,format:'iife',minify:true});
const player=await build({entryPoints:['web/player.js'],bundle:true,write:false,format:'iife',minify:true,define:{WORKER_SOURCE:JSON.stringify(worker.outputFiles[0].text)}});
let template=readFileSync('web/template.html','utf8').replace('/* PLAYER_BUNDLE */',player.outputFiles[0].text);
writeFileSync('web/dist/player.html',template);
console.log('Built self-contained browser player');
