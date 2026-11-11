// Pure choreography. Metres, Y up; physics owns released bar poses.
export const BAR = { L: .155, W: .06214, H: .04145 };
// The lingot tapers (bottom ~154 x 61.5 mm, top ~125 x 49 mm). Slots are packed
// as tight as the bottoms allow so cross bars keep their centre over the next layer.
export const SLOT = .0635;
export const ROBOT_SCALE = .55;
export const ROBOT_BASE = { x: -.32, y: .003181 * ROBOT_SCALE, z: -.28 };
export const TOWER = { x: .34, z: .34 };
export const FEED = { x: -.48, y: BAR.H, z: .45 };
export const CYCLE = 6;
export const REMOVE_CYCLE = 18;
export type Vec = [number, number, number];
export type Quat = { x:number; y:number; z:number; w:number };
export type Placement = { x:number; z:number; yaw:number; layer:number; slot:number };
export type Bar = { p:Vec; yaw:number; pitch:number; roll:number; visible:boolean; layer:number; slot:number; inTower:boolean; q?:Quat };
export type Dynamics = { reset:()=>void; step:(s:Sim,dt:number)=>boolean; settled:()=>boolean; chooseRemoval?:(s:Sim,candidates:number[])=>number|undefined };
export type Sim = {
  t:number; phase:'settle'|'build'|'remove'|'collapse'|'rest'; phaseT:number;
  placed:number; bars:Bar[]; plan:Placement[]; carry:boolean; tool:Vec; toolYaw:number;
  active:number; moves:number; removalHistory:number[]; destination?:Placement;
  physics?:Dynamics; random:()=>number; tower:{x:number;z:number}; layers:number;
  generation:number; source?:{p:Vec;yaw:number;q?:Quat}; released:boolean;
  mode:'pull'|'push'; side:number; push?:{p:Vec;yaw:number}; recoveryHistory:number[];
  removalScores?:{index:number;stable:boolean;motion:number}[];
};
export function placement(index:number, tower=TOWER):Placement {
  const layer=Math.floor(index/3),slot=index%3,offset=(slot-1)*SLOT;
  return {x:tower.x+(layer%2?offset:0),z:tower.z+(layer%2?0:offset),yaw:layer%2?Math.PI/2:0,layer,slot};
}
export function eligibleBars(s:Sim):number[] {
  const tower=s.bars.filter(b=>b.inTower);
  const top=Math.max(-1,...tower.map(b=>b.layer));
  const completed=tower.filter(b=>b.layer===top).length===3?top:top-1;
  return s.bars.flatMap((b,i)=>b.inTower&&b.layer<completed?[i]:[]);
}
export function createSim(random=Math.random):Sim {
  // A reproducible disorderly heap, already floor-adjacent, with no aligned supply columns.
  let seed=19371;
  const noise=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
  const bars:Bar[]=Array.from({length:MAX_BARS},(_,i)=>{
    const col=i%12,row=Math.floor(i/12),yaw=(noise()-.5)*Math.PI*2;
    const pitch=(noise()-.5)*.55,roll=(noise()-.5)*.7;
    const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2),cx=Math.cos(pitch/2),sx=Math.sin(pitch/2),cz=Math.cos(roll/2),sz=Math.sin(roll/2);
    return {p:[-.91+col*.085+(noise()-.5)*.04,.085+noise()*.08,.18+row*.11+(noise()-.5)*.05],yaw,pitch,roll,
      q:{x:sx*cy*cz+cx*sy*sz,y:cx*sy*cz-sx*cy*sz,z:cx*cy*sz+sx*sy*cz,w:cx*cy*cz-sx*sy*sz},visible:true,layer:-1,slot:-1,inTower:false};
  });
  return {t:0,phase:'settle',phaseT:0,placed:0,bars,plan:Array.from({length:MAX_BARS},(_,i)=>placement(i)),carry:false,tool:[FEED.x,.7,FEED.z],toolYaw:0,active:-1,moves:0,removalHistory:[],random,tower:{...TOWER},layers:LAYERS,generation:0,released:false,mode:'pull',side:1,recoveryHistory:[]};
}
/** Support-footprint margin at every layer, including the centre of mass above it. */
export function supportMargin(s:Sim,exclude:number):number {
  const remaining=s.bars.flatMap((b,i)=>b.inTower&&i!==exclude?[b]:[]);
  let margin=Infinity;
  for(let layer=0;layer<s.layers-1;layer++){
    const supports=remaining.filter(b=>b.layer===layer),above=remaining.filter(b=>b.layer>layer);
    if(!above.length)continue;
    if(!supports.length)return -1;
    const cx=above.reduce((v,b)=>v+b.p[0],0)/above.length,cz=above.reduce((v,b)=>v+b.p[2],0)/above.length;
    const hx=layer%2?BAR.W/2:BAR.L/2,hz=layer%2?BAR.L/2:BAR.W/2;
    const minX=Math.min(...supports.map(b=>b.p[0]-hx)),maxX=Math.max(...supports.map(b=>b.p[0]+hx));
    const minZ=Math.min(...supports.map(b=>b.p[2]-hz)),maxZ=Math.max(...supports.map(b=>b.p[2]+hz));
    margin=Math.min(margin,cx-minX,maxX-cx,cz-minZ,maxZ-cz);
  }
  return margin;
}
export function strategicCandidates(s:Sim):number[] {
  return eligibleBars(s).sort((a,b)=>supportMargin(s,b)-supportMargin(s,a)||(s.bars[b]?.layer??0)-(s.bars[a]?.layer??0));
}
const smooth=(x:number)=>{const t=Math.min(1,Math.max(0,x));return t*t*(3-2*t);};
const mix=(a:number,b:number,t:number)=>a+(b-a)*t;
type Key=[number,Vec,number,boolean];
function animate(s:Sim,keys:Key[],k:number) {
  let j=0;while(j<keys.length-2&&k>(keys[j+1]?.[0]??1))j++;
  const a=keys[j],b=keys[j+1];if(!a||!b)return;
  const u=smooth((k-a[0])/(b[0]-a[0]));
  s.tool=[mix(a[1][0],b[1][0],u),mix(a[1][1],b[1][1],u),mix(a[1][2],b[1][2],u)];s.toolYaw=mix(a[2],b[2],u);s.carry=a[3];
}
function follow(s:Sim,b:Bar,u=1) {
  b.p=[s.tool[0],s.tool[1]-BAR.H/2,s.tool[2]];b.yaw=s.toolYaw;
  // Reorient a fallen lingot smoothly during the lift, never flip it on the floor.
  const target={x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)},q=s.source?.q;
  if(q&&u<1){const sign=q.y*target.y+q.w*target.w<0?-1:1;
    const v={x:q.x*(1-u),y:q.y*(1-u)+target.y*u*sign,z:q.z*(1-u),w:q.w*(1-u)+target.w*u*sign};
    const n=Math.hypot(v.x,v.y,v.z,v.w);b.q={x:v.x/n,y:v.y/n,z:v.z/n,w:v.w/n};
  }else b.q=target;
}
function rememberSource(s:Sim,b:Bar) {s.source={p:[...b.p],yaw:b.q?Math.atan2(2*(b.q.w*b.q.y+b.q.x*b.q.z),1-2*(b.q.y*b.q.y+b.q.z*b.q.z)):b.yaw,...(b.q?{q:{...b.q}}:{})};s.released=false;}
function release(s:Sim,b:Bar,p:Placement,toolY=(p.layer+1)*BAR.H+.0008) {
  if(s.released)return;
  b.p=[p.x,toolY-BAR.H/2,p.z];b.yaw=p.yaw;b.q={x:0,y:Math.sin(p.yaw/2),z:0,w:Math.cos(p.yaw/2)};
  b.layer=p.layer;b.slot=p.slot;b.inTower=true;s.released=true;
}
function stepBuild(s:Sim) {
  if(s.active<0){
    const candidates=s.bars.flatMap((b,i)=>!b.inTower&&canReach([b.p[0],b.p[1]+BAR.H/2,b.p[2]])?[i]:[]);
    // Recover the topmost accessible bar first, leaving the rest of the pile untouched.
    candidates.sort((a,b)=>(s.bars[b]?.p[1]??0)-(s.bars[a]?.p[1]??0));
    const i=candidates[0];if(i===undefined){s.phase='remove';s.phaseT=0;return;}
    const bar=s.bars[i];if(!bar)return;s.active=i;rememberSource(s,bar);
    s.recoveryHistory.push(i);
  }
  const p=placement(s.placed,s.tower),b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const support=p.layer===0?0:Math.max(0,...s.bars.filter(item=>item.inTower&&item.layer===p.layer-1).map(item=>item.p[1]+BAR.H/2));
  const k=s.phaseT/CYCLE,pick:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]],dest:Vec=[p.x,support+BAR.H+.003,p.z];
  const lift:Vec=[pick[0],Math.min(maxToolHeight(pick[0],pick[2])-.01,Math.max(pick[1]+.22,dest[1]+.15)),pick[2]];
  const above:Vec=[dest[0],Math.min(maxToolHeight(dest[0],dest[2])-.01,dest[1]+.15),dest[2]];
  // Travel at lift height, then descend vertically: a diagonal path would plough through the feed stacks.
  const over:Vec=[dest[0],Math.min(maxToolHeight(dest[0],dest[2])-.01,lift[1]),dest[2]];
  animate(s,[[0,lift,source.yaw,false],[.15,pick,source.yaw,false],[.22,pick,source.yaw,true],[.36,lift,source.yaw,true],[.56,over,p.yaw,true],[.66,above,p.yaw,true],[.8,dest,p.yaw,true],[.85,dest,p.yaw,false],[1,above,p.yaw,false]],k);
  if(s.carry)follow(s,b,smooth((k-.22)/.18));
  if(k>=.85)release(s,b,p,dest[1]);
  if(k>=1-1e-8){s.placed++;s.phaseT=0;s.active=-1;delete s.source;if(s.placed>=s.layers*3){s.phase='remove';}}
}
function beginRemoval(s:Sim) {
  const eligible=strategicCandidates(s),i=s.physics?.chooseRemoval?.(s,eligible)??eligible[0];
  if(i===undefined)return;
  const b=s.bars[i];if(!b)return;
  s.active=i;s.phaseT=0;s.removalHistory.push(i);rememberSource(s,b);
  // Work from the accessible end, away from the robot base; randomness only chooses push/pull.
  const axis:Vec=[Math.cos(b.yaw),0,-Math.sin(b.yaw)];
  s.side=(b.p[0]-ROBOT_BASE.x)*axis[0]+(b.p[2]-ROBOT_BASE.z)*axis[2]>0?1:-1;
  s.mode=s.random()<.5?'pull':'push';
  const next=placement(s.placed+s.moves,s.tower);
  // At the reach limit, put extracted bars on the floor rather than overextending.
  s.destination=canReach([next.x,(next.layer+1)*BAR.H+.02,next.z])?next:{x:s.tower.x+s.side*.32,z:s.tower.z-.22,yaw:b.yaw,layer:0,slot:0};
}
function stepRemove(s:Sim) {
  if(s.active<0)beginRemoval(s);
  const b=s.bars[s.active],source=s.source,dest=s.destination;if(!b||!source||!dest)return;
  const k=s.phaseT/REMOVE_CYCLE,axis:Vec=[Math.cos(source.yaw)*s.side,0,-Math.sin(source.yaw)*s.side];
  const pick:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]];
  const outside:Vec=[pick[0]+axis[0]*(BAR.L+.055),pick[1],pick[2]+axis[2]*(BAR.L+.055)];
  const end:Vec=[pick[0]+axis[0]*BAR.L*.36,pick[1],pick[2]+axis[2]*BAR.L*.36];
  const pushStart:Vec=[pick[0]-axis[0]*(BAR.L/2+.012),pick[1],pick[2]-axis[2]*(BAR.L/2+.012)];
  const pushed:Vec=[pushStart[0]+axis[0]*(BAR.L+.055),pick[1],pushStart[2]+axis[2]*(BAR.L+.055)];
  const place:Vec=[dest.x,(dest.layer+1)*BAR.H+.0008,dest.z];
  const high=Math.min(maxToolHeight(place[0],place[2])-.02,Math.max(place[1]+.12,pick[1]+.16));
  const pulling=s.mode==='pull';
  const approach:Vec=[pick[0]+axis[0]*.24,pick[1]+.15,pick[2]+axis[2]*.24];
  const transit:Vec=[s.tool[0],Math.min(maxToolHeight(s.tool[0],s.tool[2])-.01,Math.max(pick[1]+.25,s.tool[1])),s.tool[2]];
  animate(s,[[0,transit,source.yaw,false],[.12,approach,source.yaw,false],[.25,pulling?end:pushStart,source.yaw,false],[.32,pulling?end:pushStart,source.yaw,pulling],[.64,pulling?[outside[0]+axis[0]*BAR.L*.36,outside[1],outside[2]+axis[2]*BAR.L*.36]:pushed,source.yaw,pulling],[.69,outside,source.yaw,false],[.72,outside,source.yaw,true],[.79,[outside[0],high,outside[2]],source.yaw,true],[.87,[place[0],high,place[2]],dest.yaw,true],[.94,place,dest.yaw,true],[.97,place,dest.yaw,false],[1,[place[0],place[1]+.1,place[2]],dest.yaw,false]],k);
  if(!pulling&&k>=.32&&k<.64)s.push={p:[s.tool[0],s.tool[1]-BAR.H/2,s.tool[2]],yaw:source.yaw};else delete s.push;
  if(pulling&&k>=.32&&k<.64){s.carry=true;follow(s,b);b.p[0]-=axis[0]*BAR.L*.36;b.p[2]-=axis[2]*BAR.L*.36;if(k>.6)b.inTower=false;}
  else if(k>=.72&&k<.97){
    if(!pulling&&k<.74){source.p=[...b.p];}
    b.inTower=false;follow(s,b);
  }
  if(k>=.97){release(s,b,dest);if(dest.layer===0)b.inTower=false;}
  if(k>=1){s.moves++;s.phaseT=0;s.active=-1;delete s.source;delete s.destination;}
}
export function chooseNextTower(s:Sim):{x:number;z:number} {
  const candidates=[{x:-.12,z:.34},{x:.34,z:-.12},{x:.34,z:.34},{x:-.12,z:.65}];
  return candidates.filter(p=>Math.hypot(p.x-s.tower.x,p.z-s.tower.z)>.3)
    .sort((a,b)=>clearance(b,s)-clearance(a,s))[0]??{x:-.12,z:.34};
}
function clearance(p:{x:number;z:number},s:Sim){return Math.min(...s.bars.map(b=>Math.hypot(p.x-b.p[0],p.z-b.p[2])));}
export function restartFromPile(s:Sim) {
  s.tower=chooseNextTower(s);s.layers=reachableLayers(s.tower);s.generation++;
  for(const b of s.bars){b.inTower=false;b.layer=-1;b.slot=-1;}
  s.phase='build';s.phaseT=0;s.placed=0;s.active=-1;s.moves=0;s.carry=false;delete s.source;delete s.push;
  s.plan=Array.from({length:MAX_BARS},(_,i)=>placement(i,s.tower));
}
export function stepSim(s:Sim,dtRaw:number) {
  const dt=Math.max(0,Math.min(dtRaw,.05));if(!dt)return;s.t+=dt;s.phaseT+=dt;
  if(s.phase==='settle'){
    if(s.phaseT>2&&(s.physics?.settled()??true)){s.phase='build';s.phaseT=0;}
  }else if(s.phase==='build')stepBuild(s);else if(s.phase==='remove'){
    // Let released contacts settle before inspecting the next support.
    if(s.active>=0||s.phaseT>1.5&&(s.physics?.settled()??true))stepRemove(s);
  }
  const fallen=s.physics?.step(s,dt);
  if(fallen&&(s.phase==='remove'||s.phase==='build')&&s.placed>=6){s.phase='collapse';s.phaseT=0;s.carry=false;s.active=-1;delete s.push;}
  if(s.phase==='collapse'&&s.phaseT>4&&(s.physics?.settled()??true)){s.phase='rest';s.phaseT=0;}
  else if(s.phase==='rest'&&s.phaseT>2)restartFromPile(s);
}
/**
 * Joint pivots of the uploaded "Industrial robot arm" OBJ, in model space
 * (meters, Y up, arm reaching toward -Z, yaw axis at x = -0.05).
 */
export const RIG = {
  yaw: [-0.05, 0, 0] as [number, number, number],
  shoulder: [-0.125, 0.74, 0.203] as [number, number, number],
  elbow: [-0.125, 1.63, 0.99] as [number, number, number],
  wrist: [-0.125, 1.66, -0.75] as [number, number, number],
  roll: [-0.125, 1.43, -0.75] as [number, number, number],
  toolDrop: 0.392, // wrist pivot -> point between finger tips
};
const S_F = -RIG.shoulder[2], S_Y = RIG.shoulder[1];
const E_F = -RIG.elbow[2], E_Y = RIG.elbow[1];
const W_F = -RIG.wrist[2], W_Y = RIG.wrist[1];
const L1 = Math.hypot(E_F - S_F, E_Y - S_Y);
const L2 = Math.hypot(W_F - E_F, W_Y - E_Y);
const A0 = Math.atan2(E_Y - S_Y, E_F - S_F); // rest upper-arm angle
const F0 = Math.atan2(W_Y - E_Y, W_F - E_F); // rest forearm angle
const LAT = RIG.shoulder[0] - RIG.yaw[0]; // arm plane offset from yaw axis

/** Analytic IK for a downward-pointing gripper at P; returns rotation deltas from the rest pose. */
export function solveIK(P: [number, number, number], toolYaw: number) {
  const dx = (P[0] - ROBOT_BASE.x) / ROBOT_SCALE, dz = (P[2] - ROBOT_BASE.z) / ROBOT_SCALE;
  const R = Math.max(Math.hypot(dx, dz), Math.abs(LAT) + 1e-3);
  const fwd = Math.sqrt(R * R - LAT * LAT);
  const yaw = Math.atan2(dx, dz) - Math.atan2(LAT, -fwd);
  const r = fwd - S_F;
  const h = (P[1] - ROBOT_BASE.y) / ROBOT_SCALE + RIG.toolDrop - S_Y;
  const D = Math.min(Math.max(Math.hypot(r, h), Math.abs(L2 - L1) + 1e-3), L1 + L2 - 1e-3);
  const a = Math.atan2(h, r) + Math.acos((L1 ** 2 + D ** 2 - L2 ** 2) / (2 * L1 * D));
  const b = -(Math.PI - Math.acos((L1 ** 2 + L2 ** 2 - D ** 2) / (2 * L1 * L2)));
  const shoulder = a - A0;
  const elbow = b + A0 - F0;
  const wrist = -(a + b - F0);
  const roll = toolYaw - yaw + Math.PI / 2;
  return { yaw, shoulder, elbow, wrist, roll };
}

/** Reach envelope of the actual articulated model, with a small joint-limit margin. */
export function maxToolHeight(x:number,z:number):number {
  const radius=Math.hypot((x-ROBOT_BASE.x)/ROBOT_SCALE,(z-ROBOT_BASE.z)/ROBOT_SCALE);
  const forward=Math.sqrt(Math.max(0,radius*radius-LAT*LAT))-S_F;
  const reach=L1+L2-.025;
  if(Math.abs(forward)>reach)return -1;
  return ROBOT_BASE.y+(S_Y+Math.sqrt(reach*reach-forward*forward)-RIG.toolDrop)*ROBOT_SCALE;
}
export function canReach(p:Vec):boolean {
  const radius=Math.hypot((p[0]-ROBOT_BASE.x)/ROBOT_SCALE,(p[2]-ROBOT_BASE.z)/ROBOT_SCALE);
  const forward=Math.sqrt(Math.max(0,radius*radius-LAT*LAT))-S_F;
  const h=(p[1]-ROBOT_BASE.y)/ROBOT_SCALE+RIG.toolDrop-S_Y;
  const d=Math.hypot(forward,h);
  return d<L1+L2-.025&&d>Math.abs(L2-L1)+.01&&p[1]>=0;
}
export function reachableLayers(tower=TOWER):number {
  let count=0;
  for(let layer=0;layer<100;layer++){
    if([0,1,2].some(slot=>{const p=placement(layer*3+slot,tower);return !canReach([p.x,(layer+1)*BAR.H+.02,p.z]);}))break;
    count++;
  }
  return count;
}
export const LAYERS=reachableLayers();
export const MAX_BARS=LAYERS*3;
