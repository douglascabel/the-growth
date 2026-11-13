// Pure choreography. Metres, Y up; physics owns released bar poses.
export const BAR = { L: .155, W: .06214, H: .04145 };
// The lingot tapers (bottom ~154 x 61.5 mm, top ~125 x 49 mm). Slots are packed
// as tight as the bottoms allow so cross bars keep their centre over the next layer.
export const SLOT = .0635;
export const ROBOT_SCALE = .55;
export const ROBOT_BASE = { x: -.32, y: .003181 * ROBOT_SCALE, z: -.28 };
// The tower sits on a robot axis so one bar orientation always runs sideways to the arm:
// both of its ends are reachable without the arm crossing over the tower.
export const TOWER = { x: ROBOT_BASE.x + .9, z: ROBOT_BASE.z };
export const FEED = { x: -.48, y: BAR.H, z: .45 };
export const CYCLE = 6;
export const REMOVE_CYCLE = 18;
export type Vec = [number, number, number];
export type Quat = { x:number; y:number; z:number; w:number };
/** Finger half-opening (open/closed) and how far fingertips reach below the tool point. */
export type Grip = { open:number; closed:number; depth:number };
export const END_GRIP:Grip={open:BAR.L/2+.012,closed:BAR.L/2+.004,depth:-.004};
export const PINCH_GRIP:Grip={open:BAR.W/2+.012,closed:BAR.W/2+.002,depth:BAR.H*.5};
export const PUSH_GRIP:Grip={open:.004,closed:.004,depth:BAR.H*.5};
/** Horizontal fingertip tab: lets a downward gripper push a bar end recessed below the cross layers. */
export const TAB = { L:.2, H:BAR.H*.64, W:.012 };
export type Placement = { x:number; z:number; yaw:number; layer:number; slot:number };
export type Bar = { p:Vec; yaw:number; pitch:number; roll:number; visible:boolean; layer:number; slot:number; inTower:boolean; q?:Quat; loose?:boolean };
export type Dynamics = { reset:()=>void; step:(s:Sim,dt:number)=>boolean; settled:()=>boolean; chooseRemoval?:(s:Sim,candidates:number[])=>number|undefined; armClear?:(tool:Vec,yaw:number,box:{min:Vec;max:Vec})=>boolean };
export type Sim = {
  t:number; phase:'settle'|'build'|'remove'|'collapse'|'rest'; phaseT:number;
  placed:number; bars:Bar[]; plan:Placement[]; carry:boolean; tool:Vec; toolYaw:number;
  active:number; moves:number; removalHistory:number[]; destination?:Placement;
  physics?:Dynamics; random:()=>number; tower:{x:number;z:number}; layers:number;
  generation:number; source?:{p:Vec;yaw:number;q?:Quat;entry:Vec}; released:boolean;
  mode:'pull'|'push'; side:number; push?:{p:Vec;yaw:number}; recoveryHistory:number[];
  removalScores?:{index:number;stable:boolean;motion:number}[];
  grip:Grip; grab?:Vec; restack:number;
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
  return {t:0,phase:'settle',phaseT:0,placed:0,bars,plan:Array.from({length:MAX_BARS},(_,i)=>placement(i)),carry:false,tool:[FEED.x,.7,FEED.z],toolYaw:0,active:-1,moves:0,removalHistory:[],random,tower:{...TOWER},layers:LAYERS,generation:0,released:false,mode:'pull',side:1,recoveryHistory:[],grip:END_GRIP,restack:-1};
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
/** True when a bar runs across the arm's line of sight, so both ends are reachable beside the tower. */
export function lateral(p:Vec,yaw:number):boolean {
  const dx=p[0]-ROBOT_BASE.x,dz=p[2]-ROBOT_BASE.z,n=Math.hypot(dx,dz)||1;
  return Math.abs((Math.cos(yaw)*dx-Math.sin(yaw)*dz)/n)<.35;
}
export function strategicCandidates(s:Sim):number[] {
  // Prefer bars clamped by at least three loaded layers: lightly loaded bars drag their neighbours along.
  const all=eligibleBars(s).filter(i=>{const b=s.bars[i];return !!b&&lateral(b.p,b.yaw);}),top=Math.max(0,...all.map(i=>s.bars[i]?.layer??0));
  const loaded=all.filter(i=>(s.bars[i]?.layer??0)<=top-3);
  // Shallow bars first: the push must overcome friction from the load above, and a deep push tips the whole tower.
  const margin=(i:number)=>supportMargin(s,i)>.01?1:0;
  const loose=(i:number)=>s.bars[i]?.loose?1:0;
  return (loaded.length?loaded:all).sort((a,b)=>margin(b)-margin(a)||loose(b)-loose(a)||(s.bars[b]?.layer??0)-(s.bars[a]?.layer??0)||supportMargin(s,b)-supportMargin(s,a));
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
function follow(s:Sim,b:Bar,u=1,yawOffset=0,along=0) {
  b.yaw=s.toolYaw+yawOffset;
  b.p=[s.tool[0]-Math.cos(b.yaw)*along,s.tool[1]-BAR.H/2,s.tool[2]+Math.sin(b.yaw)*along];
  // Reorient a fallen lingot smoothly during the lift, never flip it on the floor.
  const target={x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)},q=s.source?.q;
  if(q&&u<1){const sign=q.y*target.y+q.w*target.w<0?-1:1;
    const v={x:q.x*(1-u),y:q.y*(1-u)+target.y*u*sign,z:q.z*(1-u),w:q.w*(1-u)+target.w*u*sign};
    const n=Math.hypot(v.x,v.y,v.z,v.w);b.q={x:v.x/n,y:v.y/n,z:v.z/n,w:v.w/n};
  }else b.q=target;
}
function rememberSource(s:Sim,b:Bar) {s.source={p:[...b.p],entry:[...s.tool],yaw:b.q?Math.atan2(2*(b.q.w*b.q.y+b.q.x*b.q.z),1-2*(b.q.y*b.q.y+b.q.z*b.q.z)):b.yaw,...(b.q?{q:{...b.q}}:{})};s.released=false;}
/** Real bars vary in height: in each layer the outer bar nearest the robot is a hair thinner, so it carries no load. */
export function isLoose(tower:{x:number;z:number},p:Placement):boolean {
  const d=p.layer%2?ROBOT_BASE.x-tower.x:ROBOT_BASE.z-tower.z;
  return Math.abs(d)<.05?p.slot===1:p.slot===(d<0?0:2);
}
function release(s:Sim,b:Bar,p:Placement,toolY=(p.layer+1)*BAR.H+.0008) {
  if(s.released)return;
  b.p=[p.x,toolY-BAR.H/2,p.z];b.yaw=p.yaw;b.q={x:0,y:Math.sin(p.yaw/2),z:0,w:Math.cos(p.yaw/2)};
  b.layer=p.layer;b.slot=p.slot;b.inTower=true;s.released=true;b.loose=isLoose(s.tower,p);
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
  placeActive(s,placement(s.placed,s.tower),()=>{s.placed++;if(s.placed>=s.layers*3)s.phase='remove';});
}
/** End-grip carry of s.active from s.source onto placement p (used for building and restacking). */
function placeActive(s:Sim,p:Placement,done:()=>void) {
  s.grip=END_GRIP;
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const support=p.layer===0?0:Math.max(0,...s.bars.filter(item=>item.inTower&&item.layer===p.layer-1).map(item=>item.p[1]+BAR.H/2));
  const k=s.phaseT/CYCLE,pick:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]],dest:Vec=[p.x,support+BAR.H+.003,p.z];
  const lift:Vec=[pick[0],Math.min(maxToolHeight(pick[0],pick[2])-.01,Math.max(pick[1]+.22,dest[1]+.15)),pick[2]];
  const above:Vec=[dest[0],Math.min(maxToolHeight(dest[0],dest[2])-.01,dest[1]+.15),dest[2]];
  // Travel at lift height, then descend vertically: a diagonal path would plough through the feed stacks.
  const over:Vec=[dest[0],Math.min(maxToolHeight(dest[0],dest[2])-.01,Math.max(lift[1],above[1])),dest[2]];
  animate(s,[[0,lift,source.yaw,false],[.15,pick,source.yaw,false],[.22,pick,source.yaw,true],[.36,lift,source.yaw,true],[.56,over,p.yaw,true],[.66,above,p.yaw,true],[.8,dest,p.yaw,true],[.85,dest,p.yaw,false],[1,above,p.yaw,false]],k);
  if(s.carry)follow(s,b,smooth((k-.22)/.18));
  if(k>=.85)release(s,b,p,dest[1]);
  if(k>=1-1e-8){s.phaseT=0;s.active=-1;delete s.source;done();}
}
/** Tool poses of the push-then-pinch extraction, shared by planning and choreography. */
export const PUSH_STROKE=.085;
function extractionPoses(b:Bar,side:number):Vec[] {
  const a:Vec=[Math.cos(b.yaw)*side,0,-Math.sin(b.yaw)*side],y=b.p[1]+BAR.H/2;
  const at=(d:number):Vec=>[b.p[0]+a[0]*d,y,b.p[2]+a[2]*d],reach=BAR.L/2+TAB.L,grab=PUSH_STROKE+BAR.L/2-.012;
  return [at(-reach),at(-(reach-PUSH_STROKE)),at(grab),at(grab+.14)];
}
function beginRemoval(s:Sim) {
  // Only extractions where the arm never leans over the tower: one hand touches one bar.
  const tower=s.bars.filter(b=>b.inTower);
  const box={min:[Math.min(...tower.map(b=>b.p[0]))-ENVELOPE,0,Math.min(...tower.map(b=>b.p[2]))-ENVELOPE] as Vec,
    max:[Math.max(...tower.map(b=>b.p[0]))+ENVELOPE,Math.max(...tower.map(b=>b.p[1]))+BAR.H,Math.max(...tower.map(b=>b.p[2]))+ENVELOPE] as Vec};
  const sides=new Map<number,number[]>();
  for(const i of strategicCandidates(s)){
    const b=s.bars[i];if(!b)continue;
    const ok=[1,-1].filter(side=>!s.physics?.armClear||extractionPoses(b,side).every(p=>s.physics?.armClear?.(p,b.yaw+Math.PI/2,box)));
    if(ok.length)sides.set(i,ok);
  }
  // When no careful move remains, the robot still plays, and that last bar brings the tower down.
  const eligible=sides.size?[...sides.keys()]:strategicCandidates(s);
  const i=s.physics?.chooseRemoval?.(s,eligible)??eligible[0];
  if(i===undefined)return;
  const b=s.bars[i];if(!b)return;
  s.active=i;s.phaseT=0;s.removalHistory.push(i);rememberSource(s,b);delete s.grab;
  const ok=sides.get(i)??[1,-1];
  s.mode=s.random()<.5?'pull':'push';
  s.side=ok.length>1?(s.mode==='push'?1:-1):(ok[0]??1);
}
/** Tower envelope half-width: cross layers stick out beyond a bar's own end face. */
const ENVELOPE=SLOT+BAR.W/2;
function stepRemove(s:Sim) {
  if(s.restack>=0&&s.active<0){
    const b=s.bars[s.restack];if(!b){s.restack=-1;return;}
    s.active=s.restack;s.phaseT=0;rememberSource(s,b);
  }
  if(s.restack>=0){placeActive(s,placement(s.placed+s.moves,s.tower),()=>{s.moves++;s.restack=-1;});return;}
  if(s.active<0)beginRemoval(s);
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const k=s.phaseT/REMOVE_CYCLE,axis:Vec=[Math.cos(source.yaw)*s.side,0,-Math.sin(source.yaw)*s.side];
  const at=(o:Vec,d:number,y=o[1]):Vec=>[o[0]+axis[0]*d,y,o[2]+axis[2]*d];
  const c:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]];
  const roof=Math.max(...s.bars.filter(x=>x.inTower).map(x=>x.p[1]+BAR.H/2))+.12;
  const hi=(v:Vec):Vec=>[v[0],Math.min(maxToolHeight(v[0],v[2])-.01,roof),v[2]];
  // One hand only: a closed fingertip tab pushes the bar part-way, then the protruding end is pinched.
  const reach=BAR.L/2+TAB.L,stroke=PUSH_STROKE;
  const out=at(c,-(reach+.03)),contact=at(c,-reach),pushed=at(c,-(reach-stroke));
  if(!s.carry||!s.grab){const live:Vec=[b.p[0],b.p[1]+BAR.H/2,b.p[2]];s.grab=at(live,BAR.L/2-.012);}
  const grab=s.grab,pulled=at(grab,.14);
  const transit:Vec=[source.entry[0],Math.min(maxToolHeight(source.entry[0],source.entry[2])-.01,Math.max(roof,source.entry[1])),source.entry[2]];
  const yaw=source.yaw+Math.PI/2;
  // The withdrawn bar goes straight to the top, re-gripped by its ends; a floor drop only at the reach limit.
  const next=placement(s.placed+s.moves,s.tower);
  const toTop=canReach([next.x,(next.layer+1)*BAR.H+.02,next.z]);
  const floor:Vec=[pulled[0],BAR.H+.0008,pulled[2]];
  // Set the pinched bar down clear of the tower, then re-grip it by its ends for the top (neighbours leave no room for a side pinch).
  animate(s,[[0,source.entry,yaw,false],[.06,transit,yaw,false],[.13,hi(out),yaw,false],[.2,out,yaw,false],[.25,contact,yaw,false],[.45,pushed,yaw,false],[.49,out,yaw,false],[.53,hi(out),yaw,false],[.6,hi(grab),yaw,false],[.66,grab,yaw,false],[.69,grab,yaw,true],[.8,pulled,yaw,true],[.92,floor,yaw,true],[.96,floor,yaw,false],[1,hi(floor),yaw,false]],k);
  s.grip=k<.53?PUSH_GRIP:PINCH_GRIP;
  if(k>=.13&&k<.53)s.push={p:[s.tool[0]+axis[0]*TAB.L/2,s.tool[1]-s.grip.depth,s.tool[2]+axis[2]*TAB.L/2],yaw:source.yaw};else delete s.push;
  if(s.carry){follow(s,b,1,-Math.PI/2,(BAR.L/2-.012)*s.side);if(k>.75)b.inTower=false;}
  if(k>=.96)b.inTower=false;
  if(k>=1){
    s.phaseT=0;delete s.grab;delete s.push;
    if(toTop)s.restack=s.active;
    s.active=-1;delete s.source;
  }
}
export function chooseNextTower(s:Sim):{x:number;z:number} {
  const r=.9,candidates=[[1,0],[0,1],[-1,0],[0,-1]].map(([x,z])=>({x:ROBOT_BASE.x+(x??0)*r,z:ROBOT_BASE.z+(z??0)*r}));
  return candidates.filter(p=>Math.hypot(p.x-s.tower.x,p.z-s.tower.z)>.3)
    .sort((a,b)=>clearance(b,s)-clearance(a,s))[0]??{...TOWER};
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
  shoulder: [-0.125, 0.842, 0.203] as [number, number, number],
  elbow: [-0.125, 1.636, 0.998] as [number, number, number],
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
/** Headroom above a placement the wrist needs to descend vertically without straining the reach limit. */
export const REACH_CLEARANCE=.07;
export function reachableLayers(tower=TOWER):number {
  let count=0;
  for(let layer=0;layer<100;layer++){
    if([0,1,2].some(slot=>{const p=placement(layer*3+slot,tower);return !canReach([p.x,(layer+1)*BAR.H+REACH_CLEARANCE,p.z]);}))break;
    count++;
  }
  return count;
}
export const LAYERS=reachableLayers();
export const MAX_BARS=LAYERS*3;
