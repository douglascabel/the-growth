// Pure sculpture choreography and rules. Units: metres, Y up.
export const BAR = { L: 0.15, W: 0.05, H: 0.01 };
export const ROBOT_SCALE = 0.55;
export const ROBOT_BASE = { x: -0.32, y: 0.003181 * ROBOT_SCALE, z: -0.28 };
export const TOWER = { x: 0.34, z: 0.34 };
export const FEED = { x: -0.3, y: BAR.H * 5, z: 0.65 };
export const LAYERS = 18;
export const MAX_BARS = LAYERS * 3;
export const CYCLE = 2.5;
export const REMOVE_CYCLE = 4;
export type Vec = [number, number, number];
export type Placement = { x: number; z: number; yaw: number; layer: number; slot: number };
export type Bar = { p: Vec; yaw: number; pitch: number; roll: number; visible: boolean; layer: number; slot: number; q?: {x:number;y:number;z:number;w:number} };
export type Dynamics = { reset: () => void; step: (s: Sim, dt: number) => boolean };
export type Sim = {
  t: number; phase: 'build' | 'remove' | 'collapse' | 'rest'; phaseT: number;
  placed: number; bars: Bar[]; plan: Placement[]; carry: boolean; tool: Vec; toolYaw: number;
  active: number; moves: number; removalHistory: number[]; destination?: Placement;
  physics?: Dynamics; random: () => number;
};
export function placement(index: number): Placement {
  const layer = Math.floor(index / 3), slot = index % 3;
  const offset = (slot - 1) * BAR.W;
  return { x: TOWER.x + (layer % 2 ? offset : 0), z: TOWER.z + (layer % 2 ? 0 : offset), yaw: layer % 2 ? Math.PI / 2 : 0, layer, slot };
}
export function eligibleBars(s: Sim): number[] {
  const top = Math.max(...s.bars.filter(b => b.visible).map(b => b.layer));
  // Exclude the highest completed layer and all bars above it (standard Jenga rule).
  const completed = s.bars.filter(b => b.visible && b.layer === top).length === 3 ? top : top - 1;
  return s.bars.flatMap((b, i) => b.visible && b.layer < completed ? [i] : []);
}
export function createSim(random = Math.random): Sim {
  return { t: 0, phase: 'build', phaseT: 0, placed: 0, bars: Array.from({length: MAX_BARS}, () => ({p:[0,-10,0],yaw:0,pitch:0,roll:0,visible:false,layer:-1,slot:-1})), plan:Array.from({length:MAX_BARS},(_,i)=>placement(i)), carry:false,tool:[FEED.x,FEED.y+.25,FEED.z],toolYaw:Math.PI/2, active:-1,moves:0,removalHistory:[],random };
}
const smooth = (x:number) => {const t=Math.min(1,Math.max(0,x));return t*t*(3-2*t);};
const mix = (a:number,b:number,t:number) => a+(b-a)*t;
type Key = [number,Vec,number,boolean];
function animate(s:Sim, keys:Key[], k:number) {
  let j=0;
  while(j<keys.length-2 && k>(keys[j+1]?.[0] ?? 1)) j++;
  const from=keys[j], to=keys[j+1]; if(!from || !to) return;
  const u=smooth((k-from[0])/(to[0]-from[0]));
  s.tool=[mix(from[1][0],to[1][0],u),mix(from[1][1],to[1][1],u),mix(from[1][2],to[1][2],u)];
  s.toolYaw=mix(from[2],to[2],u);s.carry=from[3];
}
function seat(b:Bar,p:Placement) {
  b.p=[p.x,(p.layer+.5)*BAR.H,p.z]; b.yaw=p.yaw;b.layer=p.layer;b.slot=p.slot;b.visible=true;b.pitch=0;b.roll=0;b.q=undefined;
}
function follow(s:Sim,b:Bar) {b.visible=true;b.p=[s.tool[0],s.tool[1]-BAR.H/2,s.tool[2]];b.yaw=s.toolYaw;b.pitch=0;b.roll=0;b.q=undefined;}
function stepBuild(s:Sim) {
  const p=s.plan[s.placed], b=s.bars[s.placed]; if(!p||!b)return;
  s.active=s.placed; const k=s.phaseT/CYCLE;
  const pick:Vec=[FEED.x,FEED.y,FEED.z], dest:Vec=[p.x,(p.layer+1)*BAR.H,p.z];
  const above=(v:Vec):Vec=>[v[0],v[1]+.22,v[2]];
  animate(s,[[0,above(pick),Math.PI/2,false],[.12,pick,Math.PI/2,false],[.2,pick,Math.PI/2,true],[.32,above(pick),Math.PI/2,true],[.62,above(dest),p.yaw,true],[.76,dest,p.yaw,true],[.82,dest,p.yaw,false],[1,above(pick),Math.PI/2,false]],k);
  if(s.carry)follow(s,b);
  if(k>=.82)seat(b,p);
  if(s.phaseT>=CYCLE){seat(b,p);s.placed++;s.phaseT=0;s.active=-1;if(s.placed===MAX_BARS){s.phase='remove';s.physics?.reset();}}
}
function beginRemoval(s:Sim) {
  const eligible=eligibleBars(s); const i=eligible[Math.min(eligible.length-1,Math.floor(s.random()*eligible.length))];
  if(i===undefined){s.phase='collapse';return;}
  s.active=i;s.removalHistory.push(i);
  s.destination=placement(MAX_BARS+s.moves);
}
function stepRemove(s:Sim) {
  if(s.active<0)beginRemoval(s);
  const b=s.bars[s.active],dest=s.destination;if(!b||!dest)return;
  const k=s.phaseT/REMOVE_CYCLE;
  const origin:Vec=[b.p[0],b.p[1]+BAR.H/2,b.p[2]];
  // Store original position as a plan entry for this move so carrying cannot move its origin.
  if(k<.02)s.plan[s.active]={x:origin[0],z:origin[2],yaw:b.yaw,layer:b.layer,slot:b.slot};
  const source=s.plan[s.active];if(!source)return;
  const pick:Vec=[source.x,(source.layer+1)*BAR.H,source.z];
  const extract:Vec=[pick[0]+Math.cos(source.yaw)*.2,pick[1],pick[2]-Math.sin(source.yaw)*.2];
  const high:Vec=[extract[0],Math.max(.3,(dest.layer+1)*BAR.H+.18),extract[2]];
  const place:Vec=[dest.x,(dest.layer+1)*BAR.H,dest.z];
  animate(s,[[0,[pick[0],pick[1]+.18,pick[2]],source.yaw,false],[.16,pick,source.yaw,false],[.22,pick,source.yaw,true],[.43,extract,source.yaw,true],[.56,high,source.yaw,true],[.72,[place[0],high[1],place[2]],dest.yaw,true],[.82,place,dest.yaw,true],[.88,place,dest.yaw,false],[1,[place[0],place[1]+.2,place[2]],dest.yaw,false]],k);
  if(k>=.22 && k<.88){s.carry=true;follow(s,b);}
  if(k>=.88)seat(b,dest);
  if(s.phaseT>=REMOVE_CYCLE){seat(b,dest);s.moves++;s.phaseT=0;s.active=-1;s.destination=undefined;}
}
export function stepSim(s:Sim,dtRaw:number) {
  const dt=Math.max(0,Math.min(dtRaw,.05));s.t+=dt;s.phaseT+=dt;
  if(s.phase==='build')stepBuild(s);
  else if(s.phase==='remove')stepRemove(s);
  if((s.phase==='remove'||s.phase==='collapse') && s.physics?.step(s,dt) && s.phase==='remove'){s.phase='collapse';s.phaseT=0;s.carry=false;s.active=-1;}
  if(s.phase==='collapse'){s.tool=[ROBOT_BASE.x,.6,ROBOT_BASE.z+.3];if(s.phaseT>5){s.phase='rest';s.phaseT=0;}}
  else if(s.phase==='rest'&&s.phaseT>2){const physics=s.physics;physics?.reset();Object.assign(s,createSim(s.random),{t:s.t,physics});}
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
