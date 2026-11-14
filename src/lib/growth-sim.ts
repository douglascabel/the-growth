// Pure choreography. Metres, Y up; physics owns released bar poses.
export const BAR = { L: .155, W: .06214, H: .04145 };
// The lingot tapers (bottom ~154 x 61.5 mm, top ~125 x 49 mm). Slots are packed
// as tight as the bottoms allow so cross bars keep their centre over the next layer.
export const SLOT = .0635;
export const ROBOT_SCALE = .55;
// The uploaded picker OBJ has jaw centres at about +/-65 mm and moves each jaw 10 mm to grip.
export const GRIPPER_MODEL_HALF_SPAN = .065;
export const GRIPPER_CLOSE_TRAVEL = .01;
export const GRIPPER_OPEN_HALF_SPAN = GRIPPER_MODEL_HALF_SPAN * ROBOT_SCALE;
export const GRIPPER_CLOSED_HALF_SPAN = (GRIPPER_MODEL_HALF_SPAN - GRIPPER_CLOSE_TRAVEL) * ROBOT_SCALE;
export const GRIPPER_FINGER_HALF_WIDTH = .0297 * ROBOT_SCALE;
export const ROBOT_BASE = { x: -.32, y: .003181 * ROBOT_SCALE, z: -.28 };
// The tower sits on a robot axis so one bar orientation always runs sideways to the arm:
// both of its ends are reachable without the arm crossing over the tower.
export const TOWER = { x: ROBOT_BASE.x + .9, z: ROBOT_BASE.z };
export const FEED = { x: -.48, y: BAR.H, z: .45 };
export const CYCLE = 6;
export const REMOVE_CYCLE = 10;
const REMOVE_SETTLE_TIMEOUT = 8;
export type Vec = [number, number, number];
export type Quat = { x:number; y:number; z:number; w:number };
/** Finger half-opening (open/closed) and how far fingertips reach below the tool point. */
export type Grip = { open:number; closed:number; depth:number };
export const END_GRIP:Grip={open:GRIPPER_OPEN_HALF_SPAN,closed:GRIPPER_CLOSED_HALF_SPAN,depth:-.004};
export const PINCH_GRIP:Grip={open:GRIPPER_OPEN_HALF_SPAN,closed:GRIPPER_CLOSED_HALF_SPAN,depth:BAR.H*.5};
export const PUSH_GRIP:Grip={open:GRIPPER_OPEN_HALF_SPAN,closed:GRIPPER_CLOSED_HALF_SPAN,depth:BAR.H*.5};
/** Horizontal fingertip tab: lets a downward gripper push a bar end recessed below the cross layers. */
export const TAB = { L:.2, H:BAR.H*.64, W:.012 };
export type Placement = { x:number; z:number; yaw:number; layer:number; slot:number };
export type Bar = { p:Vec; yaw:number; pitch:number; roll:number; visible:boolean; layer:number; slot:number; inTower:boolean; q?:Quat; loose?:boolean };
export type Bounds={min:Vec;max:Vec};
export type Dynamics = { reset:()=>void; step:(s:Sim,dt:number)=>boolean; settled:()=>boolean; chooseRemoval?:(s:Sim,candidates:number[],routes?:{index:number;side:number}[])=>number|undefined; armClear?:(tool:Vec,yaw:number,box:Bounds)=>boolean; armClearMany?:(tool:Vec,yaw:number,boxes:Bounds[])=>boolean };
export type Sim = {
  t:number; phase:'settle'|'build'|'remove'|'collapse'|'rest'; phaseT:number;
  placed:number; bars:Bar[]; plan:Placement[]; carry:boolean; tool:Vec; toolYaw:number;
  active:number; moves:number; removalHistory:number[]; destination?:Placement;
  physics?:Dynamics; random:()=>number; tower:{x:number;z:number}; layers:number;
  generation:number; source?:{p:Vec;yaw:number;q?:Quat;entry:Vec}; released:boolean;
  mode:'pull'|'push'; side:number; push?:{p:Vec;yaw:number}; closed:boolean; recoveryHistory:number[];
  removalScores?:{index:number;stable:boolean;motion:number}[];
  grip:Grip; grab?:Vec; nextRemovalCheck:number;
};
export function placement(index:number, tower=TOWER):Placement {
  const layer=Math.floor(index/3),slot=index%3,offset=(slot-1)*SLOT;
  return {x:tower.x+(layer%2?offset:0),z:tower.z+(layer%2?0:offset),yaw:layer%2?Math.PI/2:0,layer,slot};
}
export function eligibleBars(s:Sim):number[] {
  const tower=s.bars.filter(b=>b.inTower);
  const layers=[...new Set(tower.map(b=>b.layer))];
  // A partial top row is not a story. Keep it and the highest complete story untouched.
  const highestCompleted=Math.max(-1,...layers.filter(layer=>tower.filter(b=>b.layer===layer).length===3));
  return s.bars.flatMap((b,i)=>b.inTower&&b.layer<highestCompleted?[i]:[]);
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
  return {t:0,phase:'settle',phaseT:0,placed:0,bars,plan:Array.from({length:MAX_BARS},(_,i)=>placement(i)),carry:false,tool:[FEED.x,.7,FEED.z],toolYaw:0,active:-1,moves:0,removalHistory:[],random,tower:{...TOWER},layers:LAYERS,generation:0,released:false,mode:'push',side:1,closed:false,recoveryHistory:[],grip:END_GRIP,nextRemovalCheck:0};
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
export function isLoose(_tower:{x:number;z:number},_p:Placement):boolean {
  // Construction uses uniform, full-height support contacts.  A deliberately
  // under-height bar made the completed tower lean before deconstruction began.
  return false;
}
function release(s:Sim,b:Bar,p:Placement,toolY=(p.layer+1)*BAR.H+.0008) {
  if(s.released)return;
  b.p=[p.x,toolY-BAR.H/2,p.z];b.yaw=p.yaw;b.q={x:0,y:Math.sin(p.yaw/2),z:0,w:Math.cos(p.yaw/2)};
  b.layer=p.layer;b.slot=p.slot;b.inTower=true;s.released=true;b.loose=false;
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
/** End-grip carry of s.active from s.source onto placement p. */
function placeActive(s:Sim,p:Placement,done:()=>void) {
  s.grip=END_GRIP;
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const k=s.phaseT/CYCLE,pick:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]],dest:Vec=[p.x,placementToolY(s,p),p.z];
  const lift:Vec=[pick[0],Math.min(maxToolHeight(pick[0],pick[2])-.01,Math.max(pick[1]+.22,dest[1]+.15)),pick[2]];
  const above:Vec=[dest[0],Math.min(maxToolHeight(dest[0],dest[2])-.01,dest[1]+.15),dest[2]];
  // Travel at lift height, then descend vertically: a diagonal path would plough through the feed stacks.
  const over:Vec=[dest[0],Math.min(maxToolHeight(dest[0],dest[2])-.01,Math.max(lift[1],above[1])),dest[2]];
  // Start at the exact pose left by the preceding move.  The previous sequence
  // started at `lift`, which made the arm visibly teleport from the stock.
  animate(s,[[0,source.entry,source.yaw,false],[.11,lift,source.yaw,false],[.23,pick,source.yaw,false],[.3,pick,source.yaw,true],[.44,lift,source.yaw,true],[.64,over,p.yaw,true],[.74,above,p.yaw,true],[.88,dest,p.yaw,true],[.93,dest,p.yaw,false],[1,above,p.yaw,false]],k);
  if(s.carry)follow(s,b,smooth((k-.3)/.14));
  if(k>=.93)release(s,b,p,dest[1]);
  if(k>=1-1e-8){s.phaseT=0;s.active=-1;delete s.source;done();}
}
/** A small closed-hand stroke tests a bar without converting it into a carried object. */
export const PUSH_STROKE=.018;
function extractionPoses(b:Bar,side:number,mode:'pull'|'push'):Vec[] {
  const a:Vec=[Math.cos(b.yaw)*side,0,-Math.sin(b.yaw)*side],y=b.p[1]+BAR.H/2;
  const at=(d:number):Vec=>[b.p[0]+a[0]*d,y,b.p[2]+a[2]*d],reach=BAR.L/2+TAB.L,grab=BAR.L/2-.012;
  // The picker approaches the end of the chosen bar, presses it smoothly, then
  // backs away.  It may start from either end, so alternate tower sides are valid.
  return mode==='pull'?[at(reach+.03),at(grab),at(grab+.14)]:[at(-(BAR.L/2+.09)),at(-(BAR.L/2+.012)),at(-(BAR.L/2-.012+PUSH_STROKE)),at(-(BAR.L/2+.09))];
}
function clearExtractionPath(path:Vec[],yaw:number,obstacles:Bounds[],armClear:Dynamics['armClear'],armClearMany:Dynamics['armClearMany']):boolean {
  for(let i=0;i<path.length-1;i++){
    const a=path[i],b=path[i+1];if(!a||!b)continue;
    const steps=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1],b[2]-a[2])/.02));
    for(let n=0;n<=steps;n++){
      const u=n/steps,p:Vec=[a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u,a[2]+(b[2]-a[2])*u];
      if(!canReach(p)||armClearMany&&!armClearMany(p,yaw,obstacles)||!armClearMany&&armClear&&obstacles.some(box=>!armClear(p,yaw,box)))return false;
    }
  }
  return true;
}
function barBounds(b:Bar):{min:Vec;max:Vec} {
  const q=b.q??{x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)}, {x,y,z,w}=q;
  const r00=1-2*(y*y+z*z),r01=2*(x*y-z*w),r02=2*(x*z+y*w);
  const r10=2*(x*y+z*w),r11=1-2*(x*x+z*z),r12=2*(y*z-x*w);
  const r20=2*(x*z-y*w),r21=2*(y*z+x*w),r22=1-2*(x*x+y*y);
  const hx=BAR.L/2,hy=BAR.H/2,hz=BAR.W/2;
  const ext:Vec=[Math.abs(r00)*hx+Math.abs(r01)*hy+Math.abs(r02)*hz,Math.abs(r10)*hx+Math.abs(r11)*hy+Math.abs(r12)*hz,Math.abs(r20)*hx+Math.abs(r21)*hy+Math.abs(r22)*hz];
  return {min:[b.p[0]-ext[0],b.p[1]-ext[1],b.p[2]-ext[2]],max:[b.p[0]+ext[0],b.p[1]+ext[1],b.p[2]+ext[2]]};
}
function beginRemoval(s:Sim) {
  // Deconstruction is a controlled closed-hand push.  It does not pick a bar up
  // or restack it, so all force reaches the tower through the selected bar.
  const tower=s.bars.filter(b=>b.inTower);
  const mode:'push'='push';
  const clearSides=new Map<number,number[]>(),reachableSides=new Map<number,number[]>(),ranked=strategicCandidates(s);
  for(const i of ranked.slice(0,12)){
    const b=s.bars[i];if(!b)continue;
    // Check the real gaps between bars, excluding the held bar that physics removes from robot collisions.
    const obstacles=tower.filter(item=>item!==b).map(barBounds);
    const sourceYaw=b.q?Math.atan2(2*(b.q.w*b.q.y+b.q.x*b.q.z),1-2*(b.q.y*b.q.y+b.q.z*b.q.z)):b.yaw;
    const reachable=[1,-1].filter(side=>extractionPoses(b,side,mode).every(canReach));
    if(reachable.length)reachableSides.set(i,reachable);
    const armClear=s.physics?.armClear,armClearMany=s.physics?.armClearMany;
    const clear=reachable.filter(side=>clearExtractionPath(extractionPoses(b,side,mode),sourceYaw+Math.PI/2,obstacles,armClear,armClearMany));
    if(clear.length)clearSides.set(i,clear);
    if(clearSides.size>=6)break;
  }
  // The capsule clearance is deliberately conservative around narrow tower gaps.
  // When it rejects every route, the Rapier pull probe below is the final safety gate.
  const sides=clearSides.size?clearSides:reachableSides;
  if(!sides.size)return;
  const eligible=[...sides.keys()];
  const routes=eligible.map(index=>{const options=sides.get(index)??[1,-1];return {index,side:options.length>1?(mode==='push'?1:-1):(options[0]??1)};});
  const i=s.physics?.chooseRemoval?s.physics.chooseRemoval(s,eligible,routes):eligible[0];
  if(i===undefined)return;
  const b=s.bars[i];if(!b)return;
  const ok=sides.get(i)??[1,-1];
  const side=ok.length>1?(mode==='push'?1:-1):(ok[0]??1);
  s.active=i;s.phaseT=0;s.removalHistory.push(i);rememberSource(s,b);delete s.grab;
  s.mode=mode;s.side=side;delete s.destination;
}
function stepRemove(s:Sim) {
  if(s.active<0){
    if(s.phaseT<s.nextRemovalCheck)return;
    beginRemoval(s);
    if(s.active<0)s.nextRemovalCheck=s.phaseT+15;
    else s.nextRemovalCheck=0;
  }
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const k=s.phaseT/REMOVE_CYCLE,axis:Vec=[Math.cos(source.yaw)*s.side,0,-Math.sin(source.yaw)*s.side];
  const at=(o:Vec,d:number,y=o[1]):Vec=>[o[0]+axis[0]*d,y,o[2]+axis[2]*d];
  const c:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]];
  const top=Math.max(...s.bars.filter(x=>x.inTower).map(x=>x.p[1]+BAR.H/2));
  const roof=top+BAR.H+.04;
  const hi=(v:Vec):Vec=>[v[0],Math.min(maxToolHeight(v[0],v[2])-.01,roof),v[2]];
  // Keep the hand closed and press only one exposed bar end.  The gripper enters
  // from the selected side, performs an 18 mm stroke, then retracts along it.
  const contact=at(c,-(BAR.L/2+.012)),pushed=at(contact,PUSH_STROKE),outPush=at(c,-(BAR.L/2+.09));
  const transit:Vec=[source.entry[0],Math.min(maxToolHeight(source.entry[0],source.entry[2])-.01,Math.max(roof,source.entry[1])),source.entry[2]];
  const sourceYaw=source.yaw+Math.PI/2;
  animate(s,[[0,source.entry,sourceYaw,false],[.09,transit,sourceYaw,false],[.2,hi(outPush),sourceYaw,false],[.3,outPush,sourceYaw,false],[.4,contact,sourceYaw,false],[.63,pushed,sourceYaw,false],[.78,contact,sourceYaw,false],[.9,outPush,sourceYaw,false],[1,hi(outPush),sourceYaw,false]],k);
  s.grip=PUSH_GRIP;s.closed=k>=.2&&k<.92;
  if(k>=.4&&k<.78)s.push={p:[...s.tool],yaw:source.yaw};
  else delete s.push;
  if(k>=1){
    s.phaseT=0;s.carry=false;s.closed=false;delete s.grab;delete s.push;delete s.source;delete s.destination;s.active=-1;s.nextRemovalCheck=0;
  }
}
function placementToolY(s:Sim,p:Placement):number {
  const support=p.layer===0?0:Math.max(0,...s.bars.filter(item=>item.inTower&&item.layer===p.layer-1).map(item=>item.p[1]+BAR.H/2));
  // Match the nearly full-height physical contact hull to avoid a visible drop
  // after every release, which accumulated into a leaning construction tower.
  return support+BAR.H*.99+.0008;
}
function nextRestackPlacement(s:Sim):Placement|undefined {
  const tower=s.bars.filter(item=>item.inTower);if(!tower.length)return undefined;
  const p=placement(s.placed+s.moves,s.tower),toolY=placementToolY(s,p),roof=Math.max(...tower.map(item=>item.p[1]+BAR.H/2));
  return canReach([p.x,toolY,p.z])&&maxToolHeight(p.x,p.z)>=roof+BAR.H+.05?p:undefined;
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
  // Rebuild Rapier from the recorded fallen poses; no bar is respawned or moved.
  s.physics?.reset();
  s.phase='build';s.phaseT=0;s.placed=0;s.active=-1;s.moves=0;s.carry=false;s.closed=false;delete s.source;delete s.destination;delete s.grab;delete s.push;s.nextRemovalCheck=0;
  s.plan=Array.from({length:MAX_BARS},(_,i)=>placement(i,s.tower));
}
export function stepSim(s:Sim,dtRaw:number) {
  const dt=Math.max(0,Math.min(dtRaw,.05));if(!dt)return;s.t+=dt;s.phaseT+=dt;
  if(s.phase==='settle'){
    if(s.phaseT>2&&(s.physics?.settled()??true)){s.phase='build';s.phaseT=0;}
  }else if(s.phase==='build')stepBuild(s);else if(s.phase==='remove'){
    // Let released contacts settle before inspecting the next support, but don't leave a
    // perpetually jittering tower frozen in a blank deconstruction phase.
    if(s.active>=0||s.phaseT>1.5&&((s.physics?.settled()??true)||s.phaseT>REMOVE_SETTLE_TIMEOUT))stepRemove(s);
  }
  const fallen=s.physics?.step(s,dt);
  if(fallen&&(s.phase==='remove'||s.phase==='build')&&s.placed>=6){s.phase='collapse';s.phaseT=0;s.carry=false;s.closed=false;s.active=-1;delete s.source;delete s.destination;delete s.grab;delete s.push;s.nextRemovalCheck=0;}
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

