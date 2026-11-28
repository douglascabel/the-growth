// Pure choreography. Metres, Y up; physics owns released bar poses.
export const BAR = { L: .155, W: .06214, H: .04145 };
/** Rapier contact cuboid, slightly inset so neighbours never start interpenetrating. */
export const BAR_CONTACT = { L: BAR.L * .985, H: BAR.H * .99, W: BAR.W * .985 };
// The lingot tapers (bottom ~154 x 61.5 mm, top ~125 x 49 mm). Slots are packed
// as tight as the bottoms allow so cross bars keep their centre over the next layer.
export const SLOT = .0635;
/** Half the tower footprint: outer cross bars reach SLOT + W/2 from the centre. */
export const TOWER_HALF = SLOT + BAR.W / 2;
export const ROBOT_SCALE = 1;
/** The KR 16 R2010 GLB faces +Z; the solver's rest forward is -Z. Renderer and IK share this yaw. */
export const ROBOT_MODEL_YAW = Math.PI;
export const ROBOT_BASE = { x: 0, y: 0, z: 0 };

// KR 16 R2010 lever gripper. Authored values are in the gripper's own frame;
// the hand is scaled so its rest opening (74 mm) clears a 62 mm bar.
export const GRIPPER_SCALE = 1.5;
const LEVER_T = .0216, LEVER_S = .0321; // lever pivot -> finger pivot (tool axis, jaw axis)
const LEVER_R = Math.hypot(LEVER_T, LEVER_S), LEVER_REST = Math.atan2(LEVER_S, LEVER_T);
/** Pad inner face to the jaw centre line, at the authored (fully open) pose. */
export const GRIPPER_OPEN_HALF_GAP = .0248 * GRIPPER_SCALE;
/** Pads close on the tapered bar flank at mid-height (~55.8 mm across). */
export const GRIPPER_CLOSED_HALF_GAP = .0279;
/** Pad thickness/2 across the jaw, and pad width/2 along the bar. */
export const GRIPPER_FINGER_RADIUS = .0065;
export const GRIPPER_FINGER_HALF_WIDTH = .0063 * GRIPPER_SCALE;
export const GRIPPER_JAW_SPEED = .08;
const PAD_HALF_LENGTH = .0064 * GRIPPER_SCALE;
/** A5 to the flange (152.9 mm), then the housing until the fingers emerge. */
export const WRIST_TO_FINGERS = .1529 + .0936 * GRIPPER_SCALE;
/** Parallel-lever jaw pose for a pad half-gap: lever angle and how far the pads advance along the tool axis. */
export function jawPose(halfGap:number):{angle:number;advance:number} {
  const ds=(halfGap-GRIPPER_OPEN_HALF_GAP)/GRIPPER_SCALE;
  const angle=Math.asin(Math.min(1,Math.max(-1,(LEVER_S+ds)/LEVER_R)))-LEVER_REST;
  return {angle,advance:(LEVER_R*Math.cos(LEVER_REST+angle)-LEVER_T)*GRIPPER_SCALE};
}
const CLOSED_ADVANCE = jawPose(GRIPPER_CLOSED_HALF_GAP).advance;
/** Tool point = closed pad centre backed off half a bar: with the tool on a bar's top face the pads hold its mid-height. */
export const GRIPPER_TOOL_OFFSET = CLOSED_ADVANCE - BAR.H / 2;
/** Pad tip depth below the tool point for a jaw half-gap. */
export function fingertipDepth(halfGap:number):number { return PAD_HALF_LENGTH + BAR.H / 2 + jawPose(halfGap).advance - CLOSED_ADVANCE; }
/** End grip: pad centre from the bar centre, leaving the pads clear of a neighbour's end once nudged out. */
export const END_GRIP_ALONG = BAR.L / 2 - .0155;
/** Bars are set down this far out along their length, beyond their neighbours' ends, then pushed home. */
export const NUDGE = .03;
/** Fingers stay this far outside a tower face. */
export const FINGER_CLEAR = GRIPPER_FINGER_HALF_WIDTH + .004;

// The tower sits on a robot axis so one bar orientation always runs sideways to the arm:
// both of its ends are reachable without the arm crossing over the tower.
export const TOWER = { x: ROBOT_BASE.x + 1.25, z: ROBOT_BASE.z };
/** The starting scatter lies beside the tower, inside the KR 16 floor reach. */
export const PILE = { x: .95, z: .8 };
export const REST_TOOL:[number,number,number] = [.9, .95, .35];
export const CYCLE = 8;
export const REMOVE_CYCLE = 16;
const REMOVE_SETTLE_TIMEOUT = 8;
export type Vec = [number, number, number];
export type Quat = { x:number; y:number; z:number; w:number };
export type RigProfile = {
  yaw:Vec; shoulder:Vec; elbow:Vec; wrist:Vec; roll:Vec; toolDrop:number;
  /** Rest angle of A5 -> tool in the forward/up plane (0 = forward, pi/2 = up). */
  toolRest:number;
  linkLengths:{ a1ToA2:number; a2ToA3:number; a3ToA5:number; a5ToTool:number };
};
export type RigBonePositions = { track:Vec; a1:Vec; a2:Vec; a3:Vec; a5:Vec; tool:Vec };
/** Gold bars are solid metal rigid bodies: gold density, high metal-on-metal friction, a dull clink. */
export const GOLD_DENSITY = 19300;
export const BAR_RESTITUTION = .02;
/** Steel-trowelled concrete floor against gold: slightly less grip than metal on metal, a low bounce. */
export const CONCRETE_FRICTION = .5;
export const CONCRETE_RESTITUTION = .18;
export type Placement = { x:number; z:number; yaw:number; layer:number; slot:number };
export type Bar = { p:Vec; yaw:number; pitch:number; roll:number; visible:boolean; layer:number; slot:number; inTower:boolean; q?:Quat; loose?:boolean };
export type Bounds={min:Vec;max:Vec};
export type Dynamics = { reset:()=>void; step:(s:Sim,dt:number)=>boolean; settled:()=>boolean; chooseRemoval?:(s:Sim,candidates:number[],routes?:{index:number;side:number}[])=>number|undefined;
  /** Incremental push probe of one bar; each next() advances a few physics steps. */
  probe?:(s:Sim,index:number,side:number)=>Generator<void,{index:number;stable:boolean;motion:number}|undefined,void>; armClear?:(tool:Vec,yaw:number,box:Bounds,rig?:RigProfile,tilt?:number)=>boolean; armClearMany?:(tool:Vec,yaw:number,boxes:Bounds[],rig?:RigProfile,tilt?:number)=>boolean };
export type Sim = {
  t:number; phase:'settle'|'build'|'remove'|'collapse'|'rest'; phaseT:number;
  placed:number; bars:Bar[]; plan:Placement[]; carry:boolean; tool:Vec; toolYaw:number;
  /** 0: tool points straight down; pi/2: it points horizontally, across its jaw axis. */
  toolTilt:number;
  active:number; moves:number; removalHistory:number[]; rig:RigProfile;
  physics?:Dynamics; random:()=>number; tower:{x:number;z:number}; layers:number;
  /** Hold (fixed when the jaws close): signed pad offset along the bar's +length axis, height and yaw relative to the tool. */
  generation:number; source?:{p:Vec;yaw:number;q?:Quat;entry:Vec;entryYaw:number;entryTilt:number;along?:number;dy?:number;yawOffset?:number;pushed?:number}; released:boolean;
  mode:'push'; side:number; pushing:boolean; closed:boolean; recoveryHistory:number[]; blocked:number;
  removalScores?:{index:number;stable:boolean;motion:number}[];
  /** In-progress removal choice and its per-frame time budget in ms (Infinity: decide at once). */
  planner?:Generator<void,void,void>; planBudget:number;
  /** Velocity a just-released bar starts falling with; the physics adapter consumes it. */
  releaseKick?:{index:number;v:Vec;w:Vec};
  /** Current pad half-gap; renderer and physics fingers both follow it. */
  jaw:number; nextRemovalCheck:number;
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
export function createSim(random=Math.random,rig:RigProfile=RIG):Sim {
  // A reproducible disorderly heap, already floor-adjacent, with no aligned supply columns.
  let seed=19371;
  const noise=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
  const layers=reachableLayers(TOWER,rig),barCount=layers*3,rows=Math.ceil(barCount/12);
  const bars:Bar[]=Array.from({length:barCount},(_,i)=>{
    const col=i%12,row=Math.floor(i/12),yaw=(noise()-.5)*Math.PI*2;
    const pitch=(noise()-.5)*.55,roll=(noise()-.5)*.7;
    const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2),cx=Math.cos(pitch/2),sx=Math.sin(pitch/2),cz=Math.cos(roll/2),sz=Math.sin(roll/2);
    return {p:[PILE.x+(col-5.5)*.065+(noise()-.5)*.04,.085+noise()*.08,PILE.z+(row-(rows-1)/2)*.07+(noise()-.5)*.05],yaw,pitch,roll,
      q:{x:sx*cy*cz+cx*sy*sz,y:cx*sy*cz-sx*cy*sz,z:cx*cy*sz+sx*sy*cz,w:cx*cy*cz-sx*sy*sz},visible:true,layer:-1,slot:-1,inTower:false};
  });
  return {t:0,phase:'settle',phaseT:0,placed:0,bars,plan:Array.from({length:barCount},(_,i)=>placement(i)),carry:false,tool:[...REST_TOOL],toolYaw:0,toolTilt:0,active:-1,moves:0,removalHistory:[],random,tower:{...TOWER},layers,generation:0,released:false,mode:'push',side:1,pushing:false,blocked:0,planBudget:Infinity,closed:false,recoveryHistory:[],jaw:GRIPPER_OPEN_HALF_GAP,nextRemovalCheck:0,rig};
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
/**
 * Which slot direction beside a tower bar is empty (+1 towards slot 2, -1 towards slot 0,
 * 0 for neither), preferring the tower's outside.
 */
export function freeSide(s:Sim,b:Bar):number {
  const mates=new Set(s.bars.filter(x=>x!==b&&x.inTower&&x.layer===b.layer).map(x=>x.slot));
  const free=(dir:number)=>!mates.has(b.slot+dir);
  if(b.slot!==1)return b.slot===2?1:-1;
  return free(1)?1:free(-1)?-1:0;
}
/** True when a bar runs across the arm's line of sight, so both ends are reachable beside the tower. */
export function lateral(p:Vec,yaw:number):boolean {
  const dx=p[0]-ROBOT_BASE.x,dz=p[2]-ROBOT_BASE.z,n=Math.hypot(dx,dz)||1;
  return Math.abs((Math.cos(yaw)*dx-Math.sin(yaw)*dz)/n)<.35;
}
export function strategicCandidates(s:Sim):number[] {
  // One fingertip pushes the end while the other finger needs a free lane beside the bar:
  // outer bars always have one, a middle bar once a neighbour has been pulled.
  const all=eligibleBars(s).filter(i=>{const b=s.bars[i];return !!b&&freeSide(s,b)!==0&&lateral(b.p,b.yaw);});
  // Start just below the protected top story.  A short push there has far less
  // load above it than a deep support, while the completed top remains intact.
  // Each support margin walks the whole tower: work it out once per bar, not per comparison.
  const margins=new Map(all.map(i=>[i,supportMargin(s,i)])),m=(i:number)=>margins.get(i)??0;
  const margin=(i:number)=>m(i)>.01?1:0;
  const loose=(i:number)=>s.bars[i]?.loose?1:0;
  return all.sort((a,b)=>margin(b)-margin(a)||loose(b)-loose(a)||(s.bars[b]?.layer??0)-(s.bars[a]?.layer??0)||m(b)-m(a));
}
const smooth=(x:number)=>{const t=Math.min(1,Math.max(0,x));return t*t*(3-2*t);};
const mix=(a:number,b:number,t:number)=>a+(b-a)*t;
/** Unit vector along a bar's +length axis for a yaw about +Y. */
export const barAxis=(yaw:number):Vec=>[Math.cos(yaw),0,-Math.sin(yaw)];
const along=(o:Vec,d:Vec,distance:number,y=o[1]):Vec=>[o[0]+d[0]*distance,y,o[2]+d[2]*distance];
const up=(v:Vec,dy:number):Vec=>[v[0],v[1]+dy,v[2]];
/** The equivalent of `angle` (mod 2pi) closest to `ref`, so the wrist never spins the long way round. */
const nearestAngle=(angle:number,ref:number)=>angle+Math.round((ref-angle)/(2*Math.PI))*2*Math.PI;
const yawOfQuat=(q:Quat)=>Math.atan2(2*(q.w*q.y+q.x*q.z),1-2*(q.y*q.y+q.z*q.z));
const yawQuat=(yaw:number):Quat=>({x:0,y:Math.sin(yaw/2),z:0,w:Math.cos(yaw/2)});
/** [time, tool point, tool yaw, carrying, tool tilt] */
type Key=[number,Vec,number,boolean,number];
function animate(s:Sim,keys:Key[],k:number) {
  let j=0;while(j<keys.length-2&&k>(keys[j+1]?.[0]??1))j++;
  const a=keys[j],b=keys[j+1];if(!a||!b)return;
  const u=smooth((k-a[0])/(b[0]-a[0]));
  s.tool=[mix(a[1][0],b[1][0],u),mix(a[1][1],b[1][1],u),mix(a[1][2],b[1][2],u)];s.toolYaw=mix(a[2],b[2],u);s.carry=a[3];s.toolTilt=mix(a[4],b[4],u);
}
/** Chain tool yaws so each key turns the wrist the short way from the one before. */
function chainYaws(keys:Key[]):Key[] { for(let i=1;i<keys.length;i++){const a=keys[i-1],b=keys[i];if(a&&b)b[2]=nearestAngle(b[2],a[2]);} return keys; }
/** Jaws straddle the bar width, so the bar's length axis is 90 degrees from the tool yaw. */
const BAR_FROM_TOOL=-Math.PI/2;
function follow(s:Sim,b:Bar,u:number,offset:number,dy=-BAR.H/2,yawOffset=BAR_FROM_TOOL) {
  b.yaw=s.toolYaw+yawOffset;
  const d=barAxis(b.yaw);
  b.p=[s.tool[0]-d[0]*offset,s.tool[1]+dy,s.tool[2]-d[2]*offset];
  // Reorient a fallen lingot smoothly during the lift, never flip it on the floor.
  const target=yawQuat(b.yaw),q=s.source?.q;
  if(q&&u<1){const sign=q.y*target.y+q.w*target.w<0?-1:1;
    const v={x:q.x*(1-u),y:q.y*(1-u)+target.y*u*sign,z:q.z*(1-u),w:q.w*(1-u)+target.w*u*sign};
    const n=Math.hypot(v.x,v.y,v.z,v.w);b.q={x:v.x/n,y:v.y/n,z:v.z/n,w:v.w/n};
  }else b.q=target;
}
function rememberSource(s:Sim,b:Bar) {s.source={p:[...b.p],entry:[...s.tool],entryYaw:s.toolYaw,entryTilt:s.toolTilt,yaw:b.q?yawOfQuat(b.q):b.yaw,...(b.q?{q:{...b.q}}:{})};s.released=false;}
/** Closed pad centre along the bar's length axis, measured from its centre. */
function padAlong(s:Sim,b:Bar):number {
  const {t}=toolAxes(s.toolYaw,s.toolTilt),d=barAxis(b.yaw),pad=[s.tool[0]+t[0]*BAR.H/2,0,s.tool[2]+t[2]*BAR.H/2];
  return (pad[0]!-b.p[0])*d[0]+(pad[2]!-b.p[2])*d[2];
}
/** Lock the hold offset from the bar's actual pose the moment the jaws close, so it never jumps. */
function grip(s:Sim,b:Bar):number {
  const source=s.source;if(!source)return 0;
  if(source.along===undefined){
    const d=barAxis(b.yaw);source.along=(s.tool[0]-b.p[0])*d[0]+(s.tool[2]-b.p[2])*d[2];
    source.dy=b.p[1]-s.tool[1];source.yawOffset=b.yaw-s.toolYaw;
  }
  return source.along;
}
/** Real bars vary in height: in each layer the outer bar nearest the robot is a hair thinner, so it carries no load. */
export function isLoose(_tower:{x:number;z:number},_p:Placement):boolean {
  // Construction uses uniform, full-height support contacts.  A deliberately
  // under-height bar made the completed tower lean before deconstruction began.
  return false;
}
function release(s:Sim,b:Bar,p:Placement,centre:Vec,yaw:number) {
  if(s.released)return;
  b.p=[...centre];b.yaw=yaw;b.q=yawQuat(yaw);
  b.layer=p.layer;b.slot=p.slot;b.inTower=true;s.released=true;b.loose=false;
}
/**
 * Set-down and push-home poses for a bar held `offset` from its centre. The held end
 * faces the robot; the bar lands NUDGE further out so the fingers stay clear of its
 * neighbours' ends, then the closed hand's tab pushes it home.
 */
export function settlePoses(p:{x:number;z:number;yaw:number},offset:number,toolY:number) {
  const toRobot=barAxis(p.yaw),c:Vec=[p.x,toolY,p.z];
  const faces=toRobot[0]*(ROBOT_BASE.x-p.x)+toRobot[2]*(ROBOT_BASE.z-p.z)>=0;
  const d:Vec=faces?toRobot:[-toRobot[0],0,-toRobot[2]];
  // Bar yaw whose +axis times sign(offset) points at the held end.
  const barYaw=(faces?p.yaw:p.yaw+Math.PI)+(offset<0?Math.PI:0);
  const reach=Math.abs(offset),end=BAR_CONTACT.L/2;
  const set=along(c,d,NUDGE+reach),clear=up(set,.06);
  // The jaws turn along the bar, so the leading closed finger meets its end on the centre line.
  const lead=end+GRIPPER_FINGER_RADIUS+FINGER_SPREAD;
  const outLow=along(c,d,lead+NUDGE+.015),outHigh=[outLow[0],clear[1],outLow[2]] as Vec;
  const home=along(c,d,lead);
  return {barYaw,centre:along(c,d,NUDGE,toolY-BAR.H/2),set,clear,outHigh,outLow,home,pushYaw:yawOf(d)};
}
/**
 * Friction of the selected bar while it is slid: extracted, or pushed home after a set-down.
 * Full friction shears the whole compliant stack with it, so bars spring back short or ride along.
 */
export const SLIDE_FRICTION=.04;
export const BAR_FRICTION=.62;
/** Closed fingers sit this far either side of the tool along the jaw axis. */
export const FINGER_SPREAD=GRIPPER_CLOSED_HALF_GAP+GRIPPER_FINGER_RADIUS;
/** Yaw of a horizontal unit vector, the inverse of barAxis. */
export const yawOf=(d:Vec)=>Math.atan2(-d[2],d[0]);
/** Tool yaw that points a horizontal tool along unit `d`; its jaws then open horizontally across it. */
export const toolYawFacing=(d:Vec)=>yawOf(d)-Math.PI/2;
function stepBuild(s:Sim) {
  if(s.active<0){
    const candidates=s.bars.flatMap((b,i)=>!b.inTower&&canReach(pickPoint(b),s.rig)?[i]:[]);
    // Recover the topmost accessible bar first, leaving the rest of the pile untouched.
    candidates.sort((a,b)=>(s.bars[b]?.p[1]??0)-(s.bars[a]?.p[1]??0));
    const i=candidates[0];if(i===undefined){s.phase='remove';s.phaseT=0;return;}
    const bar=s.bars[i];if(!bar)return;s.active=i;rememberSource(s,bar);
    s.recoveryHistory.push(i);
  }
  // Construction keeps nominal slots: copying each layer's drift into the next would compound a lean.
  placeActive(s,placement(s.placed,s.tower),()=>{s.placed++;if(s.placed>=s.layers*3)s.phase='remove';});
}
/** End grip on the bar end nearer the robot. */
function pickPoint(b:Bar):Vec {
  const yaw=b.q?yawOfQuat(b.q):b.yaw,d=barAxis(yaw),toRobot=d[0]*(ROBOT_BASE.x-b.p[0])+d[2]*(ROBOT_BASE.z-b.p[2])>=0?1:-1;
  return along([b.p[0],b.p[1]+BAR.H/2,b.p[2]],d,END_GRIP_ALONG*toRobot);
}
const clampHeight=(v:Vec,rig:RigProfile,y:number):Vec=>[v[0],Math.min(maxToolHeight(v[0],v[2],rig)-.01,y),v[2]];
/** End-grip carry of s.active from s.source onto placement p, then a short push home. */
function placeActive(s:Sim,p:Placement,done:()=>void) {
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const k=s.phaseT/CYCLE,pick=pickPoint({...b,p:source.p,yaw:source.yaw,...(source.q?{q:source.q}:{})});
  const pickYaw=nearestAngle(source.yaw-BAR_FROM_TOOL,source.entryYaw);
  const d0=barAxis(source.yaw),offset=(pick[0]-source.p[0])*d0[0]+(pick[2]-source.p[2])*d0[2];
  const toolY=placementToolY(s,p),pose=settlePoses(p,offset,toolY);
  // Jaws are symmetric: turn them along the bar by the shorter quarter turn.
  const placeYaw=nearestAngle(pose.barYaw-BAR_FROM_TOOL,pickYaw),pushYaw=pose.pushYaw+Math.round((placeYaw-pose.pushYaw)/Math.PI)*Math.PI;
  const lift=clampHeight(pick,s.rig,Math.max(pick[1]+.32,toolY+.25));
  const above=clampHeight(pose.set,s.rig,toolY+.15);
  // Travel at lift height, then descend vertically: a diagonal path would plough through the scatter.
  const over=clampHeight(pose.set,s.rig,Math.max(lift[1],above[1]));
  // Start at the exact pose left by the preceding move, so the arm never teleports.
  animate(s,[[0,source.entry,source.entryYaw,false,source.entryTilt],[.1,lift,pickYaw,false,0],[.18,pick,pickYaw,false,0],[.22,pick,pickYaw,true,0],[.32,lift,pickYaw,true,0],
    [.48,over,placeYaw,true,0],[.55,above,placeYaw,true,0],[.63,pose.set,placeYaw,true,0],[.66,pose.set,placeYaw,false,0],[.7,pose.clear,placeYaw,false,0],
    [.74,pose.outHigh,pushYaw,false,0],[.78,pose.outLow,pushYaw,false,0],[.88,pose.home,pushYaw,false,0],[.92,pose.outLow,pushYaw,false,0],[1,clampHeight(pose.outLow,s.rig,toolY+.15),pushYaw,false,0]],k);
  s.closed=k>=.2&&k<.66||k>=.76&&k<.92;
  if(s.carry){source.along=offset;follow(s,b,smooth((k-.22)/.1),offset);}
  if(k>=.66)release(s,b,p,pose.centre,pose.barYaw);
  // The leading fingertip slides the bar home; physics gives the fingers to this bar alone.
  s.pushing=k>=.78&&k<.88;
  if(k>=1-1e-8){s.phaseT=0;s.active=-1;s.closed=false;s.pushing=false;delete s.source;done();}
}
/** One-fingertip stroke: long enough that the far end clears the cross layers for the pads. */
export const PUSH_STROKE=.05;
/** The pads need this much of the pushed end beyond its neighbours' recessed ends. */
const MIN_PROTRUSION=.024;
/** Pulled bars are let go once their inner end is this far clear of the tower face. */
const PULL_CLEAR=.15;
const TIP=fingertipDepth(GRIPPER_CLOSED_HALF_GAP);
/**
 * Side push and opposite pull for an outer bar, all at its mid-height with the tool horizontal.
 * `c` is the bar centre, `a` the push direction, `out` points away from the tower across the bar.
 * Each pose carries the horizontal tool direction.
 */
export function removalPoses(c:Vec,a:Vec,out:Vec,pushed=PUSH_STROKE) {
  // One closed finger meets the recessed end on the bar's centre line; the other stays outside the tower.
  const contactAlong=-(BAR_CONTACT.L/2+TIP+GRIPPER_FINGER_RADIUS+.003),lateral=FINGER_SPREAD+.004;
  const side=(v:Vec):Vec=>[v[0]+out[0]*lateral,v[1],v[2]+out[2]*lateral];
  const back:Vec=[-a[0],0,-a[2]];
  // The pads close on the end that now protrudes, between the cross layers above and below
  // and clear of the neighbours' recessed ends; the tool point sits half a bar behind them.
  const pad=pushed+BAR.L/2-(TIP-BAR.H/2)-.002,grip=pad+BAR.H/2;
  return {
    away:{p:side(along(c,a,contactAlong-.08)),t:a},contact:{p:side(along(c,a,contactAlong)),t:a},pushed:{p:side(along(c,a,contactAlong+PUSH_STROKE)),t:a},
    approach:{p:along(c,a,grip+.06),t:back},pinch:{p:along(c,a,grip),t:back},
    pulled:{p:along(c,a,grip+TOWER_HALF+PULL_CLEAR+BAR.L/2-pushed),t:back},
  };
}
function removalFrame(s:Sim,b:Bar,side:number,pushed=PUSH_STROKE) {
  const yaw=b.q?yawOfQuat(b.q):b.yaw,a=barAxis(yaw).map(v=>v*side) as Vec,perp:Vec=[-a[2],0,a[0]];
  const dot=(v:Vec,x:number,z:number)=>v[0]*x+v[2]*z;
  // Slots run along +X on odd layers and +Z on even ones; point `out` at the free lane.
  const slotAxis:Vec=b.layer%2?[1,0,0]:[0,0,1],free=freeSide(s,b)||1;
  const out:Vec=dot(perp,slotAxis[0]*free,slotAxis[2]*free)>=0?perp:[-perp[0],0,-perp[2]];
  // The hand crosses from the push side to the pull side over the top of the tower.
  const roofY=Math.max(...s.bars.filter(x=>x.inTower).map(x=>x.p[1]+BAR.H/2))+.08;
  const poses=removalPoses([...b.p],a,out,pushed),high=(pose:{p:Vec;t:Vec})=>({p:[pose.p[0],roofY,pose.p[2]] as Vec,t:pose.t});
  return {a,roofY,poses,awayHigh:high(poses.away),approachHigh:high(poses.approach),over:clampHeight([b.p[0],roofY+.1,b.p[2]],s.rig,roofY+.1)};
}
function extractionPoses(s:Sim,b:Bar,side:number):{p:Vec;t:Vec}[] {
  const {poses,awayHigh,approachHigh}=removalFrame(s,b,side);
  return [awayHigh,poses.away,poses.contact,poses.pushed,approachHigh,poses.approach,poses.pinch,poses.pulled];
}
function* clearExtractionPath(path:{p:Vec;t:Vec}[],obstacles:Bounds[],armClear:Dynamics['armClear'],armClearMany:Dynamics['armClearMany'],rig:RigProfile):Generator<void,boolean,void> {
  for(let i=0;i<path.length-1;i++){
    yield;
    const a=path[i],b=path[i+1];if(!a||!b)continue;
    const steps=Math.max(1,Math.ceil(Math.hypot(b.p[0]-a.p[0],b.p[1]-a.p[1],b.p[2]-a.p[2])/.02));
    for(let n=0;n<=steps;n++){
      if(n)yield;
      const u=n/steps,p:Vec=[a.p[0]+(b.p[0]-a.p[0])*u,a.p[1]+(b.p[1]-a.p[1])*u,a.p[2]+(b.p[2]-a.p[2])*u],yaw=toolYawFacing(u<.5?a.t:b.t);
      if(!canReachPose(p,yaw,Math.PI/2,rig)||armClearMany&&!armClearMany(p,yaw,obstacles,rig,Math.PI/2)||!armClearMany&&armClear&&obstacles.some(box=>!armClear(p,yaw,box,rig,Math.PI/2)))return false;
    }
  }
  return true;
}
function barBounds(b:Bar):{min:Vec;max:Vec} {
  const q=b.q??yawQuat(b.yaw), {x,y,z,w}=q;
  const r00=1-2*(y*y+z*z),r01=2*(x*y-z*w),r02=2*(x*z+y*w);
  const r10=2*(x*y+z*w),r11=1-2*(x*x+z*z),r12=2*(y*z-x*w);
  const r20=2*(x*z-y*w),r21=2*(y*z+x*w),r22=1-2*(x*x+y*y);
  const hx=BAR.L/2,hy=BAR.H/2,hz=BAR.W/2;
  const ext:Vec=[Math.abs(r00)*hx+Math.abs(r01)*hy+Math.abs(r02)*hz,Math.abs(r10)*hx+Math.abs(r11)*hy+Math.abs(r12)*hz,Math.abs(r20)*hx+Math.abs(r21)*hy+Math.abs(r22)*hz];
  return {min:[b.p[0]-ext[0],b.p[1]-ext[1],b.p[2]-ext[2]],max:[b.p[0]+ext[0],b.p[1]+ext[1],b.p[2]+ext[2]]};
}
/** After this many fruitless checks the robot takes the least risky move, or unstacks what stands. */
const PATIENCE=3;
export type RemovalScore={index:number;stable:boolean;motion:number};
/**
 * Choose the next extraction a little at a time: the route checks and Rapier probes are
 * a generator the robot advances within a per-frame budget, so choosing never stalls a frame.
 */
function* planRemoval(s:Sim):Generator<void,void,void> {
  // Deconstruction: push a bar's end from the side with one fingertip, cross over the
  // tower, grip the end that now protrudes, pull it straight out and let it fall.
  const tower=s.bars.filter(b=>b.inTower);
  // A person's choice is not a strict ranking: jitter it so any reasonable bar can come next.
  yield;
  const ranked=strategicCandidates(s);
  yield;
  const pool=ranked.slice(0,16).map((index,rank)=>({index,key:rank+s.random()*CHOICE_SPREAD}))
    .sort((a,b)=>a.key-b.key).map(item=>item.index);
  const clearSides=new Map<number,number[]>(),reachableSides=new Map<number,number[]>();
  for(const i of pool){
    const b=s.bars[i];if(!b)continue;
    // Check the real gaps between bars, excluding the bar the hand works on.
    const obstacles=tower.filter(item=>item!==b).map(barBounds);
    const reachable=[1,-1].filter(side=>extractionPoses(s,b,side).every(pose=>canReachPose(pose.p,toolYawFacing(pose.t),Math.PI/2,s.rig)));
    if(reachable.length)reachableSides.set(i,reachable);
    const armClear=s.physics?.armClear,armClearMany=s.physics?.armClearMany,clear:number[]=[];
    for(const side of reachable){if(yield* clearExtractionPath(extractionPoses(s,b,side),obstacles,armClear,armClearMany,s.rig))clear.push(side);}
    if(clear.length)clearSides.set(i,clear);
    if(clearSides.size>=6)break;
  }
  // The capsule clearance is deliberately conservative around narrow tower gaps.
  // When it rejects every route, the Rapier push probe below is the final safety gate.
  const sides=clearSides.size?clearSides:reachableSides;
  if(!sides.size){if(s.blocked>=PATIENCE){s.phase='rest';s.phaseT=0;}return;}
  const eligible=[...sides.keys()];
  // Either end may be pushed when both routes are open.
  const routes=eligible.map(index=>{const options=sides.get(index)??[1,-1];return {index,side:options[options.length>1&&s.random()<.5?1:0]??1};});
  let i:number|undefined;
  const probe=s.physics?.probe;
  if(probe){
    const scores:RemovalScore[]=[];
    for(const route of routes.slice(0,PROBES)){
      const run=probe(s,route.index,route.side);let step=run.next();
      while(!step.done){yield;step=run.next();}
      if(step.value)scores.push(step.value);
    }
    s.removalScores=scores;
    // Among the moves the probe calls safe, prefer gentler ones but leave room for chance.
    const stable=scores.filter(score=>score.stable),weights=stable.map(score=>1/(score.motion+.01));
    let pick=s.random()*weights.reduce((sum,w)=>sum+w,0);
    i=stable.find((_,n)=>(pick-=weights[n]??0)<=0)?.index??stable[0]?.index;
  }else i=s.physics?.chooseRemoval?s.physics.chooseRemoval(s,eligible,routes):eligible[0];
  // Nothing passes the stability probe any more: make the gentlest move and let physics decide.
  if(i===undefined&&s.blocked>=PATIENCE)i=s.removalScores?.slice().sort((a,b)=>a.motion-b.motion)[0]?.index??eligible[0];
  if(i===undefined)return;
  const b=s.bars[i];if(!b)return;
  s.active=i;s.phaseT=0;s.removalHistory.push(i);rememberSource(s,b);s.blocked=0;
  s.mode='push';s.side=routes.find(route=>route.index===i)?.side??1;
}
/** How many ranks a candidate may jump in the jittered order. */
const CHOICE_SPREAD=6;
/** Physics probes per decision. */
const PROBES=3;
const now=()=>globalThis.performance?.now()??Date.now();
function stepRemove(s:Sim) {
  if(s.active<0){
    if(!s.planner){if(s.phaseT<s.nextRemovalCheck)return;s.planner=planRemoval(s);}
    const until=now()+s.planBudget;let done=false;
    for(;;){if(s.planner.next().done){done=true;break;}if(now()>=until)break;}
    if(!done)return;
    delete s.planner;
    // Retry soon after normal contact settling instead of leaving the arm parked
    // at the construction handoff when a candidate is temporarily unavailable.
    if(s.active<0){s.nextRemovalCheck=s.phaseT+2;s.blocked++;}
    else s.nextRemovalCheck=0;
  }
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const k=s.phaseT/REMOVE_CYCLE,frame=(pushed?:number)=>removalFrame(s,{...b,p:source.p,...(source.q?{q:source.q}:{})},s.side,pushed);
  // Once the finger is back, measure how far the bar really moved: contacts spring back a little.
  if(k>=.41&&source.pushed===undefined){
    const d=frame().a;source.pushed=(b.p[0]-source.p[0])*d[0]+(b.p[2]-source.p[2])*d[2];
    if(source.pushed<MIN_PROTRUSION){
      // Too little sticks out to grip without touching a neighbour: retreat and choose again.
      s.phaseT=0;s.carry=false;s.closed=false;s.pushing=false;delete s.source;s.active=-1;s.nextRemovalCheck=0;return;
    }
  }
  const {a,roofY,poses,awayHigh,approachHigh,over}=frame(source.pushed);
  const H=Math.PI/2,f=(pose:{p:Vec;t:Vec},carry:boolean,time:number):Key=>[time,pose.p,toolYawFacing(pose.t),carry,H];
  const released=poses.pulled.p,retreat:Vec=[released[0],Math.max(roofY,released[1]+.15),released[2]];
  // Over the top the hand points down, its jaws turned half way between the two sides.
  const overYaw=toolYawFacing(a)+Math.PI/2;
  animate(s,chainYaws([[0,source.entry,source.entryYaw,false,source.entryTilt],f(awayHigh,false,.1),
    f(poses.away,false,.16),f(poses.contact,false,.21),f(poses.pushed,false,.33),f(poses.contact,false,.37),f(poses.away,false,.41),
    f(awayHigh,false,.46),[.52,over,overYaw,false,0],f(approachHigh,false,.58),f(poses.approach,false,.63),
    f(poses.pinch,false,.66),f(poses.pinch,true,.69),f(poses.pulled,true,.8),f(poses.pulled,false,.82),[1,retreat,toolYawFacing(poses.pulled.t),false,H]]),k);
  s.closed=k>=.15&&k<.41||k>=.67&&k<.82;
  s.pushing=k>=.19&&k<.37;
  if(s.carry&&source.along===undefined&&Math.abs(padAlong(s,b))>BAR.L/2-GRIPPER_FINGER_HALF_WIDTH){
    // The pushed bar sprang back: the pads would close beside its end, not on it.
    // Leave it in the tower and choose again from the current hand pose.
    s.phaseT=0;s.carry=false;s.closed=false;s.pushing=false;delete s.source;s.active=-1;s.nextRemovalCheck=0;return;
  }
  if(s.carry)follow(s,b,1,grip(s,b),source.dy,source.yawOffset);
  if(k>=.82&&!s.released){
    // Let go: the bar is out of the tower and drops to the floor under gravity. The pads
    // free the held end a moment late, so the far end noses down and it carries a little
    // of the pull; no two drops are alike.
    s.released=true;b.inTower=false;b.layer=-1;b.slot=-1;
    const held=Math.sign(source.along??1),d=barAxis(b.yaw),free:Vec=[-d[0]*held,0,-d[2]*held];
    const pitch=1.5+s.random()*2.5,roll=(s.random()-.5)*2,drift=.08+s.random()*.15;
    s.releaseKick={index:s.active,v:[a[0]*drift,-.05,a[2]*drift],
      // up x free tips the free end down; a little roll about the length adds a twist.
      w:[-free[2]*pitch+d[0]*roll,0,free[0]*pitch+d[2]*roll]};
  }
  if(k>=1){
    s.phaseT=0;s.carry=false;s.closed=false;s.pushing=false;delete s.source;s.active=-1;s.nextRemovalCheck=0;
  }
}
function placementToolY(s:Sim,p:Placement,exclude?:Bar):number {
  const support=p.layer===0?0:Math.max(0,...s.bars.filter(item=>item!==exclude&&item.inTower&&item.layer===p.layer-1).map(item=>item.p[1]+BAR.H/2));
  // Match the nearly full-height physical contact hull to avoid a visible drop
  // after every release, which accumulated into a leaning construction tower.
  return support+BAR.H*.99+.0008;
}
export function chooseNextTower(s:Sim):{x:number;z:number} {
  const r=.95,candidates=[[1,0],[0,1],[-1,0],[0,-1]].map(([x,z])=>({x:ROBOT_BASE.x+(x??0)*r,z:ROBOT_BASE.z+(z??0)*r}));
  return candidates.filter(p=>Math.hypot(p.x-s.tower.x,p.z-s.tower.z)>.3)
    .sort((a,b)=>clearance(b,s)-clearance(a,s))[0]??{...TOWER};
}
function clearance(p:{x:number;z:number},s:Sim){return Math.min(...s.bars.map(b=>Math.hypot(p.x-b.p[0],p.z-b.p[2])));}
export function restartFromPile(s:Sim) {
  s.tower=chooseNextTower(s);s.layers=Math.min(reachableLayers(s.tower,s.rig),Math.floor(s.bars.length/3));s.generation++;
  for(const b of s.bars){b.inTower=false;b.layer=-1;b.slot=-1;}
  // Rebuild Rapier from the recorded fallen poses; no bar is respawned or moved.
  s.physics?.reset();
  s.phase='build';s.phaseT=0;s.placed=0;s.active=-1;s.moves=0;s.carry=false;s.closed=false;delete s.source;delete s.planner;s.pushing=false;s.blocked=0;s.nextRemovalCheck=0;
  s.plan=Array.from({length:s.bars.length},(_,i)=>placement(i,s.tower));
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
  const target=s.carry||s.closed?GRIPPER_CLOSED_HALF_GAP:GRIPPER_OPEN_HALF_GAP,travel=GRIPPER_JAW_SPEED*dt;
  s.jaw=Math.abs(target-s.jaw)<=travel?target:s.jaw+Math.sign(target-s.jaw)*travel;
  const fallen=s.physics?.step(s,dt);
  if(fallen&&(s.phase==='remove'||s.phase==='build')&&s.placed>=6){s.phase='collapse';s.phaseT=0;s.carry=false;s.closed=false;s.active=-1;delete s.source;delete s.planner;s.pushing=false;s.blocked=0;s.nextRemovalCheck=0;}
  if(s.phase==='collapse'&&s.phaseT>4&&(s.physics?.settled()??true)){s.phase='rest';s.phaseT=0;}
  else if(s.phase==='rest'&&s.phaseT>2)restartFromPile(s);
}
/** Convert the GLB's live Y-up pivots into the solver's -Z-forward frame through ROBOT_MODEL_YAW. */
export function rigProfileFromBones(bones:RigBonePositions):RigProfile {
  const c=Math.cos(ROBOT_MODEL_YAW),sn=Math.sin(ROBOT_MODEL_YAW);
  const toSolver=([x,y,z]:Vec):Vec=>{const dx=x-bones.track[0],dz=z-bones.track[2];return [dx*c+dz*sn,y-bones.track[1],-dx*sn+dz*c];};
  const wrist=toSolver(bones.a5),tool=toSolver(bones.tool);
  // A2 and A3 pivots can sit anywhere along their lateral pitch axes; only the
  // wrist line fixes the arm's lateral offset from A1.
  const onArmPlane=([,y,z]:Vec):Vec=>[wrist[0],y,z];
  const yaw=toSolver(bones.a1),shoulder=onArmPlane(toSolver(bones.a2)),elbow=onArmPlane(toSolver(bones.a3));
  const distance=(a:Vec,b:Vec)=>Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
  const linkLengths={
    a1ToA2:distance(yaw,shoulder),
    a2ToA3:distance(shoulder,elbow),
    a3ToA5:distance(elbow,wrist),
    a5ToTool:distance(wrist,tool),
  };
  if(Object.values(linkLengths).some(length=>!Number.isFinite(length)||length<=0))throw new Error('KUKA GLB has a zero-length or invalid rig link');
  return {yaw,shoulder,elbow,wrist,roll:tool,toolDrop:linkLengths.a5ToTool,toolRest:Math.atan2(tool[1]-wrist[1],-(tool[2]-wrist[2])),linkLengths};
}
/** Default profile mirrors the KR 16 R2010 GLB pivots (and scaled gripper) until its live nodes load. */
export const RIG=rigProfileFromBones({
  track:[0,0,0],a1:[0,.2845,0],a2:[-.1048,.52,.16],a3:[-.1251,1.5,.16],a5:[0,1.65,1.02],
  tool:[0,1.65,1.02+.1529+.1625*GRIPPER_SCALE+GRIPPER_TOOL_OFFSET],
});
function rigDimensions(rig:RigProfile) {
  const S_F=-(rig.shoulder[2]-rig.yaw[2]),S_Y=rig.shoulder[1];
  const E_F=-(rig.elbow[2]-rig.shoulder[2]),E_Y=rig.elbow[1]-rig.shoulder[1];
  const W_F=-(rig.wrist[2]-rig.elbow[2]),W_Y=rig.wrist[1]-rig.elbow[1];
  return {
    S_F,S_Y,L1:rig.linkLengths.a2ToA3,L2:rig.linkLengths.a3ToA5,
    A0:Math.atan2(E_Y,E_F),F0:Math.atan2(W_Y,W_F),LAT:rig.shoulder[0]-rig.yaw[0],
  };
}

const rx=(v:Vec,a:number):Vec=>[v[0],v[1]*Math.cos(a)-v[2]*Math.sin(a),v[1]*Math.sin(a)+v[2]*Math.cos(a)];
const ry=(v:Vec,a:number):Vec=>[v[0]*Math.cos(a)+v[2]*Math.sin(a),v[1],-v[0]*Math.sin(a)+v[2]*Math.cos(a)];
const rz=(v:Vec,a:number):Vec=>[v[0]*Math.cos(a)-v[1]*Math.sin(a),v[0]*Math.sin(a)+v[1]*Math.cos(a),v[2]];
/**
 * Tool axis `t` and jaw axis `j` for a tool yaw and tilt. The jaws always open horizontally
 * along barAxis(toolYaw); tilt swings the tool from straight down (0) to horizontal (pi/2).
 */
export function toolAxes(toolYaw:number,tilt=0):{t:Vec;j:Vec} {
  const j=barAxis(toolYaw),h=barAxis(toolYaw+Math.PI/2);
  return {t:[h[0]*Math.sin(tilt),-Math.cos(tilt),h[2]*Math.sin(tilt)],j};
}
/** A5, the KR 16's spherical wrist centre, behind the tool point along the tool axis. */
export function wristCentre(P:Vec,toolYaw:number,tilt:number,rig:RigProfile=RIG):Vec {
  const {t}=toolAxes(toolYaw,tilt);
  return [(P[0]-ROBOT_BASE.x)/ROBOT_SCALE-t[0]*rig.toolDrop,(P[1]-ROBOT_BASE.y)/ROBOT_SCALE-t[1]*rig.toolDrop,(P[2]-ROBOT_BASE.z)/ROBOT_SCALE-t[2]*rig.toolDrop];
}

/**
 * Six-axis IK, rotation deltas from the rest pose. A1-A3 place the wrist centre; the
 * spherical wrist (A4 forearm roll, A5 bend, A6 flange roll) orients the tool. At rest the
 * forearm and flange point forward (-Z in the solver frame) and the jaws open along +Y.
 */
export function solveIK(P: [number, number, number], toolYaw: number,rig:RigProfile=RIG,tilt=0) {
  const {S_F,S_Y,L1,L2,A0,F0,LAT}=rigDimensions(rig);
  const W=wristCentre(P,toolYaw,tilt,rig),{t,j}=toolAxes(toolYaw,tilt);
  const dx = W[0]-rig.yaw[0], dz = W[2]-rig.yaw[2];
  const R = Math.max(Math.hypot(dx, dz), Math.abs(LAT) + 1e-3);
  const fwd = Math.sqrt(R * R - LAT * LAT);
  const yaw = Math.atan2(dx, dz) - Math.atan2(LAT, -fwd);
  const r = fwd - S_F;
  const h = W[1] - S_Y;
  const D = Math.min(Math.max(Math.hypot(r, h), Math.abs(L2 - L1) + 1e-3), L1 + L2 - 1e-3);
  const a = Math.atan2(h, r) + Math.acos((L1 ** 2 + D ** 2 - L2 ** 2) / (2 * L1 * D));
  const b = -(Math.PI - Math.acos((L1 ** 2 + L2 ** 2 - D ** 2) / (2 * L1 * L2)));
  const shoulder = a - A0;
  const elbow = b + A0 - F0;
  // Tool and jaw axes seen from the forearm frame (undo A1, then the A2+A3 pitch).
  const pitch=shoulder+elbow,local=(v:Vec)=>rx(ry(v,-yaw),-pitch);
  const tl=local(t),jl=local(j);
  // tl = Rf(A4) Rx(A5) f with f = (0,0,-1) and Rf a roll about f: (s5 sin A4, s5 cos A4, -c5).
  const c5=Math.max(-1,Math.min(1,-tl[2])),s5=Math.sqrt(1-c5*c5);
  // Of the two mirror solutions keep A5 bent the same way (A5 <= 0, A4 = 0 for a downward
  // tool) so the wrist never flips between poses.
  const forearm=s5>1e-6?Math.atan2(-tl[0],-tl[1]):0,wrist=-Math.acos(c5);
  // A6 turns the rest jaw axis (0,1,0) onto the wanted one: Rf(A6)(0,1,0) = (sin A6, cos A6, 0).
  const jw=rx(rz(jl,forearm),-wrist);
  const roll=Math.atan2(jw[0],jw[1]);
  return { yaw, shoulder, elbow, forearm, wrist, roll };
}

/** Reach margins: stay short of full stretch, out of the base column, and inside the A5 range. */
const REACH_MARGIN=.04, MIN_REACH_RADIUS=.7, WRIST_LIMIT=2.18;
export function canReachPose(p:Vec,toolYaw:number,tilt:number,rig:RigProfile=RIG):boolean {
  if(p[1]<0)return false;
  const {S_F,S_Y,L1,L2}=rigDimensions(rig),W=wristCentre(p,toolYaw,tilt,rig);
  const radius=Math.hypot(W[0]-rig.yaw[0],W[2]-rig.yaw[2]);
  if(radius<MIN_REACH_RADIUS||W[1]<.15)return false;
  const d=Math.hypot(radius-S_F,W[1]-S_Y);
  if(d>L1+L2-REACH_MARGIN||d<Math.abs(L1-L2)+.3)return false;
  return Math.abs(solveIK(p,toolYaw,rig,tilt).wrist)<=WRIST_LIMIT;
}
export function canReach(p:Vec,rig:RigProfile=RIG):boolean { return canReachPose(p,0,0,rig); }

/** Highest downward tool point the articulated model reaches above (x, z), or -1. */
export function maxToolHeight(x:number,z:number,rig:RigProfile=RIG):number {
  let lo=.02,hi=3;
  if(!canReach([x,lo,z],rig))return -1;
  for(let i=0;i<24;i++){const mid=(lo+hi)/2;if(canReach([x,mid,z],rig))lo=mid;else hi=mid;}
  return lo;
}
/** Headroom above a placement the wrist needs to descend vertically without straining the reach limit. */
export const REACH_CLEARANCE=.07;
export function reachableLayers(tower=TOWER,rig:RigProfile=RIG):number {
  let count=0;
  for(let layer=0;layer<100;layer++){
    const y=(layer+1)*BAR.H+REACH_CLEARANCE;
    // The set-down and push-home hand poses sit up to ~0.18 m along the bar from the slot.
    const reach=(p:Placement)=>[0,.18,-.18].every(o=>{const d=barAxis(p.yaw);return canReach([p.x+d[0]*o,y,p.z+d[2]*o],rig);});
    if([0,1,2].some(slot=>!reach(placement(layer*3+slot,tower))))break;
    count++;
  }
  return count;
}
export const LAYERS=reachableLayers();
export const MAX_BARS=LAYERS*3;
