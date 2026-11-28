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
/**
 * Hidden push tab ahead of the closed fingers. A vertical hand cannot reach a bar
 * end recessed under the cross layers, so the tab bridges from fingers kept outside the tower.
 */
export const TAB = { L:.08, H:BAR.H*.6, W:BAR.W*.7 };
export type Placement = { x:number; z:number; yaw:number; layer:number; slot:number };
export type Bar = { p:Vec; yaw:number; pitch:number; roll:number; visible:boolean; layer:number; slot:number; inTower:boolean; q?:Quat; loose?:boolean };
export type Bounds={min:Vec;max:Vec};
export type Dynamics = { reset:()=>void; step:(s:Sim,dt:number)=>boolean; settled:()=>boolean; chooseRemoval?:(s:Sim,candidates:number[],routes?:{index:number;side:number}[])=>number|undefined; armClear?:(tool:Vec,yaw:number,box:Bounds,rig?:RigProfile)=>boolean; armClearMany?:(tool:Vec,yaw:number,boxes:Bounds[],rig?:RigProfile)=>boolean };
export type Sim = {
  t:number; phase:'settle'|'build'|'remove'|'collapse'|'rest'; phaseT:number;
  placed:number; bars:Bar[]; plan:Placement[]; carry:boolean; tool:Vec; toolYaw:number;
  active:number; moves:number; removalHistory:number[]; rig:RigProfile;
  physics?:Dynamics; random:()=>number; tower:{x:number;z:number}; layers:number;
  /** along: signed pad offset from the bar centre along its +length axis, fixed when the jaws close. */
  generation:number; source?:{p:Vec;yaw:number;q?:Quat;entry:Vec;entryYaw:number;along?:number;centre?:{x:number;z:number}}; released:boolean;
  mode:'push'; side:number; push?:{p:Vec;yaw:number}; closed:boolean; recoveryHistory:number[];
  removalScores?:{index:number;stable:boolean;motion:number}[];
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
  return {t:0,phase:'settle',phaseT:0,placed:0,bars,plan:Array.from({length:barCount},(_,i)=>placement(i)),carry:false,tool:[...REST_TOOL],toolYaw:0,active:-1,moves:0,removalHistory:[],random,tower:{...TOWER},layers,generation:0,released:false,mode:'push',side:1,closed:false,recoveryHistory:[],jaw:GRIPPER_OPEN_HALF_GAP,nextRemovalCheck:0,rig};
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
  const all=eligibleBars(s).filter(i=>{const b=s.bars[i];return !!b&&lateral(b.p,b.yaw);});
  // Start just below the protected top story.  A short push there has far less
  // load above it than a deep support, while the completed top remains intact.
  const margin=(i:number)=>supportMargin(s,i)>.01?1:0;
  const loose=(i:number)=>s.bars[i]?.loose?1:0;
  return all.sort((a,b)=>margin(b)-margin(a)||loose(b)-loose(a)||(s.bars[b]?.layer??0)-(s.bars[a]?.layer??0)||supportMargin(s,b)-supportMargin(s,a));
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
type Key=[number,Vec,number,boolean];
function animate(s:Sim,keys:Key[],k:number) {
  let j=0;while(j<keys.length-2&&k>(keys[j+1]?.[0]??1))j++;
  const a=keys[j],b=keys[j+1];if(!a||!b)return;
  const u=smooth((k-a[0])/(b[0]-a[0]));
  s.tool=[mix(a[1][0],b[1][0],u),mix(a[1][1],b[1][1],u),mix(a[1][2],b[1][2],u)];s.toolYaw=mix(a[2],b[2],u);s.carry=a[3];
}
/** Jaws straddle the bar width, so the bar's length axis is 90 degrees from the tool yaw. */
const BAR_FROM_TOOL=-Math.PI/2;
function follow(s:Sim,b:Bar,u:number,offset:number) {
  b.yaw=s.toolYaw+BAR_FROM_TOOL;
  const d=barAxis(b.yaw);
  b.p=[s.tool[0]-d[0]*offset,s.tool[1]-BAR.H/2,s.tool[2]-d[2]*offset];
  // Reorient a fallen lingot smoothly during the lift, never flip it on the floor.
  const target=yawQuat(b.yaw),q=s.source?.q;
  if(q&&u<1){const sign=q.y*target.y+q.w*target.w<0?-1:1;
    const v={x:q.x*(1-u),y:q.y*(1-u)+target.y*u*sign,z:q.z*(1-u),w:q.w*(1-u)+target.w*u*sign};
    const n=Math.hypot(v.x,v.y,v.z,v.w);b.q={x:v.x/n,y:v.y/n,z:v.z/n,w:v.w/n};
  }else b.q=target;
}
function rememberSource(s:Sim,b:Bar) {s.source={p:[...b.p],entry:[...s.tool],entryYaw:s.toolYaw,yaw:b.q?yawOfQuat(b.q):b.yaw,...(b.q?{q:{...b.q}}:{})};s.released=false;}
/** Lock the hold offset from the bar's actual pose the moment the jaws close, so it never jumps. */
function grip(s:Sim,b:Bar):number {
  const source=s.source;if(!source)return 0;
  if(source.along===undefined){const d=barAxis(source.yaw);source.along=(s.tool[0]-b.p[0])*d[0]+(s.tool[2]-b.p[2])*d[2];}
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
  const outLow=along(c,d,end+NUDGE+TAB.L+.015),outHigh=[outLow[0],clear[1],outLow[2]] as Vec;
  const home=along(c,d,end+TAB.L);
  return {barYaw,centre:along(c,d,NUDGE,toolY-BAR.H/2),set,clear,outHigh,outLow,home,pushYaw:Math.atan2(-d[2],d[0]),pushDir:[-d[0],0,-d[2]] as Vec};
}
/**
 * Friction of the selected bar while it is slid: extracted, or pushed home after a set-down.
 * Full friction shears the whole compliant stack with it, so bars spring back short or ride along.
 */
export const SLIDE_FRICTION=.04;
export const BAR_FRICTION=.62;
/** Push tab centre for the hand at `hand` pushing along `dir`, at the bar's mid-height. */
export function tabCentre(hand:Vec,dir:Vec):Vec { return [hand[0]+dir[0]*TAB.L/2,hand[1]-BAR.H/2,hand[2]+dir[2]*TAB.L/2]; }
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
  // The closed fingers push across the bar's width too, so the wrist keeps the place yaw.
  const placeYaw=nearestAngle(pose.barYaw-BAR_FROM_TOOL,pickYaw),pushYaw=placeYaw;
  const lift=clampHeight(pick,s.rig,Math.max(pick[1]+.32,toolY+.25));
  const above=clampHeight(pose.set,s.rig,toolY+.15);
  // Travel at lift height, then descend vertically: a diagonal path would plough through the scatter.
  const over=clampHeight(pose.set,s.rig,Math.max(lift[1],above[1]));
  // Start at the exact pose left by the preceding move, so the arm never teleports.
  animate(s,[[0,source.entry,source.entryYaw,false],[.1,lift,pickYaw,false],[.18,pick,pickYaw,false],[.22,pick,pickYaw,true],[.32,lift,pickYaw,true],
    [.48,over,placeYaw,true],[.55,above,placeYaw,true],[.63,pose.set,placeYaw,true],[.66,pose.set,placeYaw,false],[.7,pose.clear,placeYaw,false],
    [.74,pose.outHigh,pushYaw,false],[.78,pose.outLow,pushYaw,false],[.88,pose.home,pushYaw,false],[.92,pose.outLow,pushYaw,false],[1,clampHeight(pose.outLow,s.rig,toolY+.15),pushYaw,false]],k);
  s.closed=k>=.2&&k<.66||k>=.76&&k<.92;
  if(s.carry){source.along=offset;follow(s,b,smooth((k-.22)/.1),offset);}
  if(k>=.66)release(s,b,p,pose.centre,pose.barYaw);
  if(k>=.78&&k<.88)s.push={p:tabCentre(s.tool,pose.pushDir),yaw:pose.pushYaw};else delete s.push;
  if(k>=1-1e-8){s.phaseT=0;s.active=-1;s.closed=false;delete s.push;delete s.source;done();}
}
/** Closed-hand stroke: long enough that the far end clears the cross layer above for the pads. */
export const PUSH_STROKE=.045;
/** The pulled bar's inner end clears the tower face by this much before it is lifted past drifted neighbours. */
const PULL_CLEAR=.07;
/** Hand poses along the push axis (unit `a`, from the bar centre top `c`). */
function removalPoses(c:Vec,a:Vec) {
  const contact=-(BAR_CONTACT.L/2+.004+TAB.L),pinch=TOWER_HALF+FINGER_CLEAR;
  return {contact:along(c,a,contact),pushed:along(c,a,contact+PUSH_STROKE),out:along(c,a,contact-.05),
    pinch:along(c,a,pinch),pulled:along(c,a,pinch+BAR.L-PUSH_STROKE+PULL_CLEAR),orbitRadius:Math.max(pinch,-contact)+.05};
}
function extractionPoses(b:Bar,side:number):Vec[] {
  const a:Vec=barAxis(b.yaw).map(v=>v*side) as Vec,poses=removalPoses([b.p[0],b.p[1]+BAR.H/2,b.p[2]],a);
  return [poses.out,poses.contact,poses.pushed,poses.pinch,poses.pulled];
}
function clearExtractionPath(path:Vec[],yaw:number,obstacles:Bounds[],armClear:Dynamics['armClear'],armClearMany:Dynamics['armClearMany'],rig:RigProfile):boolean {
  for(let i=0;i<path.length-1;i++){
    const a=path[i],b=path[i+1];if(!a||!b)continue;
    const steps=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1],b[2]-a[2])/.02));
    for(let n=0;n<=steps;n++){
      const u=n/steps,p:Vec=[a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u,a[2]+(b[2]-a[2])*u];
      if(!canReach(p,rig)||armClearMany&&!armClearMany(p,yaw,obstacles,rig)||!armClearMany&&armClear&&obstacles.some(box=>!armClear(p,yaw,box,rig)))return false;
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
/**
 * Centre of the bars a layer rests on, measured from their actual poses: a tall stack
 * drifts by millimetres, and a nominal slot can leave an outer bar's weight over the edge.
 */
export function supportCentre(s:Sim,layer:number,exclude?:Bar):{x:number;z:number} {
  const below=s.bars.filter(b=>b!==exclude&&b.inTower&&b.layer===layer-1);
  if(layer===0||!below.length)return {...s.tower};
  // Slots spread across a bar, so subtract each bar's slot offset on that axis.
  const across=(layer-1)%2?0:2,centre={x:0,z:0};
  for(const b of below){const o=(b.slot-1)*SLOT;centre.x+=b.p[0]-(across===0?o:0);centre.z+=b.p[2]-(across===2?o:0);}
  return {x:centre.x/below.length,z:centre.z/below.length};
}
function topPlacement(s:Sim,exclude?:Bar):Placement {
  const tower=s.bars.filter(b=>b!==exclude&&b.inTower);
  const topLayer=Math.max(...tower.map(b=>b.layer),-1);
  const filled=tower.filter(b=>b.layer===topLayer).length;
  // Continue a partial top row; otherwise start the next crossed story.
  const layer=filled>0&&filled<3?topLayer:topLayer+1,slot=layer===topLayer?filled:0;
  return placement(layer*3+slot,supportCentre(s,layer,exclude));
}
function beginRemoval(s:Sim) {
  // Deconstruction is a controlled closed-hand push, then a pinch of the protruding
  // end, a straight pull out of the tower and a placement on the top story.
  const top=topPlacement(s),topToolY=placementToolY(s,top);
  if(!canReach([top.x,topToolY,top.z],s.rig)||maxToolHeight(top.x,top.z,s.rig)<topToolY+.05)return;
  const tower=s.bars.filter(b=>b.inTower);
  const clearSides=new Map<number,number[]>(),reachableSides=new Map<number,number[]>(),ranked=strategicCandidates(s);
  for(const i of ranked.slice(0,12)){
    const b=s.bars[i];if(!b)continue;
    // Check the real gaps between bars, excluding the held bar that physics removes from robot collisions.
    const obstacles=tower.filter(item=>item!==b).map(barBounds);
    const sourceYaw=b.q?yawOfQuat(b.q):b.yaw;
    const reachable=[1,-1].filter(side=>extractionPoses(b,side).every(p=>canReach(p,s.rig)));
    if(reachable.length)reachableSides.set(i,reachable);
    const armClear=s.physics?.armClear,armClearMany=s.physics?.armClearMany;
    const clear=reachable.filter(side=>clearExtractionPath(extractionPoses(b,side),sourceYaw-BAR_FROM_TOOL,obstacles,armClear,armClearMany,s.rig));
    if(clear.length)clearSides.set(i,clear);
    if(clearSides.size>=6)break;
  }
  // The capsule clearance is deliberately conservative around narrow tower gaps.
  // When it rejects every route, the Rapier push probe below is the final safety gate.
  const sides=clearSides.size?clearSides:reachableSides;
  if(!sides.size)return;
  const eligible=[...sides.keys()];
  const routes=eligible.map(index=>{const options=sides.get(index)??[1,-1];return {index,side:options.length>1?1:(options[0]??1)};});
  const i=s.physics?.chooseRemoval?s.physics.chooseRemoval(s,eligible,routes):eligible[0];
  if(i===undefined)return;
  const b=s.bars[i];if(!b)return;
  s.active=i;s.phaseT=0;s.removalHistory.push(i);rememberSource(s,b);
  s.mode='push';s.side=routes.find(route=>route.index===i)?.side??1;
}
function stepRemove(s:Sim) {
  if(s.active<0){
    if(s.phaseT<s.nextRemovalCheck)return;
    beginRemoval(s);
    // Retry soon after normal contact settling instead of leaving the arm parked
    // at the construction handoff when a candidate is temporarily unavailable.
    if(s.active<0)s.nextRemovalCheck=s.phaseT+2;
    else s.nextRemovalCheck=0;
  }
  const b=s.bars[s.active],source=s.source;if(!b||!source)return;
  const k=s.phaseT/REMOVE_CYCLE,axis:Vec=barAxis(source.yaw).map(v=>v*s.side) as Vec;
  const c:Vec=[source.p[0],source.p[1]+BAR.H/2,source.p[2]];
  const roofY=Math.max(...s.bars.filter(x=>x.inTower&&x!==b).map(x=>x.p[1]+BAR.H/2))+BAR.H+.06;
  const hi=(v:Vec):Vec=>clampHeight(v,s.rig,roofY);
  const poses=removalPoses(c,axis),orbitY=c[1]+BAR.H+.03;
  const perpendicular:Vec=[-axis[2],0,axis[0]];
  const orbit=(angle:number):Vec=>[
    c[0]+axis[0]*Math.cos(angle)*poses.orbitRadius+perpendicular[0]*Math.sin(angle)*poses.orbitRadius,
    orbitY,
    c[2]+axis[2]*Math.cos(angle)*poses.orbitRadius+perpendicular[2]*Math.sin(angle)*poses.orbitRadius,
  ];
  const toolYaw=nearestAngle(source.yaw-BAR_FROM_TOOL,source.entryYaw);
  // The hold offset is fixed when the jaws close; before that, plan with the commanded push.
  const offset=source.along??s.side*(TOWER_HALF+FINGER_CLEAR-PUSH_STROKE);
  // Fix the destination once per removal: the layer under it keeps settling by fractions of a millimetre.
  source.centre??=supportCentre(s,topPlacement(s,b).layer,b);
  const top=topPlacement(s,b),dest=placement(top.layer*3+top.slot,source.centre),toolY=placementToolY(s,dest,b),pose=settlePoses(dest,offset,toolY);
  const placeYaw=nearestAngle(pose.barYaw-BAR_FROM_TOOL,toolYaw),pushYaw=placeYaw;
  const above=clampHeight(pose.set,s.rig,toolY+.15);
  // Push the recessed end, circle above the story to the protruding end, pinch it,
  // pull it straight out of the tower, then set it on top and push it home.
  animate(s,[[0,source.entry,source.entryYaw,false],[.05,clampHeight(source.entry,s.rig,Math.max(roofY,source.entry[1])),toolYaw,false],
    [.11,hi(poses.out),toolYaw,false],[.15,poses.out,toolYaw,false],[.19,poses.contact,toolYaw,false],[.3,poses.pushed,toolYaw,false],[.34,poses.contact,toolYaw,false],[.37,poses.out,toolYaw,false],
    [.41,orbit(Math.PI),toolYaw,false],[.45,orbit(Math.PI/2),toolYaw,false],[.48,orbit(0),toolYaw,false],[.5,poses.pinch,toolYaw,false],[.52,poses.pinch,toolYaw,true],
    [.58,poses.pulled,toolYaw,true],[.62,hi(poses.pulled),toolYaw,true],[.7,hi(pose.set),placeYaw,true],[.74,above,placeYaw,true],[.79,pose.set,placeYaw,true],
    [.81,pose.set,placeYaw,false],[.83,pose.clear,placeYaw,false],[.86,pose.outHigh,pushYaw,false],[.88,pose.outLow,pushYaw,false],[.95,pose.home,pushYaw,false],[.97,pose.outLow,pushYaw,false],[1,hi(pose.outLow),pushYaw,false]],k);
  s.closed=k>=.13&&k<.4||k>=.52&&k<.81||k>=.87&&k<.97;
  if(s.carry&&source.along===undefined&&Math.abs(grip(s,b))>BAR.L/2-GRIPPER_FINGER_HALF_WIDTH){
    // The pushed bar sprang back: the pads would close beside its end, not on it.
    // Leave it in the tower and choose again from the current hand pose.
    s.phaseT=0;s.carry=false;s.closed=false;delete s.push;delete s.source;s.active=-1;s.nextRemovalCheck=0;return;
  }
  if(s.carry){follow(s,b,1,grip(s,b));}
  if(k>=.19&&k<.34)s.push={p:tabCentre(s.tool,axis),yaw:source.yaw};
  else if(k>=.88&&k<.95)s.push={p:tabCentre(s.tool,pose.pushDir),yaw:pose.pushYaw};
  else delete s.push;
  if(k>=.81&&!s.released)release(s,b,dest,pose.centre,pose.barYaw);
  if(k>=1){
    s.phaseT=0;s.carry=false;s.closed=false;delete s.push;delete s.source;s.active=-1;s.nextRemovalCheck=0;
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
  s.phase='build';s.phaseT=0;s.placed=0;s.active=-1;s.moves=0;s.carry=false;s.closed=false;delete s.source;delete s.push;s.nextRemovalCheck=0;
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
  if(fallen&&(s.phase==='remove'||s.phase==='build')&&s.placed>=6){s.phase='collapse';s.phaseT=0;s.carry=false;s.closed=false;s.active=-1;delete s.source;delete s.push;s.nextRemovalCheck=0;}
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

/** Analytic IK for a downward-pointing gripper at P; returns rotation deltas from the rest pose. */
export function solveIK(P: [number, number, number], toolYaw: number,rig:RigProfile=RIG) {
  const {S_F,S_Y,L1,L2,A0,F0,LAT}=rigDimensions(rig);
  const dx = (P[0] - ROBOT_BASE.x) / ROBOT_SCALE-rig.yaw[0], dz = (P[2] - ROBOT_BASE.z) / ROBOT_SCALE-rig.yaw[2];
  const R = Math.max(Math.hypot(dx, dz), Math.abs(LAT) + 1e-3);
  const fwd = Math.sqrt(R * R - LAT * LAT);
  const yaw = Math.atan2(dx, dz) - Math.atan2(LAT, -fwd);
  const r = fwd - S_F;
  const h = (P[1] - ROBOT_BASE.y) / ROBOT_SCALE + rig.toolDrop - S_Y;
  const D = Math.min(Math.max(Math.hypot(r, h), Math.abs(L2 - L1) + 1e-3), L1 + L2 - 1e-3);
  const a = Math.atan2(h, r) + Math.acos((L1 ** 2 + D ** 2 - L2 ** 2) / (2 * L1 * D));
  const b = -(Math.PI - Math.acos((L1 ** 2 + L2 ** 2 - D ** 2) / (2 * L1 * L2)));
  const shoulder = a - A0;
  const elbow = b + A0 - F0;
  // Pitch the flange straight down whatever its rest direction.
  const wrist = -Math.PI / 2 - rig.toolRest - (a + b - F0);
  // The KR 16 jaws open along the flange's rest up axis; A6 turns them onto the tool yaw.
  const roll = yaw + Math.PI / 2 - toolYaw;
  return { yaw, shoulder, elbow, wrist, roll };
}

/** Reach margins: stay short of full stretch, out of the base column, and inside the A5 range. */
const REACH_MARGIN=.04, MIN_REACH_RADIUS=.7, WRIST_LIMIT=2.18;
export function canReach(p:Vec,rig:RigProfile=RIG):boolean {
  if(p[1]<0)return false;
  const {S_F,S_Y,L1,L2}=rigDimensions(rig);
  const radius=Math.hypot((p[0]-ROBOT_BASE.x)/ROBOT_SCALE-rig.yaw[0],(p[2]-ROBOT_BASE.z)/ROBOT_SCALE-rig.yaw[2]);
  if(radius<MIN_REACH_RADIUS)return false;
  const d=Math.hypot(radius-S_F,(p[1]-ROBOT_BASE.y)/ROBOT_SCALE+rig.toolDrop-S_Y);
  if(d>L1+L2-REACH_MARGIN||d<Math.abs(L1-L2)+.3)return false;
  return Math.abs(solveIK(p,0,rig).wrist)<=WRIST_LIMIT;
}
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
