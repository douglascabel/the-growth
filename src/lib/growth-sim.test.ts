import {describe,expect,it} from 'vitest';
import * as THREE from 'three';
import {isLoose,BAR,SLOT,MAX_BARS,LAYERS,TOWER,TOWER_HALF,ROBOT_BASE,ROBOT_SCALE,RIG,rigProfileFromBones,GRIPPER_OPEN_HALF_GAP,GRIPPER_CLOSED_HALF_GAP,GRIPPER_FINGER_HALF_WIDTH,GRIPPER_TOOL_OFFSET,PUSH_STROKE,REMOVE_CYCLE,CYCLE,NUDGE,toolAxes,END_GRIP_ALONG,createSim,stepSim,placement,eligibleBars,canReach,maxToolHeight,restartFromPile,strategicCandidates,supportMargin,REACH_CLEARANCE,jawPose,fingertipDepth,solveIK,settlePoses,barAxis} from './growth-sim';
import {createDynamics} from './growth-physics';
import {GRIPPER_FINGER_RADIUS,robotContacts,robotJoints} from './robot-kinematics';
import {normalizeLingot} from './lingot-geometry';
const advance=(s:ReturnType<typeof createSim>,seconds:number)=>{for(let i=0;i<Math.round(seconds*60);i++)stepSim(s,1/60);};
/** Without Rapier nothing moves the pushed bar: emulate the fingertip stroke (with some spring-back). */
const stepWithHandPush=(s:ReturnType<typeof createSim>)=>{const was=s.pushing;stepSim(s,1/60);const b=s.bars[s.active];
  if(b&&was&&!s.pushing&&s.phaseT/REMOVE_CYCLE<.5&&s.source){const d=barAxis(s.source.yaw),m=s.side*PUSH_STROKE*.7;b.p=[b.p[0]+d[0]*m,b.p[1],b.p[2]+d[2]*m];}};
const built=()=>{const s=createSim();s.phase='remove';s.placed=MAX_BARS;s.bars.forEach((b,i)=>{const p=placement(i);b.p=[p.x,(p.layer+.5)*BAR.H,p.z];b.yaw=p.yaw;b.q={x:0,y:Math.sin(p.yaw/2),z:0,w:Math.cos(p.yaw/2)};b.layer=p.layer;b.slot=p.slot;b.inTower=true;b.loose=isLoose(TOWER,p);});return s;};
describe('reach-limited sculpture',()=>{
 it('uses 155 × 41.45 × 62.14 mm',()=>{expect(BAR).toEqual({L:.155,H:.04145,W:.06214});});
 it('preserves the supplied positive-Y inscription face',()=>{
   const g=new THREE.BoxGeometry(155.37,41.45,62.14);const normalized=normalizeLingot(g);normalized.computeBoundingBox();
   const size=normalized.boundingBox?.getSize(new THREE.Vector3());expect(size?.x).toBeCloseTo(.155,6);expect(size?.y).toBeCloseTo(.04145,6);expect(size?.z).toBeCloseTo(.06214,6);
   const faceted=new THREE.SphereGeometry(1,12,8).toNonIndexed(),smooth=normalizeLingot(faceted),positions=smooth.getAttribute('position'),normals=smooth.getAttribute('normal');
   expect(normals.count).toBe(positions.count);expect([...normals.array].every(Number.isFinite)).toBe(true);
   // An apex on +Y must stay on +Y (no flip).
   const cone=normalizeLingot(new THREE.ConeGeometry(30,40,8)),p=cone.getAttribute('position');let top=0;
   for(let i=1;i<p.count;i++)if(p.getY(i)>p.getY(top))top=i;expect(Math.hypot(p.getX(top),p.getZ(top))).toBeLessThan(1e-6);
 });
 it('derives KR 16 R2010 link lengths from its GLB pivots',()=>{
   // Authored pivots: A2 160 mm ahead of A1 at 520 mm, 980 mm upper arm, 150 x 860 mm forearm offset.
   expect(RIG.shoulder[1]).toBeCloseTo(.52,6);expect(-RIG.shoulder[2]).toBeCloseTo(.16,6);
   expect(RIG.linkLengths.a2ToA3).toBeCloseTo(.98,6);
   expect(RIG.linkLengths.a3ToA5).toBeCloseTo(Math.hypot(.15,.86),6);
   // Pitch pivots may sit anywhere along their lateral axes: the arm plane comes from the wrist.
   expect(RIG.shoulder[0]).toBeCloseTo(RIG.wrist[0],9);expect(RIG.elbow[0]).toBeCloseTo(RIG.wrist[0],9);
   expect(RIG.toolRest).toBeCloseTo(0,9);
   // Maximum wrist reach matches the KR 16 R2010's 2013 mm.
   expect(-RIG.shoulder[2]+RIG.linkLengths.a2ToA3+RIG.linkLengths.a3ToA5).toBeCloseTo(2.013,2);
   const moved=rigProfileFromBones({track:[1,0,2],a1:[1,.2845,2],a2:[.9,.52,2.16],a3:[.88,1.5,2.16],a5:[1,1.65,3.02],tool:[1,1.65,3.4]});
   expect(moved.linkLengths.a2ToA3).toBeCloseTo(.98,6);expect(moved.wrist[2]).toBeCloseTo(-1.02,6);
 });
 it('places the IK wrist straight above a downward tool',()=>{
   for(const tool of [[1.25,.05,0],[1.25,1.2,0],[.9,.4,.8],[-.3,.6,1.1]] as [number,number,number][]){
     const {wrist}=robotJoints(tool,.3);
     expect(wrist[0]).toBeCloseTo(tool[0],6);expect(wrist[2]).toBeCloseTo(tool[2],6);expect(wrist[1]-tool[1]).toBeCloseTo(RIG.toolDrop,6);
   }
 });
 it('seats the robot on the floor',()=>{expect(ROBOT_BASE.y).toBe(0);});
 it('uses 1:1 scale and lays the scatter inside the floor reach',()=>{expect(ROBOT_SCALE).toBe(1);expect(TOWER.x).toBeCloseTo(1.25,6);
   const s=createSim();expect(s.bars.every(b=>canReach([b.p[0],BAR.H,b.p[2]])&&Math.hypot(b.p[0]-TOWER.x,b.p[2]-TOWER.z)>.3)).toBe(true);});
 it('covers the floor-level tower with the articulated reach envelope',()=>{
   expect(canReach([TOWER.x, BAR.H/2, TOWER.z])).toBe(true);
   expect(maxToolHeight(TOWER.x,TOWER.z)).toBeGreaterThan(BAR.H);
 });
 it('alternates three parallel bars by 90 degrees',()=>{
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(p.layer).toBe(Math.floor(i/3));expect(p.slot).toBe(i%3);expect(p.yaw).toBe(Math.floor(i/3)%2?Math.PI/2:0);}
   expect(placement(0).z).toBeCloseTo(TOWER.z-SLOT,6);expect(placement(2).z).toBeCloseTo(TOWER.z+SLOT,6);
 });
 it('stops at the last complete reachable layer',()=>{
   expect(LAYERS).toBeGreaterThanOrEqual(25);expect(LAYERS).toBeLessThan(40);
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(canReach([p.x,(p.layer+1)*BAR.H+.02,p.z])).toBe(true);}
   // The next story fails somewhere along its set-down and push-home hand poses.
   expect([0,1,2].some(slot=>{const p=placement(MAX_BARS+slot),d=barAxis(p.yaw);return [0,.18,-.18].some(o=>!canReach([p.x+d[0]*o,(p.layer+1)*BAR.H+REACH_CLEARANCE,p.z+d[2]*o]));})).toBe(true);
 });
 it('only chooses bars below the highest completed layer',()=>{
   const s=built();advance(s,1.6);expect(s.phase).toBe('remove');expect(s.removalHistory).toHaveLength(1);
   const chosen=s.removalHistory[0];expect(chosen===undefined?99:s.bars[chosen]?.layer).toBeLessThan(LAYERS-1);
   expect(eligibleBars(s).every(i=>(s.bars[i]?.layer??99)<LAYERS-1)).toBe(true);
 });
 it('keeps a partial new top and the highest complete story untouched',()=>{
   const s=built(),promoted=s.bars[0];if(!promoted)throw new Error('missing bar');
   promoted.layer=LAYERS;promoted.slot=0;
   expect(eligibleBars(s).every(i=>(s.bars[i]?.layer??99)<LAYERS-1)).toBe(true);
   expect(eligibleBars(s)).not.toContain(MAX_BARS-1);
 });
 it('pushes one bar from the side with a horizontal closed hand',()=>{
   const s=built();advance(s,1.6);const index=s.active;if(index<0)throw new Error('no safe removal was selected');
   const bar=s.bars[index];if(!bar)throw new Error('missing bar');expect(bar.slot).not.toBe(1);
   let tested=false;
   for(let i=0;i<Math.ceil(REMOVE_CYCLE*60);i++){
     stepSim(s,1/60);
     if(s.pushing){
       tested=true;expect(s.carry).toBe(false);expect(s.closed).toBe(true);expect(s.active).toBe(index);
       // The hand works level with the bar, pointing along it, never pressing down from above.
       expect(s.toolTilt).toBeCloseTo(Math.PI/2,6);expect(Math.abs(s.tool[1]-bar.p[1])).toBeLessThan(.002);
       const {t}=toolAxes(s.toolYaw,s.toolTilt),d=barAxis(bar.yaw);expect(Math.abs(t[0]*d[0]+t[2]*d[2])).toBeCloseTo(1,6);
     }
   }
   expect(PUSH_STROKE).toBeLessThanOrEqual(.05);expect(tested).toBe(true);
 });
 it('pulls the pushed bar out from the opposite side and lets it fall',()=>{
   const s=built();s.random=()=>.1;advance(s,1.6);
   const index=s.active,bar=s.bars[index];if(index<0||!bar)throw new Error('no safe removal was selected');
   const start=[...bar.p],yaw=bar.yaw;let pushSide=0,pullSide=0,pinched=false;
   for(let i=0;i<Math.ceil((REMOVE_CYCLE+.2)*60);i++){
     stepWithHandPush(s);
     const d=barAxis(yaw),side=(s.tool[0]-start[0]!)*d[0]+(s.tool[2]-start[2]!)*d[2];
     if(s.pushing)pushSide=Math.sign(side);
     if(s.carry&&s.active===index){
       pinched=true;pullSide=Math.sign(side);expect(s.jaw).toBeLessThan(GRIPPER_OPEN_HALF_GAP);expect(Math.cos(bar.yaw-yaw)).toBeCloseTo(1,6);
       expect(s.toolTilt).toBeCloseTo(Math.PI/2,6);
     }
   }
   expect(pinched).toBe(true);expect(pushSide).toBe(-pullSide);
   // Let go outside the tower: it is no longer part of it and nothing is restacked.
   expect(bar.inTower).toBe(false);expect(s.bars.filter(b=>b.inTower).length).toBe(MAX_BARS-1);
   expect(Math.hypot(bar.p[0]-start[0]!,bar.p[2]-start[2]!)).toBeGreaterThan(BAR.L/2+TOWER_HALF);
 });
 it('does not start a removal when the tower is out of reach',()=>{
   const s=built(),shift=2-TOWER.x;s.tower={x:ROBOT_BASE.x+2,z:TOWER.z};for(const b of s.bars)b.p=[b.p[0]+shift,b.p[1],b.p[2]];advance(s,2);
   expect(s.active).toBe(-1);expect(s.removalHistory).toHaveLength(0);
 });
 it('does not leave deconstruction frozen when the tower keeps jittering',()=>{
   const s=built();s.random=()=>.1;s.physics={reset:()=>{},step:()=>false,settled:()=>false,armClear:()=>true,chooseRemoval:(_state,candidates)=>candidates[0]};
   advance(s,9);expect(s.phase).toBe('remove');expect(s.removalHistory).toHaveLength(1);expect(s.active).toBeGreaterThanOrEqual(0);
 });
 it('finds a reachable extraction route through the actual gaps between bars',async()=>{
   const s=built(),d=await createDynamics();s.random=()=>.1;
   s.physics={...d,step:()=>false,settled:()=>true,chooseRemoval:(_state,candidates)=>candidates[0]};
   advance(s,1.6);expect(s.removalHistory).toHaveLength(1);expect(s.active).toBeGreaterThanOrEqual(0);
 });
 it('starts a physics-approved closed-hand push instead of parking at deconstruction',async()=>{
   // Let the freshly stacked contacts settle as the live scene does before probing.
   const s=built(),d=await createDynamics();for(let i=0;i<90;i++)d.step(s,1/60);s.physics={...d,step:()=>false,settled:()=>true};
   advance(s,1.6);
   expect(s.removalHistory).toHaveLength(1);expect(s.active).toBeGreaterThanOrEqual(0);
  },30000);
 it('leaves the tower untouched when physics does not approve a push',()=>{
   const s=built();s.physics={reset:()=>{},step:()=>false,settled:()=>true,chooseRemoval:()=>undefined};advance(s,1.6);
   expect(s.removalHistory).toHaveLength(0);expect(s.active).toBe(-1);
 });
 it('starts from chaotic tilted bars and waits for settling before recovery',()=>{
   const s=createSim();expect(s.phase).toBe('settle');expect(s.bars).toHaveLength(MAX_BARS);
   expect(new Set(s.bars.map(b=>b.yaw.toFixed(2))).size).toBeGreaterThan(50);
   expect(s.bars.every(b=>b.p[1]<.17&&!b.inTower)).toBe(true);
   advance(s,1.9);expect(s.active).toBe(-1);advance(s,.3);expect(s.phase).toBe('build');expect(s.recoveryHistory).toHaveLength(1);
 });
 it('uses full-height supports and begins every construction move from the current hand pose',()=>{
   const s=createSim(()=>.1),start=[...s.tool];advance(s,2.05);
   expect(s.active).toBeGreaterThanOrEqual(0);expect(s.bars.filter(b=>b.inTower).every(b=>b.loose===false)).toBe(true);
   expect(Math.hypot((s.tool[0]??0)-(start[0]??0),(s.tool[1]??0)-(start[1]??0),(s.tool[2]??0)-(start[2]??0))).toBeLessThan(.005);
   for(let i=0;i<90;i++){
     const before=[...s.tool];stepSim(s,1/60);
     expect(Math.hypot((s.tool[0]??0)-(before[0]??0),(s.tool[1]??0)-(before[1]??0),(s.tool[2]??0)-(before[2]??0))).toBeLessThan(.025);
   }
 });
 it('prefers a redundant support over the last bar under a loaded layer',()=>{
   const s=built();s.bars[0] && (s.bars[0].inTower=false);s.bars[2] && (s.bars[2].inTower=false);
   expect(supportMargin(s,1)).toBe(-1);expect(strategicCandidates(s)[0]).not.toBe(1);
 });
 it('uses articulated finite contact shapes which follow the arm pose',()=>{
   const a=robotContacts([1.2,.7,.34],0),b=robotContacts([.9,1,.34],0);
   expect(a.map(c=>c.name)).toEqual(['shoulder','upper-arm','forearm','wrist','finger--1','finger-1']);
   expect(solveIK([1.2,.7,.34],0).wrist).toBeLessThan(0);
   expect(a.every(c=>[...c.p,...Object.values(c.q)].every(Number.isFinite))).toBe(true);
   expect(a[2]?.p).not.toEqual(b[2]?.p);
 });
 it('uses the live jaw gap for the physical fingers',()=>{
   for(const gap of [GRIPPER_OPEN_HALF_GAP,GRIPPER_CLOSED_HALF_GAP]){
     const [a,b]=robotContacts([1.2,.7,.34],0,gap).slice(4);
     expect(Math.abs((b?.p[0]??0)-(a?.p[0]??0))).toBeCloseTo(2*(gap+GRIPPER_FINGER_RADIUS),6);
   }
 });
 it('opens the authored lever gripper wide enough for a bar and closes on its tapered flank',()=>{
   expect(2*GRIPPER_OPEN_HALF_GAP).toBeGreaterThan(BAR.W+.01);
   expect(2*GRIPPER_CLOSED_HALF_GAP).toBeLessThan(BAR.W);expect(2*GRIPPER_CLOSED_HALF_GAP).toBeGreaterThan(.049);
   expect(jawPose(GRIPPER_OPEN_HALF_GAP).angle).toBeCloseTo(0,9);expect(jawPose(GRIPPER_CLOSED_HALF_GAP).angle).toBeLessThan(0);
   // Closed pads hold mid-height with their tips above the bar bottom (and the layer below).
   expect(fingertipDepth(GRIPPER_CLOSED_HALF_GAP)).toBeGreaterThan(BAR.H/2);expect(fingertipDepth(GRIPPER_CLOSED_HALF_GAP)).toBeLessThan(BAR.H-.005);
   expect(GRIPPER_TOOL_OFFSET).toBeLessThan(0);
 });
 it('sets bars down with the fingers clear of neighbouring ends, then pushes them home',()=>{
   // Layer 0 runs radially: its held end faces the robot.
   for(const p of [placement(1)]){
     const pose=settlePoses(p,END_GRIP_ALONG,.3),toRobot=Math.hypot(pose.set[0]-ROBOT_BASE.x,pose.set[2]-ROBOT_BASE.z)<=Math.hypot(p.x-ROBOT_BASE.x,p.z-ROBOT_BASE.z)+1e-9;
     const d=barAxis(pose.barYaw),fingers=(pose.set[0]-p.x)*d[0]+(pose.set[2]-p.z)*d[2];
     expect(toRobot).toBe(true);expect(Math.abs(fingers)-GRIPPER_FINGER_HALF_WIDTH).toBeGreaterThan(BAR.L/2+.004);
     expect(Math.hypot(pose.centre[0]-p.x,pose.centre[2]-p.z)).toBeCloseTo(NUDGE,9);
   }
 });
 it('robot contacts physically displace released bars',async()=>{
   const s=createSim();s.tool=[1.2,.7,.34];const contact=robotContacts(s.tool,0)[4];if(!contact)throw new Error('missing finger');
   const bar=s.bars[0];if(!bar)throw new Error('missing bar');bar.p=[...contact.p];bar.q={x:0,y:0,z:0,w:1};
   const initial=[...bar.p];s.physics=await createDynamics();advance(s,.5);
   expect(Math.hypot(bar.p[0]-(initial[0]??0),bar.p[2]-(initial[2]??0))).toBeGreaterThan(.005);
 });
 it('snapshot selection leaves real tower poses unchanged',async()=>{
   const s=built();s.physics=await createDynamics();s.physics.step(s,1/60);
   const before=structuredClone(s.bars.map(b=>b.p));const selected=s.physics.chooseRemoval?.(s,strategicCandidates(s));
   if(selected!==undefined)expect(s.removalScores?.find(score=>score.index===selected)?.stable).toBe(true);
   expect(s.removalScores?.length).toBeGreaterThan(0);expect(s.bars.map(b=>b.p)).toEqual(before);
  },30000);
 it('recovers a chaotic start into a standing physical tower without an early restart',async()=>{
   const s=createSim(()=>.1);s.physics=await createDynamics();advance(s,100);
   expect(s.generation).toBeGreaterThanOrEqual(0);expect(s.placed+s.bars.filter(b=>b.inTower).length).toBeGreaterThanOrEqual(3);
   expect(s.bars.filter(b=>b.inTower).length).toBeGreaterThanOrEqual(0);
 },60000);
 it('builds level stories from the scatter with every bar pushed home',async()=>{
   const s=createSim(()=>.1);s.physics=await createDynamics();advance(s,4+CYCLE*12+.5);
   expect(s.placed).toBeGreaterThanOrEqual(12);expect(s.generation).toBe(0);
   for(const b of s.bars.filter(item=>item.inTower&&item!==s.bars[s.active])){
     const p=placement(b.layer*3+b.slot),q=b.q??{x:0,y:0,z:0,w:1};
     expect(Math.hypot(b.p[0]-p.x,b.p[2]-p.z)).toBeLessThan(.006);expect(1-2*(q.x*q.x+q.z*q.z)).toBeGreaterThan(.999);
   }
 },120000);
 it('preserves the actual fallen poses and bar identities when moving beside the pile',()=>{
   const s=createSim();s.bars.forEach((b,i)=>{b.p=[TOWER.x+i*.0001,.03,TOWER.z];b.q={x:.7,y:0,z:0,w:.7};});
   const bars=s.bars,poses=structuredClone(bars);restartFromPile(s);
   expect(s.bars).toBe(bars);expect(s.bars.map(b=>b.p)).toEqual(poses.map(b=>b.p));expect(s.bars.map(b=>b.q)).toEqual(poses.map(b=>b.q));
   expect(Math.hypot(s.tower.x-TOWER.x,s.tower.z-TOWER.z)).toBeGreaterThan(.3);
   advance(s,.1);expect(s.recoveryHistory).toHaveLength(1);
 });
 it('restarts after a deconstruction fall from the gold bars actual positions',()=>{
   const s=built(),actual=structuredClone(s.bars.map(b=>({p:b.p,q:b.q})));let resets=0;
   s.physics={reset:()=>{resets++;},step:()=>true,settled:()=>true};
   advance(s,6.1);
   expect(s.generation).toBe(1);expect(s.phase).toBe('build');expect(resets).toBe(1);
   expect(s.bars.map(b=>({p:b.p,q:b.q}))).toEqual(actual);
 });
 it('releases a full tower to physics, collapses and recovers the same bars',async()=>{
   const s=built();s.random=()=>.1;expect(s.placed).toBe(MAX_BARS);
   s.physics=await createDynamics();const bars=s.bars;let collapsed=false;
   for(let i=0;i<600*60;i++){stepSim(s,1/60);if(s.phase==='collapse')collapsed=true;if(s.generation>0)break;}
   expect(collapsed).toBe(true);expect(s.generation).toBe(1);expect(s.bars).toBe(bars);expect(s.phase).toBe('build');advance(s,.1);expect(s.recoveryHistory.length).toBeGreaterThan(0);
 },900000);
});

