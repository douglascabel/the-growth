import {describe,expect,it} from 'vitest';
import * as THREE from 'three';
import {isLoose,BAR,SLOT,MAX_BARS,LAYERS,TOWER,ROBOT_BASE,ROBOT_SCALE,GRIPPER_MODEL_HALF_SPAN,GRIPPER_CLOSE_TRAVEL,GRIPPER_FINGER_HALF_WIDTH,PUSH_GRIP,PUSH_STROKE,REMOVE_CYCLE,END_GRIP,PINCH_GRIP,createSim,stepSim,placement,eligibleBars,canReach,restartFromPile,strategicCandidates,supportMargin,REACH_CLEARANCE} from './growth-sim';
import {createDynamics} from './growth-physics';
import {GRIPPER_FINGER_RADIUS,gripHalfSpan,robotContacts} from './robot-kinematics';
import {normalizeLingot} from './lingot-geometry';
const advance=(s:ReturnType<typeof createSim>,seconds:number)=>{for(let i=0;i<Math.round(seconds*60);i++)stepSim(s,1/60);};
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
 it('seats the robot on the floor',()=>{expect(ROBOT_BASE.y-.003181*ROBOT_SCALE).toBeCloseTo(0,8);});
 it('alternates three parallel bars by 90 degrees',()=>{
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(p.layer).toBe(Math.floor(i/3));expect(p.slot).toBe(i%3);expect(p.yaw).toBe(Math.floor(i/3)%2?Math.PI/2:0);}
   expect(placement(0).z).toBeCloseTo(TOWER.z-SLOT,6);expect(placement(2).z).toBeCloseTo(TOWER.z+SLOT,6);
 });
 it('stops at the last complete reachable layer',()=>{
   expect(LAYERS).toBeGreaterThanOrEqual(30);
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(canReach([p.x,(p.layer+1)*BAR.H+.02,p.z])).toBe(true);}
   expect([0,1,2].some(slot=>{const p=placement(MAX_BARS+slot);return !canReach([p.x,(p.layer+1)*BAR.H+REACH_CLEARANCE,p.z]);})).toBe(true);
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
 it('uses a closed-picker push on only one bar',()=>{
   const s=built();advance(s,1.6);const index=s.active;if(index<0)throw new Error('no safe removal was selected');
   let tested=false,closed=false;
   for(let i=0;i<Math.ceil(REMOVE_CYCLE*60);i++){
     stepSim(s,1/60);
     if(s.push){tested=true;expect(s.carry).toBe(false);expect(s.closed).toBe(true);expect(s.grip).toBe(PUSH_GRIP);expect(s.active).toBe(index);}
     if(s.closed)closed=true;
   }
   expect(PUSH_STROKE).toBeLessThanOrEqual(.02);expect(tested).toBe(true);expect(closed).toBe(true);
 });
 it('keeps the pushed bar in its layer instead of picking it up',()=>{
   const s=built();s.random=()=>.1;advance(s,1.6);
   const index=s.active;if(index<0)throw new Error('no safe removal was selected');
   const layer=s.bars[index]?.layer;
   advance(s,REMOVE_CYCLE+.2);
   const moved=s.bars[index];expect(moved?.inTower).toBe(true);expect(moved?.layer).toBe(layer);expect(s.moves).toBe(0);
 });
 it('does not start a removal when no reachable top placement exists',()=>{
   const s=built();s.tower={x:ROBOT_BASE.x+2,z:TOWER.z};advance(s,2);
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
   const s=built(),d=await createDynamics();d.step(s,1/60);s.physics={...d,step:()=>false,settled:()=>true};
   advance(s,1.6);
   expect(s.removalHistory).toHaveLength(1);expect(s.active).toBeGreaterThanOrEqual(0);
 });
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
   const a=robotContacts([.34,.7,.34],0),b=robotContacts([.1,1,.34],0);
   expect(a.map(c=>c.name)).toEqual(['shoulder','upper-arm','forearm','wrist','finger--1','finger-1']);
   expect(a.every(c=>[...c.p,...Object.values(c.q)].every(Number.isFinite))).toBe(true);
   expect(a[2]?.p).not.toEqual(b[2]?.p);
 });
 it('uses the same open and closed gripper span for rendered and physical fingers',()=>{
   expect(gripHalfSpan(PINCH_GRIP,false)).toBe(PINCH_GRIP.open);expect(gripHalfSpan(PINCH_GRIP,true)).toBe(PINCH_GRIP.closed);
   const open=robotContacts([.34,.7,.34],0,false,PINCH_GRIP).slice(4),closed=robotContacts([.34,.7,.34],0,true,PINCH_GRIP).slice(4);
   expect(Math.abs((open[1]?.p[0]??0)-(open[0]?.p[0]??0))).toBeCloseTo(PINCH_GRIP.open*2,6);
   expect(Math.abs((closed[1]?.p[0]??0)-(closed[0]?.p[0]??0))).toBeCloseTo(PINCH_GRIP.closed*2,6);
 });
 it('uses the authored picker jaw layout and mesh thickness for render and contacts',()=>{
   expect(END_GRIP.open).toBeCloseTo(GRIPPER_MODEL_HALF_SPAN*ROBOT_SCALE,8);
   expect(END_GRIP.closed).toBeCloseTo((GRIPPER_MODEL_HALF_SPAN-GRIPPER_CLOSE_TRAVEL)*ROBOT_SCALE,8);
   expect(PINCH_GRIP.open).toBe(END_GRIP.open);expect(PINCH_GRIP.closed).toBe(END_GRIP.closed);
   expect(GRIPPER_FINGER_RADIUS).toBe(GRIPPER_FINGER_HALF_WIDTH);
 });
 it('robot contacts physically displace released bars',async()=>{
   const s=createSim();s.tool=[.34,.7,.34];const contact=robotContacts(s.tool,0)[4];if(!contact)throw new Error('missing finger');
   const bar=s.bars[0];if(!bar)throw new Error('missing bar');bar.p=[...contact.p];bar.q={x:0,y:0,z:0,w:1};
   const initial=[...bar.p];s.physics=await createDynamics();advance(s,.5);
   expect(Math.hypot(bar.p[0]-(initial[0]??0),bar.p[2]-(initial[2]??0))).toBeGreaterThan(.005);
 });
 it('snapshot selection leaves real tower poses unchanged',async()=>{
   const s=built();s.physics=await createDynamics();s.physics.step(s,1/60);
   const before=structuredClone(s.bars.map(b=>b.p));const selected=s.physics.chooseRemoval?.(s,strategicCandidates(s));
   if(selected!==undefined)expect(s.removalScores?.find(score=>score.index===selected)?.stable).toBe(true);
   expect(s.removalScores?.length).toBeGreaterThan(0);expect(s.bars.map(b=>b.p)).toEqual(before);
 });
 it('recovers a chaotic start into a standing physical tower without an early restart',async()=>{
   const s=createSim(()=>.1);s.physics=await createDynamics();advance(s,100);
   expect(s.generation).toBe(0);expect(s.placed).toBeGreaterThanOrEqual(15);
   expect(s.bars.filter(b=>b.inTower&&b.p[1]>.15).length).toBeGreaterThanOrEqual(3);
 },60000);
 it('builds to full reach and recovers the same bars after a fall',async()=>{
   const s=createSim(()=>.1);s.physics=await createDynamics();const bars=s.bars;
   advance(s,640);expect(s.generation).toBe(1);expect(s.phase).toBe('build');
   expect(s.recoveryHistory.length).toBeGreaterThanOrEqual(MAX_BARS);
   const recovered=s.recoveryHistory.length;advance(s,120);
   expect(s.generation).toBe(1);expect(s.bars).toBe(bars);expect(s.phase).toBe('build');
   expect(s.recoveryHistory.length).toBeGreaterThan(recovered);
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
 },60000);
});

