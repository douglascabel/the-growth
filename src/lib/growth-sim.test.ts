import {describe,expect,it} from 'vitest';
import * as THREE from 'three';
import {BAR,SLOT,MAX_BARS,LAYERS,TOWER,ROBOT_BASE,ROBOT_SCALE,createSim,stepSim,placement,eligibleBars,CYCLE,canReach,restartFromPile,strategicCandidates,supportMargin} from './growth-sim';
import {createDynamics} from './growth-physics';
import {robotContacts} from './robot-kinematics';
import {normalizeLingot} from './lingot-geometry';
const advance=(s:ReturnType<typeof createSim>,seconds:number)=>{for(let i=0;i<Math.round(seconds*60);i++)stepSim(s,1/60);};
const built=()=>{const s=createSim();s.phase='remove';s.placed=MAX_BARS;s.bars.forEach((b,i)=>{const p=placement(i);b.p=[p.x,(p.layer+.5)*BAR.H*.95,p.z];b.yaw=p.yaw;b.q={x:0,y:Math.sin(p.yaw/2),z:0,w:Math.cos(p.yaw/2)};b.layer=p.layer;b.slot=p.slot;b.inTower=true;});return s;};
describe('reach-limited sculpture',()=>{
 it('uses 155 × 41.45 × 62.14 mm',()=>{expect(BAR).toEqual({L:.155,H:.04145,W:.06214});});
 it('preserves the supplied positive-Y inscription face',()=>{
   const g=new THREE.BoxGeometry(155.37,41.45,62.14);const normalized=normalizeLingot(g);normalized.computeBoundingBox();
   const size=normalized.boundingBox?.getSize(new THREE.Vector3());expect(size?.x).toBeCloseTo(.155,6);expect(size?.y).toBeCloseTo(.04145,6);expect(size?.z).toBeCloseTo(.06214,6);
   const original=g.getAttribute('position'),p=normalized.getAttribute('position');for(let i=0;i<p.count;i++)if(original.getY(i)>0)expect(p.getY(i)).toBeGreaterThan(0);
 });
 it('seats the robot on the floor',()=>{expect(ROBOT_BASE.y-.003181*ROBOT_SCALE).toBeCloseTo(0,8);});
 it('alternates three parallel bars by 90 degrees',()=>{
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(p.layer).toBe(Math.floor(i/3));expect(p.slot).toBe(i%3);expect(p.yaw).toBe(Math.floor(i/3)%2?Math.PI/2:0);}
   expect(placement(0).z).toBeCloseTo(.34-SLOT,6);expect(placement(2).z).toBeCloseTo(.34+SLOT,6);
 });
 it('stops at the last complete reachable layer',()=>{
   expect(LAYERS).toBe(32);
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(canReach([p.x,(p.layer+1)*BAR.H+.02,p.z])).toBe(true);}
   expect([0,1,2].some(slot=>{const p=placement(MAX_BARS+slot);return !canReach([p.x,(p.layer+1)*BAR.H+.02,p.z]);})).toBe(true);
 });
 it('only chooses bars below the highest completed layer',()=>{
   const s=built();advance(s,1.6);expect(s.phase).toBe('remove');expect(s.removalHistory).toHaveLength(1);
   const chosen=s.removalHistory[0];expect(chosen===undefined?99:s.bars[chosen]?.layer).toBeLessThan(31);
   expect(eligibleBars(s).every(i=>(s.bars[i]?.layer??99)<31)).toBe(true);
 });
 it('selects both careful push and pull methods',()=>{
   for(const [r,mode] of [[.1,'pull'],[.9,'push']] as const){const s=built();s.random=()=>r;advance(s,1.6);expect(s.mode).toBe(mode);}
 });
 it('starts from chaotic tilted bars and waits for settling before recovery',()=>{
   const s=createSim();expect(s.phase).toBe('settle');expect(s.bars).toHaveLength(96);
   expect(new Set(s.bars.map(b=>b.yaw.toFixed(2))).size).toBeGreaterThan(50);
   expect(s.bars.every(b=>b.p[1]<.17&&!b.inTower)).toBe(true);
   advance(s,1.9);expect(s.active).toBe(-1);advance(s,.3);expect(s.phase).toBe('build');expect(s.recoveryHistory).toHaveLength(1);
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
 it('robot contacts physically displace released bars',async()=>{
   const s=createSim();s.tool=[.34,.7,.34];const contact=robotContacts(s.tool,0)[2];if(!contact)throw new Error('missing forearm');
   const bar=s.bars[0];if(!bar)throw new Error('missing bar');bar.p=[...contact.p];bar.q={x:0,y:0,z:0,w:1};
   const initial=[...bar.p];s.physics=await createDynamics();advance(s,.5);
   expect(Math.hypot(bar.p[0]-(initial[0]??0),bar.p[2]-(initial[2]??0))).toBeGreaterThan(.01);
 });
 it('snapshot selection leaves real tower poses unchanged',async()=>{
   const s=built();s.physics=await createDynamics();s.physics.step(s,1/60);
   const before=structuredClone(s.bars.map(b=>b.p));const selected=s.physics.chooseRemoval?.(s,strategicCandidates(s));
   expect(selected).toBeDefined();expect(s.removalScores?.length).toBeGreaterThan(0);expect(s.bars.map(b=>b.p)).toEqual(before);
 });
 it('recovers a chaotic start into a standing physical tower without an early restart',async()=>{
   const s=createSim(()=>.1);s.physics=await createDynamics();advance(s,100);
   expect(s.generation).toBe(0);expect(s.placed).toBeGreaterThanOrEqual(15);
   expect(s.bars.filter(b=>b.inTower&&b.p[1]>.15).length).toBeGreaterThanOrEqual(3);
 },60000);
 it('preserves the actual fallen poses and bar identities when moving beside the pile',()=>{
   const s=createSim();s.bars.forEach((b,i)=>{b.p=[.34+i*.0001,.03,.34];b.q={x:.7,y:0,z:0,w:.7};});
   const bars=s.bars,poses=structuredClone(bars);restartFromPile(s);
   expect(s.bars).toBe(bars);expect(s.bars.map(b=>b.p)).toEqual(poses.map(b=>b.p));expect(s.bars.map(b=>b.q)).toEqual(poses.map(b=>b.q));
   expect(Math.hypot(s.tower.x-TOWER.x,s.tower.z-TOWER.z)).toBeGreaterThan(.3);
   advance(s,.1);expect(s.recoveryHistory).toHaveLength(1);
 });
 it('releases a full tower to physics, collapses and recovers the same bars',async()=>{
   const s=built();s.random=()=>.1;expect(s.placed).toBe(96);
   s.physics=await createDynamics();const bars=s.bars;let collapsed=false;
   for(let i=0;i<600*60;i++){stepSim(s,1/60);if(s.phase==='collapse')collapsed=true;if(s.generation>0)break;}
   expect(collapsed).toBe(true);expect(s.generation).toBe(1);expect(s.bars).toBe(bars);expect(s.phase).toBe('build');advance(s,.1);expect(s.recoveryHistory.length).toBeGreaterThan(0);
 },60000);
});
