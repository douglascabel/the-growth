import {describe,expect,it} from 'vitest';
import * as THREE from 'three';
import {BAR,MAX_BARS,LAYERS,TOWER,ROBOT_BASE,ROBOT_SCALE,createSim,stepSim,placement,eligibleBars,CYCLE,canReach,restartFromPile} from './growth-sim';
import {createDynamics} from './growth-physics';
import {normalizeLingot} from './lingot-geometry';
const advance=(s:ReturnType<typeof createSim>,seconds:number)=>{for(let i=0;i<Math.round(seconds*60);i++)stepSim(s,1/60);};
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
   expect(placement(0).z).toBeCloseTo(.27786,6);expect(placement(2).z).toBeCloseTo(.40214,6);
 });
 it('stops at the last complete reachable layer',()=>{
   expect(LAYERS).toBe(32);
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(canReach([p.x,(p.layer+1)*BAR.H+.02,p.z])).toBe(true);}
   expect([0,1,2].some(slot=>{const p=placement(MAX_BARS+slot);return !canReach([p.x,(p.layer+1)*BAR.H+.02,p.z]);})).toBe(true);
 });
 it('only chooses bars below the highest completed layer',()=>{
   const s=createSim(()=>.7);advance(s,MAX_BARS*CYCLE+.1);expect(s.phase).toBe('remove');expect(s.removalHistory).toHaveLength(1);
   expect(s.removalHistory[0]).toBe(65);expect(eligibleBars(s)).not.toContain(95);
 });
 it('selects both push/pull methods and both opposite ends',()=>{
   for(const [r,mode,side] of [[.1,'pull',-1],[.9,'push',1]] as const){const s=createSim(()=>r);advance(s,MAX_BARS*CYCLE+.1);expect(s.mode).toBe(mode);expect(s.side).toBe(side);}
 });
 it('preserves the actual fallen poses and bar identities when moving beside the pile',()=>{
   const s=createSim();s.bars.forEach((b,i)=>{b.p=[.34+i*.0001,.03,.34];b.q={x:.7,y:0,z:0,w:.7};});
   const bars=s.bars,poses=structuredClone(bars);restartFromPile(s);
   expect(s.bars).toBe(bars);expect(s.bars.map(b=>b.p)).toEqual(poses.map(b=>b.p));expect(s.bars.map(b=>b.q)).toEqual(poses.map(b=>b.q));
   expect(Math.hypot(s.tower.x-TOWER.x,s.tower.z-TOWER.z)).toBeGreaterThan(.3);
   advance(s,.1);expect(s.recoveryHistory).toHaveLength(1);
 });
 it('releases a full tower to physics, collapses and recovers the same bars',async()=>{
   const s=createSim(()=>.82);advance(s,MAX_BARS*CYCLE);expect(s.phase).toBe('remove');expect(s.placed).toBe(96);
   s.physics=await createDynamics();const bars=s.bars;let collapsed=false;
   for(let i=0;i<180*60;i++){stepSim(s,1/60);if(s.phase==='collapse')collapsed=true;if(s.generation>0)break;}
   expect(collapsed).toBe(true);expect(s.generation).toBe(1);expect(s.bars).toBe(bars);expect(s.phase).toBe('build');advance(s,.1);expect(s.recoveryHistory.length).toBeGreaterThan(0);
 },60000);
});
