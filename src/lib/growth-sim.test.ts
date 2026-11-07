import {describe,expect,it} from 'vitest';
import {BAR,MAX_BARS,ROBOT_BASE,ROBOT_SCALE,createSim,stepSim,placement,eligibleBars,solveIK,CYCLE} from './growth-sim';
import {createDynamics} from './growth-physics';
describe('floor-level Jenga sculpture',()=>{
 it('keeps lingots at 15 × 5 × 1 cm',()=>{expect(BAR).toEqual({L:.15,W:.05,H:.01});});
 it('seats the robot footprint directly on the floor',()=>{expect(ROBOT_BASE.y-.003181*ROBOT_SCALE).toBeCloseTo(0,8);});
 it('builds three parallel bars per layer alternating 90 degrees',()=>{
   for(let i=0;i<MAX_BARS;i++){const p=placement(i);expect(p.layer).toBe(Math.floor(i/3));expect(p.slot).toBe(i%3);expect(p.yaw).toBe(Math.floor(i/3)%2?Math.PI/2:0);}
   expect(placement(0).z).toBeCloseTo(.29);expect(placement(2).z).toBeCloseTo(.39);expect(placement(3).x).toBeCloseTo(.29);
 });
 it('releases the first bar on the concrete floor',()=>{
   const s=createSim();for(let i=0;i<Math.ceil(CYCLE*.9*60);i++)stepSim(s,1/60);
   expect(s.bars[0]?.p[1]).toBeCloseTo(.005,8);expect(s.carry).toBe(false);expect(Object.values(solveIK(s.tool,s.toolYaw)).every(Number.isFinite)).toBe(true);
 });
 it('selects random eligible bars below the highest completed layer',()=>{
   const s=createSim(()=>.7);for(let i=0;i<(MAX_BARS*CYCLE+1)*60;i++)stepSim(s,1/60);
   expect(s.phase).toBe('remove');expect(s.removalHistory).toHaveLength(1);
   const chosen=s.removalHistory[0];expect(chosen).toBe(35);expect(eligibleBars(s)).not.toContain(53);
 });
 it('uses rigid-body dynamics to fall after random removals and restarts',async()=>{
   const s=createSim(()=>.82);s.physics=await createDynamics();let collapsed=false,restarted=false;
   for(let i=0;i<600*60;i++){stepSim(s,1/60);if(s.phase==='collapse')collapsed=true;if(collapsed&&s.phase==='build'){restarted=true;break;}}
   expect(collapsed).toBe(true);expect(restarted).toBe(true);
 },30000);
 it('keeps a completed tower stable before the extraction begins',async()=>{
   const s=createSim(()=>.5);s.physics=await createDynamics();
   while(s.phase==='build')stepSim(s,1/60);
   for(let i=0;i<24;i++)stepSim(s,1/60);
   expect(s.phase).toBe('remove');expect(s.carry).toBe(false);
   expect(s.bars[53]?.p[1]).toBeCloseTo(.175,3);
 });
});
