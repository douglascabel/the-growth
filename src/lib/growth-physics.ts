import RAPIER from '@dimforge/rapier3d-compat';
import { BAR, TAB, type Dynamics } from './growth-sim';
import { robotContacts } from './robot-kinematics';
let ready:Promise<void>|undefined;
export async function createDynamics(hull?:Float32Array):Promise<Dynamics> {
  ready??=RAPIER.init();await ready;
  let world:RAPIER.World;
  let bodies=new Map<number,RAPIER.RigidBody>();
  let pusher:RAPIER.RigidBody|undefined;
  let quietTime=0;
  let robot:RAPIER.RigidBody[]=[];
  const reset=()=>{
    world?.free();world=new RAPIER.World({x:0,y:-9.81,z:0});world.numSolverIterations=40;
    world.integrationParameters.contact_natural_frequency=60;
    world.integrationParameters.lengthUnit=.1;
    world.integrationParameters.normalizedAllowedLinearError=.0005;
    world.integrationParameters.normalizedPredictionDistance=.01;
    bodies=new Map();pusher=undefined;quietTime=0;robot=[];
    world.createCollider(RAPIER.ColliderDesc.cuboid(20,.1,20).setTranslation(0,-.1,0).setFriction(.7));
  };reset();
  return {reset,settled:()=>quietTime>1.5,chooseRemoval(s,candidates){
    const snapshot=world.takeSnapshot();
    const scores:{index:number;stable:boolean;motion:number}[]=[];
    // Probe the actual contact network without moving the visible world or pinning supports.
    for(const index of candidates.slice(0,3)){
      const original=bodies.get(index);if(!original)continue;
      const probe=RAPIER.World.restoreSnapshot(snapshot);
      const target=probe.getRigidBody(original.handle);if(target)probe.removeRigidBody(target);
      probe.timestep=1/120;
      for(let n=0;n<30;n++)probe.step();
      let motion=0,stable=true;
      for(const [i,body] of bodies){
        const b=s.bars[i];if(!b?.inTower||i===index)continue;
        const other=probe.getRigidBody(body.handle);if(!other)continue;
        const p=other.translation(),q=other.rotation();
        const displacement=Math.hypot(p.x-b.p[0],p.y-b.p[1],p.z-b.p[2]);
        motion=Math.max(motion,displacement);
        if(displacement>.008||1-2*(q.x*q.x+q.z*q.z)<.96)stable=false;
      }
      probe.free();scores.push({index,stable,motion});
      // Bounded lookahead avoids freezing the visitor's scene; support margins shortlist options.
      if(scores.filter(score=>score.stable).length>=3)break;
    }
    scores.sort((a,b)=>Number(b.stable)-Number(a.stable)||a.motion-b.motion);
    s.removalScores=scores;
    return scores[0]?.index;
  },step(s,dt){
    for(const [i,b] of s.bars.entries()){
      let body=bodies.get(i);
      // Recreate on release: a former kinematic body keeps solver state that kicks it on handover.
      if(body&&!(s.active===i&&s.carry)&&body.bodyType()===RAPIER.RigidBodyType.KinematicPositionBased){world.removeRigidBody(body);bodies.delete(i);body=undefined;}
      if(!body){
        body=world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(...b.p).setRotation(b.q??{x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)}).setLinearDamping(.5).setAngularDamping(.8).setCcdEnabled(true));
        // Real bars vary slightly in height: a middle bar a hair thinner carries no load, the loose block Jenga players test for.
        const collider=RAPIER.ColliderDesc.cuboid(BAR.L/2*.98,BAR.H/2*(b.inTower&&b.slot===1?.92:.95),BAR.W/2*.96);
        world.createCollider(collider.setDensity(19300).setFriction(.32).setRestitution(.01),body);bodies.set(i,body);
      }
      if(s.active===i&&s.carry){
        if(body.bodyType()!==RAPIER.RigidBodyType.KinematicPositionBased)body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased,true);
      }else if(body.bodyType()!==RAPIER.RigidBodyType.Dynamic){
        body.setBodyType(RAPIER.RigidBodyType.Dynamic,true);
        body.setTranslation({x:b.p[0],y:b.p[1],z:b.p[2]},true);body.setRotation(b.q??{x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)},true);
        body.setLinvel({x:0,y:0,z:0},true);body.setAngvel({x:0,y:0,z:0},true);
      }
      // A grasped bar still contacts the tower, but not its own fingers or links.
      body.collider(0).setCollisionGroups(s.active===i&&s.carry?0x00020001:0x00010005);
    }
    const contacts=robotContacts(s.tool,s.toolYaw,s.carry,s.grip);
    if(!robot.length){
      for(const contact of contacts){
        const body=world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(...contact.p).setRotation(contact.q));
        world.createCollider(RAPIER.ColliderDesc.capsule(contact.halfHeight,contact.radius).setFriction(.35).setCollisionGroups(0x00040001),body);
        robot.push(body);
      }
    }
    const robotStarts=robot.map(body=>({p:body.translation(),q:body.rotation()}));
    // Pushes transfer force through contact; the target remains fully dynamic.
    if(s.push){
      if(!pusher){pusher=world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(...s.push.p).setRotation({x:0,y:Math.sin(s.push.yaw/2),z:0,w:Math.cos(s.push.yaw/2)}));world.createCollider(RAPIER.ColliderDesc.cuboid(TAB.L/2,TAB.H/2,TAB.W/2).setFriction(.4),pusher);}
    }else if(pusher){world.removeRigidBody(pusher);pusher=undefined;}
    // Fixed substeps keep stacking and contact independent of render frame rate.
    const steps=Math.max(1,Math.ceil(dt/(1/120)));world.timestep=dt/steps;
    const held=bodies.get(s.active),bar=s.bars[s.active],start=held?.translation(),startQ=held?.rotation();
    const pushStart=pusher?.translation();
    for(let i=0;i<steps;i++){
      const u=(i+1)/steps;
      robot.forEach((body,j)=>{
        const start=robotStarts[j],end=contacts[j];if(!start||!end)return;
        body.setNextKinematicTranslation({x:start.p.x+(end.p[0]-start.p.x)*u,y:start.p.y+(end.p[1]-start.p.y)*u,z:start.p.z+(end.p[2]-start.p.z)*u});
        const sign=start.q.x*end.q.x+start.q.y*end.q.y+start.q.z*end.q.z+start.q.w*end.q.w<0?-1:1;
        const q={x:start.q.x+(end.q.x*sign-start.q.x)*u,y:start.q.y+(end.q.y*sign-start.q.y)*u,z:start.q.z+(end.q.z*sign-start.q.z)*u,w:start.q.w+(end.q.w*sign-start.q.w)*u};
        const n=Math.hypot(q.x,q.y,q.z,q.w);body.setNextKinematicRotation({x:q.x/n,y:q.y/n,z:q.z/n,w:q.w/n});
      });
      if(s.carry&&held&&bar&&start&&startQ){
        held.setNextKinematicTranslation({x:start.x+(bar.p[0]-start.x)*u,y:start.y+(bar.p[1]-start.y)*u,z:start.z+(bar.p[2]-start.z)*u});
        const q=bar.q??{x:0,y:Math.sin(bar.yaw/2),z:0,w:Math.cos(bar.yaw/2)};
        held.setNextKinematicRotation({x:startQ.x+(q.x-startQ.x)*u,y:startQ.y+(q.y-startQ.y)*u,z:startQ.z+(q.z-startQ.z)*u,w:startQ.w+(q.w-startQ.w)*u});
      }
      if(pusher&&s.push&&pushStart)pusher.setNextKinematicTranslation({x:pushStart.x+(s.push.p[0]-pushStart.x)*u,y:pushStart.y+(s.push.p[1]-pushStart.y)*u,z:pushStart.z+(s.push.p[2]-pushStart.z)*u});
      world.step();
    }
    let fallen=false,quiet=true;
    for(const [i,body] of bodies){
      const b=s.bars[i];if(!b||(s.active===i&&s.carry))continue;
      const p=body.translation(),q=body.rotation();b.p=[p.x,p.y,p.z];b.q={...q};
      const v=body.linvel(),a=body.angvel();if(Math.hypot(v.x,v.y,v.z)>.025||Math.hypot(a.x,a.y,a.z)>.1)quiet=false;
      if(b.inTower&&b.layer>1&&(1-2*(q.x*q.x+q.z*q.z)<.85||p.y<(b.layer+.5)*BAR.H*.75))fallen=true;
    }
    quietTime=quiet?quietTime+dt:0;return fallen;
  }};
}
