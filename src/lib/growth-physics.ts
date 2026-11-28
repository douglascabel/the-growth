import RAPIER from '@dimforge/rapier3d-compat';
import { BAR, BAR_CONTACT, BAR_FRICTION, PUSH_STROKE, SLIDE_FRICTION, TAB, type Dynamics } from './growth-sim';
import { armClear, armClearMany, robotContacts } from './robot-kinematics';
let ready:Promise<void>|undefined;
export async function createDynamics(hull?:Float32Array):Promise<Dynamics> {
  ready??=RAPIER.init();await ready;
  let world:RAPIER.World;
  let bodies=new Map<number,RAPIER.RigidBody>();
  let pusher:RAPIER.RigidBody|undefined;
  let quietTime=0;
  let robot:RAPIER.RigidBody[]=[];
  const reset=()=>{
    world?.free();world=new RAPIER.World({x:0,y:-9.81,z:0});world.numSolverIterations=72;
    world.integrationParameters.contact_natural_frequency=80;
    world.integrationParameters.lengthUnit=.1;
    world.integrationParameters.normalizedAllowedLinearError=.0005;
    world.integrationParameters.normalizedPredictionDistance=.01;
    bodies=new Map();pusher=undefined;quietTime=0;robot=[];
    world.createCollider(RAPIER.ColliderDesc.cuboid(20,.1,20).setTranslation(0,-.1,0).setFriction(.62).setRestitution(.005));
  };reset();
  return {reset,settled:()=>quietTime>1.5,armClear:(tool,yaw,box,rig)=>armClear(tool,yaw,box,rig),armClearMany:(tool,yaw,boxes,rig)=>armClearMany(tool,yaw,boxes,rig),chooseRemoval(s,candidates,routes=[]){
    const snapshot=world.takeSnapshot();
    const scores:{index:number;stable:boolean;motion:number}[]=[];
    // Probe the actual contact network without moving the visible world or pinning supports.
    for(const index of candidates.slice(0,3)){
      const original=bodies.get(index);if(!original)continue;
      const pulled=RAPIER.World.restoreSnapshot(snapshot),moving=pulled.getRigidBody(original.handle),bar=s.bars[index];
      if(!moving||!bar){pulled.free();continue;}
      let motion=0,stable=true;
      const observe=()=>{
        for(const [i,body] of bodies){
          const b=s.bars[i];if(!b?.inTower||i===index)continue;
          const other=pulled.getRigidBody(body.handle);if(!other)continue;
          const p=other.translation(),q=other.rotation(),displacement=Math.hypot(p.x-b.p[0],p.y-b.p[1],p.z-b.p[2]);
          motion=Math.max(motion,displacement);
          // A short push may settle adjacent bars by a few millimetres.  Reject
          // only a meaningful shift or lean, which is the onset of a fall.
          if(displacement>.06||1-2*(q.x*q.x+q.z*q.z)<.82)stable=false;
        }
      };
      moving.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased,true);
      // Probe the selected bar with the same short closed-hand push used live.
      moving.collider(0).setCollisionGroups(0x00020001);moving.collider(0).setFriction(SLIDE_FRICTION);
      const start=moving.translation(),side=routes.find(route=>route.index===index)?.side??1,yaw=bar.yaw;
      const end={x:start.x+Math.cos(yaw)*side*PUSH_STROKE,y:start.y,z:start.z-Math.sin(yaw)*side*PUSH_STROKE};
      pulled.timestep=1/120;
      for(let n=1;n<=240;n++){
        const u=n/240;moving.setNextKinematicTranslation({x:start.x+(end.x-start.x)*u,y:start.y,z:start.z+(end.z-start.z)*u});
        pulled.step();observe();
      }
      for(let n=0;n<120;n++){pulled.step();observe();}
      pulled.free();
      scores.push({index,stable,motion});
    }
    const rank=new Map(candidates.map((index,position)=>[index,position]));
    scores.sort((a,b)=>Number(b.stable)-Number(a.stable)||
      (Math.abs(a.motion-b.motion)<.002?(rank.get(a.index)??Infinity)-(rank.get(b.index)??Infinity):a.motion-b.motion));
    s.removalScores=scores;
    // Do not turn a known-unstable probe into a live extraction. The tower remains
    // untouched until another bar has a stable route.
    return scores.find(score=>score.stable)?.index;
  },step(s,dt){
    for(const [i,b] of s.bars.entries()){
      let body=bodies.get(i);
      // Recreate on release: a former kinematic body keeps solver state that kicks it on handover.
      if(body&&!(s.active===i&&s.carry)&&body.bodyType()===RAPIER.RigidBodyType.KinematicPositionBased){world.removeRigidBody(body);bodies.delete(i);body=undefined;}
      if(!body){
        body=world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(...b.p).setRotation(b.q??{x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)}).setLinearDamping(.85).setAngularDamping(1.2).setCcdEnabled(true));
        // Uniform, nearly full-size contact hulls keep each three-bar story level.
        const collider=RAPIER.ColliderDesc.cuboid(BAR_CONTACT.L/2,BAR_CONTACT.H/2,BAR_CONTACT.W/2);
        world.createCollider(collider.setDensity(19300).setFriction(BAR_FRICTION).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(.005),body);bodies.set(i,body);
      }
      if(s.active===i&&s.carry){
        if(body.bodyType()!==RAPIER.RigidBodyType.KinematicPositionBased)body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased,true);
      }else if(body.bodyType()!==RAPIER.RigidBodyType.Dynamic){
        body.setBodyType(RAPIER.RigidBodyType.Dynamic,true);
        body.setTranslation({x:b.p[0],y:b.p[1],z:b.p[2]},true);body.setRotation(b.q??{x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)},true);
        body.setLinvel({x:0,y:0,z:0},true);body.setAngvel({x:0,y:0,z:0},true);
      }
      // Keep established placement contacts during construction. In deconstruction,
      // isolate the selected bar so only its own hand and the tower can touch it.
      const selected=s.active===i;
      // Before grip closure only the small test tab can touch the selected bar.
      // Once closed, the carried bar stays physical against the tower but ignores
      // the robot colliders because its kinematic pose is the closed picker hold.
      // A released bar in construction also meets the push tab that sets it home.
      const groups=!selected?0x00010007:s.carry?0x00020001:s.phase!=='remove'?0x00020005:
        b.inTower?0x00020005:0x00020004;
      body.collider(0).setCollisionGroups(groups);
      // The selected bar slides alone while pushed home, and through a careful extraction
      // until it is set on top; at rest it grips like every other bar.
      const sliding=selected&&(!!s.push||s.carry||s.phase==='remove'&&!s.released);
      body.collider(0).setFriction(sliding?SLIDE_FRICTION:BAR_FRICTION);
    }
    const contacts=robotContacts(s.tool,s.toolYaw,s.jaw,s.rig);
    if(!robot.length){
      for(const contact of contacts){
        const body=world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(...contact.p).setRotation(contact.q));
        // Only fingertips and the push tab touch tower bars. Links guide the tool
        // pose and must not brace against or knock neighboring supports.
        const groups=contact.name.startsWith('finger-')?0x00040001:0x00040000;
        world.createCollider(RAPIER.ColliderDesc.capsule(contact.halfHeight,contact.radius).setFriction(.35).setCollisionGroups(groups),body);
        robot.push(body);
      }
    }
    // Deconstruction touches the tower only through the tab and the held bar. In
    // construction the arm and fingers meet loose bars, except while the tab pushes.
    const robotTouches=s.phase!=='remove'&&!s.push;
    robot.forEach(body=>body.collider(0).setCollisionGroups(robotTouches?0x00040001:0x00040000));
    const robotStarts=robot.map(body=>({p:body.translation(),q:body.rotation()}));
    // Pushes transfer force through contact; the target remains fully dynamic.
    if(s.push){
      if(!pusher){pusher=world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(...s.push.p).setRotation({x:0,y:Math.sin(s.push.yaw/2),z:0,w:Math.cos(s.push.yaw/2)}));world.createCollider(RAPIER.ColliderDesc.cuboid(TAB.L/2,TAB.H/2,TAB.W/2).setFriction(.62).setCollisionGroups(0x00040002),pusher);}
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

