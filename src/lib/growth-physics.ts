import RAPIER from '@dimforge/rapier3d-compat';
import { BAR, type Sim, type Dynamics } from './growth-sim';
let ready: Promise<void> | undefined;
export async function createDynamics(): Promise<Dynamics> {
  ready ??= RAPIER.init(); await ready;
  let world: RAPIER.World;
  let bodies = new Map<number,RAPIER.RigidBody>();
  let released = false;
  const reset=()=>{
    world?.free();world=new RAPIER.World({x:0,y:-9.81,z:0});world.numSolverIterations=12;
    world.integrationParameters.lengthUnit=.1;
    world.integrationParameters.normalizedAllowedLinearError=.001;
    world.integrationParameters.normalizedPredictionDistance=.01;
    bodies=new Map();released=false;
    world.createCollider(RAPIER.ColliderDesc.cuboid(20,.1,20).setTranslation(0,-.1,0).setFriction(.65));
  };reset();
  return {reset,step(s,dt){
    for(const [i,b] of s.bars.entries()){
      if(!b.visible)continue;
      let body=bodies.get(i);
      if(!body){
        const desc = released ? RAPIER.RigidBodyDesc.dynamic() : RAPIER.RigidBodyDesc.fixed();
        body=world.createRigidBody(desc.setTranslation(...b.p).setRotation({x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)}).setLinearDamping(.3).setAngularDamping(.4).setCcdEnabled(true));
        world.createCollider(RAPIER.ColliderDesc.cuboid(BAR.L/2,BAR.H/2,BAR.W/2).setContactSkin(0).setDensity(1000).setFriction(.55).setRestitution(.02),body);body.sleep();bodies.set(i,body);
      }
      const moving=s.active===i && s.carry;
      if(moving && !released){released=true;for(const item of bodies.values())item.setBodyType(RAPIER.RigidBodyType.Dynamic,true);}
      if(moving){body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased,true);body.setNextKinematicTranslation({x:b.p[0],y:b.p[1],z:b.p[2]});body.setNextKinematicRotation({x:0,y:Math.sin(b.yaw/2),z:0,w:Math.cos(b.yaw/2)});}
      else if(released && body.bodyType()!==RAPIER.RigidBodyType.Dynamic){body.setBodyType(RAPIER.RigidBodyType.Dynamic,true);body.setTranslation({x:b.p[0],y:b.p[1],z:b.p[2]},true);body.setLinvel({x:0,y:0,z:0},true);body.setAngvel({x:0,y:0,z:0},true);}
    }
    world.timestep=dt;world.step();let fallen=false;
    for(const [i,body] of bodies){
      const b=s.bars[i];if(!b || (s.active===i && s.carry))continue;
      const p=body.translation(),q=body.rotation();b.p=[p.x,p.y,p.z];b.q=q;
      if(b.layer>2&&(Math.abs(q.x)+Math.abs(q.z)>.22||p.y<(b.layer+.5)*BAR.H-.027))fallen=true;
    }
    return fallen;
  }};
}
