import { GRIPPER_FINGER_RADIUS, GRIPPER_OPEN_HALF_GAP, type RigProfile, RIG, ROBOT_BASE, ROBOT_SCALE, WRIST_TO_FINGERS, fingertipDepth, jawPose, solveIK, toolAxes, type Vec, type Quat } from './growth-sim';

export { GRIPPER_FINGER_RADIUS };
export type RobotContact = { name:string; p:Vec; q:Quat; radius:number; halfHeight:number };
const add=(a:Vec,b:Vec):Vec=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const sub=(a:Vec,b:Vec):Vec=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const rx=(v:Vec,a:number):Vec=>[v[0],v[1]*Math.cos(a)-v[2]*Math.sin(a),v[1]*Math.sin(a)+v[2]*Math.cos(a)];
const ry=(v:Vec,a:number):Vec=>[v[0]*Math.cos(a)+v[2]*Math.sin(a),v[1],-v[0]*Math.sin(a)+v[2]*Math.cos(a)];
const world=(v:Vec):Vec=>[ROBOT_BASE.x+v[0]*ROBOT_SCALE,ROBOT_BASE.y+v[1]*ROBOT_SCALE,ROBOT_BASE.z+v[2]*ROBOT_SCALE];
function capsule(name:string,a:Vec,b:Vec,radius:number):RobotContact {
  const d=sub(b,a),length=Math.hypot(...d),v:Vec=length>0?d.map(x=>x/length) as Vec:[0,1,0];
  // Quaternion taking the capsule's local +Y into the link direction.
  const n=Math.hypot(v[2],v[0],1+v[1]);
  const q=n<1e-8?{x:1,y:0,z:0,w:0}:{x:v[2]/n,y:0,z:-v[0]/n,w:(1+v[1])/n};
  return {name,p:a.map((x,i)=>(x+(b[i]??0))/2) as Vec,q,radius,halfHeight:length/2};
}
/** Forward kinematics of the KR 16 pivots, in the same model-to-world transform the renderer uses. */
export function robotJoints(tool:Vec,toolYaw:number,rig:RigProfile=RIG,tilt=0):{a1:Vec;shoulder:Vec;elbow:Vec;wrist:Vec} {
  const ik=solveIK(tool,toolYaw,rig,tilt);
  const shoulder=add(rig.yaw,ry(sub(rig.shoulder,rig.yaw),ik.yaw));
  const elbow=add(shoulder,ry(rx(sub(rig.elbow,rig.shoulder),ik.shoulder),ik.yaw));
  const wrist=add(elbow,ry(rx(sub(rig.wrist,rig.elbow),ik.shoulder+ik.elbow),ik.yaw));
  return {a1:world(rig.yaw),shoulder:world(shoulder),elbow:world(elbow),wrist:world(wrist)};
}
/** Arm links plus the two jaw pads. `halfGap` is the live pad half-gap shared with the renderer. */
export function robotContacts(tool:Vec,toolYaw:number,halfGap=GRIPPER_OPEN_HALF_GAP,rig:RigProfile=RIG,tilt=0):RobotContact[] {
  const {a1,shoulder,elbow,wrist}=robotJoints(tool,toolYaw,rig,tilt);
  // The housing and fingers run from A5 along the tool axis; the jaws open across it.
  const {t,j:axis}=toolAxes(toolYaw,tilt),at=(o:Vec,d:number):Vec=>[o[0]+t[0]*d,o[1]+t[1]*d,o[2]+t[2]*d];
  const advance=jawPose(halfGap).advance;
  const housing=at(wrist,WRIST_TO_FINGERS*ROBOT_SCALE);
  const root=at(wrist,(WRIST_TO_FINGERS+advance)*ROBOT_SCALE),tip=at(tool,fingertipDepth(halfGap)*ROBOT_SCALE);
  const offset=(halfGap+GRIPPER_FINGER_RADIUS)*ROBOT_SCALE;
  return [
    capsule('shoulder',a1,shoulder,.13),
    capsule('upper-arm',shoulder,elbow,.09),
    capsule('forearm',elbow,wrist,.07),
    capsule('wrist',wrist,housing,.047),
    ...[-1,1].map(side=>{
      const shift=(v:Vec):Vec=>[v[0]+axis[0]*offset*side,v[1]+axis[1]*offset*side,v[2]+axis[2]*offset*side];
      return capsule(`finger-${side}`,shift(root),shift(tip),GRIPPER_FINGER_RADIUS*ROBOT_SCALE);
    }),
  ];
}
/** True when the arm links (not the fingers) stay clear of axis-aligned boxes around the tower. */
export function armClearMany(tool:Vec,toolYaw:number,boxes:{min:Vec;max:Vec}[],rig:RigProfile=RIG,margin=.01,tilt=0):boolean {
  for(const c of robotContacts(tool,toolYaw,GRIPPER_OPEN_HALF_GAP,rig,tilt).slice(0,4)){
    const {x,y,z,w}=c.q,d:Vec=[2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)];
    for(let i=0;i<=12;i++){
      const t=(i/6-1)*c.halfHeight,p=c.p.map((v,j)=>v+(d[j]??0)*t);
      for(const box of boxes){
        const gap=Math.hypot(...p.map((v,j)=>Math.max((box.min[j]??0)-v,0,v-(box.max[j]??0))));
        if(gap<c.radius+margin)return false;
      }
    }
  }
  return true;
}
export function armClear(tool:Vec,toolYaw:number,box:{min:Vec;max:Vec},rig:RigProfile=RIG,margin=.01,tilt=0):boolean {
  return armClearMany(tool,toolYaw,[box],rig,margin,tilt);
}
