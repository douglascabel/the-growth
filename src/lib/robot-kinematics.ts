import { END_GRIP, GRIPPER_FINGER_HALF_WIDTH, type Grip, type RigProfile, RIG, ROBOT_BASE, ROBOT_SCALE, solveIK, type Vec, type Quat } from './growth-sim';

export type RobotContact = { name:string; p:Vec; q:Quat; radius:number; halfHeight:number };
export function gripHalfSpan(grip:Grip,carrying:boolean):number { return carrying?grip.closed:grip.open; }
// The contact capsule matches the half-width of each authored jaw mesh.
export const GRIPPER_FINGER_RADIUS = GRIPPER_FINGER_HALF_WIDTH;
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
/** KUKA KR210 pivots in the same model-to-world transform used by the renderer. */
export function robotContacts(tool:Vec,toolYaw:number,carry=false,grip:Grip=END_GRIP,rig:RigProfile=RIG):RobotContact[] {
  const ik=solveIK(tool,toolYaw,rig);
  const shoulder=add(rig.yaw,ry(sub(rig.shoulder,rig.yaw),ik.yaw));
  const elbow=add(shoulder,ry(rx(sub(rig.elbow,rig.shoulder),ik.shoulder),ik.yaw));
  const wrist=add(elbow,ry(rx(sub(rig.wrist,rig.elbow),ik.shoulder+ik.elbow),ik.yaw));
  const sh=world(shoulder),el=world(elbow),wr=world(wrist);
  const palm:Vec=[tool[0],tool[1]+.075,tool[2]];
  // Fingers spread along the gripper's roll axis; the grip decides span and fingertip depth.
  const axis:Vec=[Math.cos(toolYaw),0,-Math.sin(toolYaw)];
  const opening=gripHalfSpan(grip,carry);
  return [
    capsule('shoulder',world([0,.16,0]),sh,.075),
    capsule('upper-arm',sh,el,.065),
    capsule('forearm',el,wr,.045),
    capsule('wrist',wr,palm,.027),
    ...[-1,1].map(side=>{
      const a:Vec=[tool[0]+axis[0]*opening*side,tool[1]+.065,tool[2]+axis[2]*opening*side];
      const b:Vec=[a[0],tool[1]-grip.depth,a[2]];
      return capsule(`finger-${side}`,a,b,GRIPPER_FINGER_RADIUS);
    }),
  ];
}
/** True when the arm links (not the fingers) stay clear of an axis-aligned box around the tower. */
export function armClearMany(tool:Vec,toolYaw:number,boxes:{min:Vec;max:Vec}[],rig:RigProfile=RIG,margin=.01):boolean {
  for(const c of robotContacts(tool,toolYaw,false,END_GRIP,rig).slice(0,4)){
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
export function armClear(tool:Vec,toolYaw:number,box:{min:Vec;max:Vec},rig:RigProfile=RIG,margin=.01):boolean {
  return armClearMany(tool,toolYaw,[box],rig,margin);
}

