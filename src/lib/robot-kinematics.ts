import { BAR, RIG, ROBOT_BASE, ROBOT_SCALE, solveIK, type Vec, type Quat } from './growth-sim';

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
/** Same pivot hierarchy as the supplied OBJ, expressed without a renderer. */
export function robotContacts(tool:Vec,toolYaw:number,carry=false):RobotContact[] {
  const ik=solveIK(tool,toolYaw);
  const shoulder=ry(sub(RIG.shoulder,RIG.yaw),ik.yaw);
  const elbow=add(shoulder,ry(rx(sub(RIG.elbow,RIG.shoulder),ik.shoulder),ik.yaw));
  const wrist=add(elbow,ry(rx(sub(RIG.wrist,RIG.elbow),ik.shoulder+ik.elbow),ik.yaw));
  const sh=world(shoulder),el=world(elbow),wr=world(wrist);
  const palm:Vec=[tool[0],tool[1]+.075,tool[2]];
  const axis:Vec=[Math.sin(toolYaw),0,Math.cos(toolYaw)];
  const opening=BAR.W/2+(carry?.006:.017);
  return [
    capsule('shoulder',world([0,.16,0]),sh,.075),
    capsule('upper-arm',sh,el,.065),
    capsule('forearm',el,wr,.045),
    capsule('wrist',wr,palm,.027),
    ...[-1,1].map(side=>{
      const a:Vec=[tool[0]+axis[0]*opening*side,tool[1]+.065,tool[2]+axis[2]*opening*side];
      const b:Vec=[a[0],tool[1]-.025,a[2]];
      return capsule(`finger-${side}`,a,b,.005);
    }),
  ];
}